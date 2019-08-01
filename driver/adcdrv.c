#include <string.h>
#include "em_dma.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "em_core.h"
#include "time.h"
#include "adcdrv.h"
#include "dmactrl.h"
#include "uartdrv.h"
#include <stdbool.h>

/*
 * ADC sample clock
 * */
#define ADC_CLK_1M 1000000

/*
 * drop several samples before ADC is stable.
 * */
#define ADC_IGNORE_CNT 5

DMA_CB_TypeDef dma_adc_cb;

void initADC (void)
{
	// Enable ADC0 clock
	CMU_ClockEnable(cmuClock_ADC0, true);

	// Declare init structs
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

	// Modify init structs and initialize
	init.prescale = ADC_PrescaleCalc(ADC_CLK_1M, 0); // Init to max ADC clock for Series 0

	initSingle.diff       = false;        // single ended
	initSingle.reference  = adcRef2V5;    // internal 2.5V reference
	initSingle.resolution = adcRes8Bit;  // 12-bit resolution

	// Select ADC input. See README for corresponding EXP header pin.
	initSingle.input = adcSingleInputCh4;
	init.timebase = ADC_TimebaseCalc(0);

	ADC_Init(ADC0, &init);
	ADC_InitSingle(ADC0, &initSingle);
}

/*
 * @brief
 *   Configure ADC for scan mode.
 * */
void ADCConfig(void)
{
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;

	CMU_ClockEnable(cmuClock_ADC0, true);

	/*
	* Init common issues for both single conversion and scan mode
	* */
	init.timebase = ADC_TimebaseCalc(0);
	init.prescale = ADC_PrescaleCalc(ADC_CLK_1M, 0);
	init.ovsRateSel = _ADC_CTRL_OVSRSEL_X32;
	ADC_Init(ADC0, &init);

	/*
	* Init for scan sequence use: 7 AD sample channels
	* */
	scanInit.rep = false;
	scanInit.reference = adcRef2V5;
	scanInit.resolution = _ADC_SINGLECTRL_RES_8BIT;

	/*
	 * to do?
	 * */
	scanInit.input = ADC_SCANCTRL_INPUTMASK_CH4;
	ADC_InitScan(ADC0, &scanInit);

	/*
	 * init sample queue with all zero
	 * */
	memset((void *)&g_adcSampleDataQueue, 0x00, sizeof(g_adcSampleDataQueue));
}

void DMA_ADC_callback(unsigned int channel, bool primary, void *user)
{
	static uint8_t drop_cnt = 0;
	uint8_t *precvBuf = NULL;

	/*
	 * drop first some samples since it's likely ADC
	 * is unstable during warming up phase.
	 * */
	if (drop_cnt < ADC_IGNORE_CNT) {
		drop_cnt++;
		goto out;
	}

	g_adcSampleDataQueue.out = g_adcSampleDataQueue.in;
	g_adcSampleDataQueue.in++;
	if (g_adcSampleDataQueue.in == ADC_SAMPLE_BUFFER_NUM)
		g_adcSampleDataQueue.in = 0;

	precvBuf = &(g_adcSampleDataQueue.adc_smaple_data[g_adcSampleDataQueue.in]);

out:
	/* Re-activate the DMA */
	DMA_RefreshPingPong(channel,
						primary,
						false,
						(void *)precvBuf,
						(void *)&(ADC0->SCANDATA),
						ADC_CHNL_NUM - 1,
						false);

	ADC_Start(ADC0, adcStartScan);
}

/*
 *@brief
 *   Configure DMA usage for this application.
 * */
void DMAConfig(void)
{
	DMA_Init_TypeDef dmaInit;
	DMA_CfgDescr_TypeDef descrCfg;
	DMA_CfgChannel_TypeDef chnlCfg;

	CMU_ClockEnable(cmuClock_DMA, true);

	/*
	* Configure general DMA issues
	* */
	dmaInit.hprot = 0;
	dmaInit.controlBlock = dmaControlBlock;
	DMA_Init(&dmaInit);

	/*
	* Configure DMA channel used
	* */
	dma_adc_cb.cbFunc = DMA_ADC_callback;
	dma_adc_cb.userPtr = (void *)&g_adcSampleDataQueue;

	chnlCfg.highPri = false;
	chnlCfg.enableInt = true;
	chnlCfg.select = DMAREQ_ADC0_SCAN;
	chnlCfg.cb = &dma_adc_cb;
	DMA_CfgChannel(ADC_SCAN_DMA_CH, &chnlCfg);

	/*
	* one byte per transfer
	* */
	descrCfg.dstInc = dmaDataInc1;
	descrCfg.srcInc = dmaDataIncNone;
	descrCfg.size = dmaDataSize1;
	descrCfg.arbRate = dmaArbitrate1;
	descrCfg.hprot = 0;
	DMA_CfgDescr(ADC_SCAN_DMA_CH, true, &descrCfg);
	DMA_CfgDescr(ADC_SCAN_DMA_CH, false, &descrCfg);

	// Start DMA
	DMA_ActivatePingPong(
		ADC_SCAN_DMA_CH,
		false,
		(void *)&g_adcSampleDataQueue.adc_smaple_data[g_adcSampleDataQueue.in], // primary destination
		(void *)&(ADC0->SCANDATA), // primary source
		ADC_CHNL_NUM - 1,
		(void *)&g_adcSampleDataQueue.adc_smaple_data[g_adcSampleDataQueue.in], // alternate destination
		(void *)&(ADC0->SCANDATA), // alternate source
		ADC_CHNL_NUM - 1);
}

void ADCStart(void)
{
	ADCConfig();
	DMAConfig();

	/*
	 * Start Scan
	 * */
	ADC_Start(ADC0, adcStartScan);
}

void ADCPoll(void)
{
	uint8_t *precvBuf = NULL;

	g_adcSampleDataQueue.out = g_adcSampleDataQueue.in;
	g_adcSampleDataQueue.in++;
	if (g_adcSampleDataQueue.in == ADC_SAMPLE_BUFFER_NUM)
		g_adcSampleDataQueue.in = 0;

	precvBuf = &(g_adcSampleDataQueue.adc_smaple_data[g_adcSampleDataQueue.in]);

	// Start ADC conversion
	ADC_Start(ADC0, adcStartSingle);

	// Wait for conversion to be complete
	while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));

	// Get ADC result
	precvBuf[0] = ADC_DataSingleGet(ADC0);
}
