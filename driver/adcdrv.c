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
#include "libdw1000.h"

#define ADC_DMA_ENABLE 0

/*
 * ADC sample clock
 * */
#define ADC_CLK 360000

/*
 * drop several samples before ADC is stable.
 * */
#define ADC_IGNORE_CNT 5

DMA_CB_TypeDef dma_adc_cb;

bool sampleQueueFull(AdcSampleDataQueueDef *adcSampleDataQueue)
{
	if (adcSampleDataQueue->samples == Q_LEN)
		return true;
	else
		return false;
}

bool sampleQueueEmpty(AdcSampleDataQueueDef *adcSampleDataQueue)
{
	if (adcSampleDataQueue->samples == 0)
		return true;
	else
		return false;
}

void enqueueSample(AdcSampleDataQueueDef *adcSampleDataQueue, ADC_SAMPLE_BUFFERDef *sampleBuf)
{
	if (sampleQueueFull(adcSampleDataQueue))
		return;

	memcpy((uint8_t *)&(adcSampleDataQueue->adc_smaple_data[adcSampleDataQueue->in++]),
		(uint8_t *)sampleBuf, sizeof(ADC_SAMPLE_BUFFERDef));

	if (adcSampleDataQueue->in == Q_LEN)
		adcSampleDataQueue->in = 0;

	adcSampleDataQueue->samples++;

	/*
	 * update state machine
	 * */
	g_received_cmd = true;
	g_cur_mode = SLAVE_SAMPLEMODE;
}

ADC_SAMPLE_BUFFERDef *getSampelInputbuf(AdcSampleDataQueueDef *adcSampleDataQueue)
{
	ADC_SAMPLE_BUFFERDef *pSampleBuf = NULL;

	if (sampleQueueFull(adcSampleDataQueue))
		return NULL;

	pSampleBuf = (ADC_SAMPLE_BUFFERDef *)&adcSampleDataQueue->adc_smaple_data[adcSampleDataQueue->in];

	return pSampleBuf;
}

ADC_SAMPLE_BUFFERDef *dequeueSample(AdcSampleDataQueueDef *adcSampleDataQueue)
{
	ADC_SAMPLE_BUFFERDef *pSampleBuf = NULL;

	if (sampleQueueEmpty(adcSampleDataQueue))
		return NULL;

	pSampleBuf = (ADC_SAMPLE_BUFFERDef *)&adcSampleDataQueue->adc_smaple_data[adcSampleDataQueue->out++];
	if (adcSampleDataQueue->out == Q_LEN)
		adcSampleDataQueue->out = 0;

	CORE_CriticalDisableIrq();
	adcSampleDataQueue->samples--;
	CORE_CriticalEnableIrq();

	return pSampleBuf;
}

void initADC (void)
{
	// Enable ADC0 clock
	CMU_ClockEnable(cmuClock_ADC0, true);

	// Declare init structs
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

	// Modify init structs and initialize
	init.prescale = ADC_PrescaleCalc(ADC_CLK, 0); // Init to max ADC clock for Series 0

	initSingle.diff       = false;        // single ended
	initSingle.reference  = adcRef2V5;    // internal 2.5V reference
	initSingle.resolution = adcRes8Bit;   // 8-bit resolution
	initSingle.rep = true;

	// Select ADC input. See README for corresponding EXP header pin.
	initSingle.input = adcSingleInputCh4;
	init.timebase = ADC_TimebaseCalc(0);
	init.ovsRateSel = adcOvsRateSel8;

	ADC_Init(ADC0, &init);
	ADC_InitSingle(ADC0, &initSingle);

	// Enable ADC Single Conversion Complete interrupt
	ADC_IntEnable(ADC0, ADC_IEN_SINGLE);

	// Enable ADC interrupts
	NVIC_ClearPendingIRQ(ADC0_IRQn);
	NVIC_EnableIRQ(ADC0_IRQn);

	ADC_Start(ADC0, adcStartSingle);
}

void readADC(void)
{
	static int8_t s_index = 0;
	ADC_SAMPLE_BUFFERDef *pSampleBuf = NULL;
	uint8_t *precvBuf = NULL;

	pSampleBuf  = getSampelInputbuf(&g_adcSampleDataQueue);
	if (!pSampleBuf)
		return;

	precvBuf = (uint8_t *)&pSampleBuf->adc_sample_buffer[0];
	precvBuf[s_index++] = ADC_DataSingleGet(ADC0);
	if (s_index >= FRAME_DATA_LEN) {
		s_index = 0;
		g_adcSampleDataQueue.samples++;
		g_adcSampleDataQueue.in++;
		if (g_adcSampleDataQueue.in == Q_LEN)
			g_adcSampleDataQueue.in = 0;
	}
}

void ADC0_IRQHandler(void)
{
	// Clear the interrupt flag
	ADC_IntClear(ADC0, ADC_IFC_SINGLE);

	readADC();

	// Start next ADC conversion
	ADC_Start(ADC0, adcStartSingle);
}

#if ADC_DMA_ENABLE
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
	//ADCConfig();
	//DMAConfig();
	initADC();

	/*
	 * Start Scan
	 * */
	//ADC_Start(ADC0, adcStartScan);
}
#endif

#if 1
void ADCPoll(void)
{
	static int8_t s_index = 0;
	uint8_t *precvBuf = NULL;

	if (SLAVE_IDNUM == 3 || SLAVE_IDNUM == 4) {
		if (s_index > FRAME_DATA_LEN) {
			s_index = 0;
			g_adcSampleDataQueue.out = g_adcSampleDataQueue.in;
			g_adcSampleDataQueue.in++;
			if (g_adcSampleDataQueue.in == Q_LEN)
				g_adcSampleDataQueue.in = 0;
		}
	} else {
		g_adcSampleDataQueue.out = g_adcSampleDataQueue.in;
		g_adcSampleDataQueue.in++;
		if (g_adcSampleDataQueue.in == Q_LEN)
			g_adcSampleDataQueue.in = 0;
	}
	precvBuf = &(g_adcSampleDataQueue.adc_smaple_data[g_adcSampleDataQueue.in].adc_sample_buffer[0]);

	// Start ADC conversion
	ADC_Start(ADC0, adcStartSingle);

	// Wait for conversion to be complete
	while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));

	// Get ADC result
	if (SLAVE_IDNUM == 3 || SLAVE_IDNUM == 4)
		precvBuf[s_index++] = ADC_DataSingleGet(ADC0);
	else
		precvBuf[0] = ADC_DataSingleGet(ADC0);
}
#else
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
#endif
