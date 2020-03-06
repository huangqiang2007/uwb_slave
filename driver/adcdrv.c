#include <string.h>
#include "em_dma.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "em_core.h"
#include "em_timer.h"
#include "em_prs.h"
#include "time.h"
#include "adcdrv.h"
#include "dmactrl.h"
#include "uartdrv.h"
#include <stdbool.h>
#include "libdw1000.h"
#include "timer.h"
#include "main.h"
#include "math.h"

#define ADC_DMA_ENABLE 1

/*
 * DMA channel for ADC Scan mode
 * */
#define DMA_CHANNEL 2

/*
 * Two ADC channel for Scan mode
 * */
#define SCAN_ADC_CHNL_NUM 2
#define SCAN_ADC_DMA_LEN  24

/* Defines for ADC */
#define PRS_ADC_CH      5               /* PRS channel */
#define ADC_PRS_CH      adcPRSSELCh5
#define ADC_START_CNT   300 	        /* First PRS TIMER trigger count */
#define ADC_VDD_X1000   3300            /* VDD x 1000 */
#define ADC_AVDD_VFS    (float)3.3      /* AVDD */
#define ADC_12BIT_MAX   4096            /* 2^12 */

#define ADC_DMA_CHANNEL         0
#define ADC_SCAN_CH_NUMBER      3
/*
 * buffer for two ADC channels
 * */
static uint16_t g_primaryResultBuffer[SCAN_ADC_DMA_LEN] = {0}, g_alterResultBuffer[SCAN_ADC_DMA_LEN] = {0};

/*
 * drop several samples before ADC is stable.
 * */
#define ADC_IGNORE_CNT 5

DMA_CB_TypeDef dma_adc_cb;
int ADC_CLK = 0;

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
	ADC_Reset(ADC0);

	// Enable ADC0 clock
	CMU_ClockEnable(cmuClock_ADC0, true);

	// Declare init structs
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

	// Modify init structs and initialize
	if (UWB_Default.AD_Samples == 50){
		ADC_CLK = 867600;
		initSingle.acqTime = adcAcqTime256;
		init.ovsRateSel = adcOvsRateSel64;
	}
	else if (UWB_Default.AD_Samples == 5000){
		ADC_CLK = 1260000;
		initSingle.acqTime = adcAcqTime4;
		init.ovsRateSel = adcOvsRateSel16;
	}
	init.prescale = ADC_PrescaleCalc(ADC_CLK, 0); // Init to max ADC clock for Series 0
	init.lpfMode = adcLPFilterDeCap;
	initSingle.diff       = false;        // single ended
	initSingle.reference  = adcRef2V5;    // internal 2.5V reference
	initSingle.resolution = adcResOVS;   // 8-bit resolution
	initSingle.rep = true;

	// Select ADC input. See README for corresponding EXP header pin.
	initSingle.input = adcSingleInputCh4;
	init.timebase = ADC_TimebaseCalc(0);

	ADC_Init(ADC0, &init);
	ADC_InitSingle(ADC0, &initSingle);

	// Enable ADC Single Conversion Complete interrupt
	ADC_IntEnable(ADC0, ADC_IEN_SINGLE);

	// Enable ADC interrupts
	NVIC_SetPriority(ADC0_IRQn, 0);
	NVIC_ClearPendingIRQ(ADC0_IRQn);
	NVIC_EnableIRQ(ADC0_IRQn);

	ADC_Start(ADC0, adcStartSingle);
}

/****************************************************************************/

void collectSamples(uint8_t dataBuf[])
{
	static int8_t s_index = 0;
	ADC_SAMPLE_BUFFERDef *pSampleBuf = NULL;
	uint8_t *precvBuf = NULL;
	uint32_t sample;
	float vol = 0.0;

	pSampleBuf  = getSampelInputbuf(&g_adcSampleDataQueue);
	if (!pSampleBuf)
		return;

	precvBuf = (uint8_t *)&pSampleBuf->adc_sample_buffer[0];
//	for (int i=0;i<SCAN_ADC_DMA_LEN/2;i+2){
	precvBuf[s_index++] = ((dataBuf[1] << 8 | dataBuf[0]) & 0xFFFF) >> AD_SHIFT;
		//precvBuf[s_index++] = ((dataBuf[i+1] << 8 | dataBuf[i]) & 0xFFFF) >> AD_SHIFT;
//	}
	sample = ((dataBuf[3] << 8 | dataBuf[2]) & 0xFFFF) >> AD_SHIFT;

	if (s_index >= FRAME_DATA_LEN) {
		s_index = 0;
		precvBuf = NULL;
		g_adcSampleDataQueue.samples++;
		g_adcSampleDataQueue.in++;
		if (g_adcSampleDataQueue.in == Q_LEN)
			g_adcSampleDataQueue.in = 0;

		/*
		 * poll battery voltage every 1 second
		 * */
		if (g_Ticks > g_idle_bat_ad_time){
			//pollADCForBattery();
			// Get ADC result

			vol = (float)(sample * 5.0 / 256);

			if (vol > 3.7)
				vol = 3.7;
			else if (vol < 3.0)
				vol = 3.01;

			if (vol > 3.0)
				g_batteryVol = ((vol - 3.0) * 100 + 6) / 7;

			//delay_us = 6500;
			g_idle_bat_ad_time = g_Ticks + BAT_AD_TIME;
		}
	}
}

void DMA_For_ADC_callback(unsigned int channel, bool primary, void *user)
{
	static uint8_t drop_cnt = 0;
	uint8_t *precvBuf = NULL;
	static int cnt = 0;
	//TIMER_CounterGet();

	/*
	 * drop first some samples since it's likely ADC
	 * is unstable during warming up phase.
	 * */
	if (drop_cnt < ADC_IGNORE_CNT) {
		drop_cnt++;
		goto out;
	}

	if (primary == true) {
		precvBuf = g_primaryResultBuffer;
	} else {
		precvBuf = g_alterResultBuffer;
	}
	cnt=cnt+1;
	//collectSamples(precvBuf);

out:
	/* Re-activate the DMA */

	DMA_RefreshPingPong(
		channel,
		primary,
		false,
		NULL,
		NULL,
		3,//(SCAN_ADC_DMA_LEN>>2) - 1,
		false);

	ADC_Start(ADC0, adcStartScan);
}

/*
 *@brief
 *   Configure DMA usage for this application.
 * */
void DMAConfigForADC(void)
{
	DMA_CfgDescr_TypeDef descrCfg;
	DMA_CfgChannel_TypeDef chnlCfg;

	CMU_ClockEnable(cmuClock_DMA, true);

	/*
	* Configure DMA channel used
	* */
	dma_adc_cb.cbFunc = DMA_For_ADC_callback;
	dma_adc_cb.userPtr = NULL;

	chnlCfg.highPri = true;
	chnlCfg.enableInt = true;
	chnlCfg.select = DMAREQ_ADC0_SCAN;
	chnlCfg.cb = &dma_adc_cb;
	DMA_CfgChannel(DMA_CHANNEL, &chnlCfg);

	/*
	* one byte per transfer
	* */
	descrCfg.dstInc = dmaDataInc2;
	descrCfg.srcInc = dmaDataIncNone;
	descrCfg.size = dmaDataSize2;
	descrCfg.arbRate = dmaArbitrate8;
	descrCfg.hprot = 0;
	DMA_CfgDescr(DMA_CHANNEL, true, &descrCfg);
	DMA_CfgDescr(DMA_CHANNEL, false, &descrCfg);

	// Start DMA
	DMA_ActivatePingPong(
		DMA_CHANNEL,
		false,
		(void *)&g_primaryResultBuffer, // primary destination
		(void *)&(ADC0->SCANDATA), // primary source
		3,//(SCAN_ADC_DMA_LEN>>2) - 1,
		//0,
		(void *)&g_alterResultBuffer, // alternate destination
		(void *)&(ADC0->SCANDATA), // alternate source
		3//(SCAN_ADC_DMA_LEN>>2) - 1);
	);
}

/*
 * @brief
 *   Configure ADC for scan mode.
 * */
void ADCConfigForScan(void)
{
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;

	ADC_Reset(ADC0);

	CMU_ClockEnable(cmuClock_ADC0, true);

	/*
	* Init common issues for both single conversion and scan mode
	* */
	init.timebase = ADC_TimebaseCalc(0);
//	init.ovsRateSel = _ADC_CTRL_OVSRSEL_X32;

	// Modify init structs and initialize
	if (UWB_Default.AD_Samples == 50){
		ADC_CLK = 867600;
		scanInit.acqTime = adcAcqTime256;
		init.ovsRateSel = adcOvsRateSel64;
	}
	else if (UWB_Default.AD_Samples == 5000){
		ADC_CLK = 1260000;
		scanInit.acqTime = adcAcqTime4;
		init.ovsRateSel = adcOvsRateSel16;
	}
	init.prescale = ADC_PrescaleCalc(ADC_CLK, 0); // Init to max ADC clock for Series 0
	init.lpfMode = adcLPFilterDeCap;
	ADC_Init(ADC0, &init);

	/*
	* Init for scan sequence use: 7 AD sample channels
	* */
	scanInit.rep = false;
	scanInit.diff = false;        // single ended
	scanInit.resolution = adcResOVS;   // 8-bit resolution
	scanInit.reference = adcRef2V5;
	//scanInit.resolution = _ADC_SINGLECTRL_RES_8BIT;
//	scanInit.input = ADC_SCANCTRL_INPUTMASK_CH4 | ADC_SCANCTRL_INPUTMASK_CH7;
	scanInit.input = ADC_SCANCTRL_INPUTMASK_CH4;
	ADC_InitScan(ADC0, &scanInit);

	/*
	 * DAM config for ADC
	 * */
	DMAConfigForADC();

	/*
	 * trigger ADC
	 * */
	ADC_Start(ADC0, adcStartScan);
}

/****************************************************************************/

void pollADCForBattery (void)
{
	uint32_t sample;
	float vol = 0.0;

	ADC_Reset(ADC0);

	// Enable ADC0 clock
	CMU_ClockEnable(cmuClock_ADC0, true);

	// Declare init structs
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

	// Modify init structs and initialize
	init.prescale = ADC_PrescaleCalc(1000000, 0); // Init to max ADC clock for Series 0

	initSingle.diff       = false;        // single ended
	initSingle.reference  = adcRef2V5;    // internal 2.5V reference
	initSingle.resolution = adcRes8Bit;  // 12-bit resolution
	initSingle.acqTime = adcAcqTime4;

	// Select ADC input. See README for corresponding EXP header pin.
	initSingle.input = adcSingleInputCh7;
	init.timebase = ADC_TimebaseCalc(0);

	ADC_Init(ADC0, &init);
	ADC_InitSingle(ADC0, &initSingle);

	// Start ADC conversion
	ADC_Start(ADC0, adcStartSingle);

	// Wait for conversion to be complete
	while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));

	// Get ADC result
	sample = ADC_DataSingleGet(ADC0);
	vol = (float)(sample * 5.0 / 256);

	if (vol > 3.7) vol = 3.7;
	else if (vol < 3.0) vol = 3.01;

	if (vol > 3.0)
		g_batteryVol = ((vol - 3.0) * 100 + 6) / 7;

	/*
	 * re-init ADC for sensor sample
	 * */
	initADC();
}

void readADC(void)
{
	static int8_t s_index = 0;
	ADC_SAMPLE_BUFFERDef *pSampleBuf = NULL;
	uint8_t *precvBuf = NULL;


	pSampleBuf  = getSampelInputbuf(&g_adcSampleDataQueue);
	if (!pSampleBuf)
		return;
	//	temp = ADC_DataSingleGet(ADC0) & 0x00FF;
	//	temp = temp >> 7;

	precvBuf = (uint8_t *)&pSampleBuf->adc_sample_buffer[0];
	precvBuf[s_index++] = ((ADC_DataSingleGet(ADC0) & 0x0FFF) >> AD_SHIFT);

	if (s_index >= FRAME_DATA_LEN) {
		s_index = 0;
		g_adcSampleDataQueue.samples++;
		g_adcSampleDataQueue.in++;
		if (g_adcSampleDataQueue.in == Q_LEN)
			g_adcSampleDataQueue.in = 0;
		delay_us = 8000;
		/*
		 * poll battery voltage every 1 second
		 * */
		if (g_Ticks > g_idle_bat_ad_time){
			pollADCForBattery();
			delay_us = 6500;
			g_idle_bat_ad_time = g_Ticks + BAT_AD_TIME;
		}
	}
}

void ADC0_IRQHandler(void)
{
	static int cnt = 0;
	// Clear the interrupt flag
	ADC_IntClear(ADC0, ADC_IFC_SINGLE);
	cnt++;

	readADC();

	// Start next ADC conversion
	//ADC_Start(ADC0, adcStartSingle);
}

/***************************************************************************//**
 * @brief
 *   Use TIMER as PRS producer for ADC trigger.
 *
 * @details
 *   This example triggers an ADC conversion every time that TIMER0 overflows.
 *   TIMER0 sends a one HFPERCLK cycle high pulse through the PRS on each
 *   overflow and the ADC does a single conversion.
 *
 * @note
 *   The ADC consumes pulse signals which is the same signal produced by the
 *   TIMER. In this case, there is no edge detection needed, and the PRS leaves
 *   the incoming signal unchanged.
 ******************************************************************************/
void prsTimerAdc(void)
{

  /* Enable necessary clocks */
  CMU_ClockEnable(cmuClock_ADC0, true);
  CMU_ClockEnable(cmuClock_PRS, true);
  //CMU_ClockEnable(cmuClock_TIMER0, true);

  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;

  /* Initialize TIMER0 */
  timerInit.enable = false;                     /* Do not start after init */
  timerInit.prescale = timerPrescale8;        /* Overflow after aprox 1s */
  TIMER_Init(TIMER0, &timerInit);

  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  /* Init common settings for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
	// Modify init structs and initialize
	if (UWB_Default.AD_Samples == 50){
		ADC_CLK = 867600;
		initSingle.acqTime = adcAcqTime256;
//		init.ovsRateSel = adcOvsRateSel64;
	}
	else if (UWB_Default.AD_Samples == 5000){
		ADC_CLK = 1000000;
		initSingle.acqTime = adcAcqTime16;
//		init.ovsRateSel = adcOvsRateSel16;
	}
  init.prescale = ADC_PrescaleCalc(ADC_CLK, 0);
  init.lpfMode = adcLPFilterDeCap;

  /* Initialize ADC single sample conversion */
  initSingle.prsSel = ADC_PRS_CH;       /* Select PRS channel */
  initSingle.reference = adcRef2V5;     /* VDD or AVDD as ADC reference */
  initSingle.input = adcSingleInputCh7;   /* VDD as ADC input */
  initSingle.resolution = adcRes12Bit;   // 8-bit resolution
//  initSingle.rep = true;
  initSingle.prsEnable = true;          /* PRS enable */

  ADC_Init(ADC0, &init);
  ADC_InitSingle(ADC0, &initSingle);

  /* Enable ADC Interrupt when Single Conversion Complete */
  ADC_IntEnable(ADC0, ADC_IEN_SINGLE);
  NVIC_ClearPendingIRQ(ADC0_IRQn);
  NVIC_EnableIRQ(ADC0_IRQn);

  /* Select TIMER0 as source and timer overflow as signal */
  PRS_SourceSignalSet(PRS_ADC_CH, PRS_CH_CTRL_SOURCESEL_TIMER0, PRS_CH_CTRL_SIGSEL_TIMER0OF, prsEdgeOff);

  TIMER_CounterSet(TIMER0, ADC_START_CNT);
  TIMER_Enable(TIMER0, true);

//  TIMER_Reset(TIMER0);
//  ADC_Reset(ADC0);
//  prsDemoExit();
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
	init.prescale = ADC_PrescaleCalc(ADC_CLK, 0);
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

	precvBuf = (uint8_t *)(&g_adcSampleDataQueue.adc_smaple_data[g_adcSampleDataQueue.in]);

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

#if ADC_DMA_ENABLE
void ADCPoll(void)
{
	static int8_t s_index = 0;
	uint8_t *precvBuf = NULL;

	if (UWB_Default.subnode_id == 3 || UWB_Default.subnode_id == 4) {
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
	if (UWB_Default.subnode_id == 3 || UWB_Default.subnode_id == 4)
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
