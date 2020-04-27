#include <string.h>
#include "em_dma.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "em_core.h"
#include "em_prs.h"
#include "em_timer.h"
#include "time.h"
#include "adcdrv.h"
#include "dmactrl.h"
#include "uartdrv.h"
#include <stdbool.h>
#include "libdw1000.h"
#include "timer.h"
#include "main.h"
#include "math.h"
#include "spidrv.h"

#define ADC_DMA_ENABLE 0
#define ADC_INT_ENABLE 1
#define ADC_PRS_ENABLE 1
/* Defines for ADC */
#define PRS_ADC_CH      5               /* PRS channel */
#define ADC_PRS_CH      adcPRSSELCh5
#define ADC_START_CNT   0 	        	/* First PRS TIMER trigger count */

/*
 * ADC sample clock
 * */
//#define ADC_CLK 1150000

/*
 * drop several samples before ADC is stable.
 * */
//#define ADC_IGNORE_CNT 5

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
	//adcSampleDataQueue->out
	pSampleBuf = (ADC_SAMPLE_BUFFERDef *)&adcSampleDataQueue->adc_smaple_data[adcSampleDataQueue->out++];
	if (adcSampleDataQueue->out == Q_LEN)
		adcSampleDataQueue->out = 0;

//	CORE_CriticalDisableIrq();
	adcSampleDataQueue->samples--;
//	CORE_CriticalEnableIrq();

	return pSampleBuf;
}


#if ADC_INT_ENABLE
void initADC (void)
{
	ADC_Reset(ADC0);
	ADC_int_active = 0;

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
//		ADC_CLK = 3000000;
		initSingle.acqTime = adcAcqTime4;
		init.ovsRateSel = adcOvsRateSel16;
	}
	init.prescale = ADC_PrescaleCalc(ADC_CLK, 0); // Init to max ADC clock for Series 0
	init.lpfMode = adcLPFilterDeCap; //Init Filter Type

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


void ADC0_IRQHandler(void)
{
//	static int cnt = 0;
	// Clear the interrupt flag
	ADC_IntClear(ADC0, ADC_IFC_SINGLE);
//	cnt++;

	readADC();
	//ADC_int_active = 1;
	// Start next ADC conversion
	//ADC_Start(ADC0, adcStartSingle);
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
	precvBuf[s_index++] = (ADC_DataSingleGet(ADC0) & 0xFFFF) >> 8;

	/*
	 * poll battery voltage every 1 second
	 * */
//	if (g_Ticks > g_idle_bat_ad_time){
//		pollADCForBattery();
//		//delay_us = 6500;
//		g_idle_bat_ad_time = g_Ticks + BAT_AD_TIME;
//	}

	if (s_index >= FRAME_DATA_LEN) {
		s_index = 0;
		g_adcSampleDataQueue.samples++;
		g_adcSampleDataQueue.in++;
		if (g_adcSampleDataQueue.in == Q_LEN)
			g_adcSampleDataQueue.in = 0;
		//delay_us = 8500;

	}
}
#endif


#if ADC_PRS_ENABLE
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
//  CMU_ClockEnable(cmuClock_ADC0, true);
//  CMU_ClockEnable(cmuClock_PRS, true);
  //CMU_ClockEnable(cmuClock_TIMER0, true);

//  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
//
//  /* Initialize TIMER0 */
//  timerInit.enable = false;                     /* Do not start after init */
//  timerInit.prescale = timerPrescale8;        /* Overflow after aprox 1s */
//  TIMER_Init(TIMER0, &timerInit);

  /* Enabling PingPong Transfer*/
//  DMA_ActivatePingPong(DMA_CHANNEL_ADC,
//						  false,
//						  (void *)ramBufferAdcData1,
//						  (void *)&(ADC0->SINGLEDATA),
//						  ADCSAMPLES - 1,
//						  (void *)ramBufferAdcData2,
//						  (void *)&(ADC0->SINGLEDATA),
//						  ADCSAMPLES - 1);

//  g_spiTransDes.sendActive = false;
  ADC0_Reset();
  TIMER_Enable(TIMER0, false);

  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  /* Init common settings for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
	// Modify init structs and initialize
	if (UWB_Default.AD_Samples == 50){
//		ADC_CLK = 867600;
		initSingle.acqTime = adcAcqTime256;
		init.ovsRateSel = adcOvsRateSel64;
		init.prescale = 27;
	}
	else if (UWB_Default.AD_Samples == 5000){
//		ADC_CLK = 1000000;
//		init.prescale = ADC_PrescaleCalc(ADC_CLK, 0);
		initSingle.acqTime = adcAcqTime4;
		init.ovsRateSel = adcOvsRateSel16;
		init.prescale = 17;
	}
//  init.prescale = ADC_PrescaleCalc(ADC_CLK, 0);
//  init.lpfMode = adcLPFilterDeCap;

  /* Initialize ADC single sample conversion */
  initSingle.prsSel = ADC_PRS_CH;       /* Select PRS channel */
  initSingle.reference = adcRef2V5;     /* VDD or AVDD as ADC reference */
  initSingle.input = adcSingleInputCh4;   /* VDD as ADC input */
//  initSingle.resolution = adcRes12Bit;   // 8-bit resolution
  initSingle.resolution = adcResOVS;   // 8-bit resolution
  initSingle.prsEnable = true;          /* PRS enable */

  ADC_Init(ADC0, &init);
  ADC_InitSingle(ADC0, &initSingle);

  /* Enable ADC Interrupt when Single Conversion Complete */
  NVIC_SetPriority(ADC0_IRQn, 0);
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
#endif

#if ADC_DMA_ENABLE
void collectSamples(uint16_t dataBuf[])
{
	static int8_t s_index = 0;
//	ADC_SAMPLE_BUFFERDef *pSampleBuf = NULL;
//	uint8_t *precvBuf = NULL;
//	float vol = 0.0;

//	pSampleBuf  = getSampelInputbuf(&g_adcSampleDataQueue);
//	if (!pSampleBuf)
//		return;
//
//	precvBuf = (uint8_t *)&pSampleBuf->adc_sample_buffer[0];
//	for (int i=0;i<FRAME_DATA_LEN;i++){
//		precvBuf[i] = (dataBuf[i] & 0xFFFF) >> 4;
//	}

	for (int i=0;i<ADCSAMPLES;i++){
		g_adcSampleDataQueue.adc_smaple_data[g_adcSampleDataQueue.in].adc_sample_buffer[s_index*ADCSAMPLES+i] = (dataBuf[i] & 0xFFFF) >> 8;
//		g_adcSampleDataQueue.adc_smaple_data[g_adcSampleDataQueue.in].adc_sample_buffer[i] = (dataBuf[i] & 0xFFFF) >> 4;
	}
	s_index++;
//	precvBuf = NULL;
	if (s_index == 2){
		s_index = 0;
		g_adcSampleDataQueue.samples++;
		g_adcSampleDataQueue.in++;
		if (g_adcSampleDataQueue.in == Q_LEN)
			g_adcSampleDataQueue.in = 0;
	}
}
#endif

void ADC0_Reset(void){
	ADC_Reset(ADC0);
	g_adcSampleDataQueue.in = 0;
	g_adcSampleDataQueue.out = 0;
	g_adcSampleDataQueue.samples = 0;
	for (int i=0; i < Q_LEN; i++){
		memset(&g_adcSampleDataQueue.adc_smaple_data[i], 0x00, FRAME_DATA_LEN);
	}
}

void ADCPoll(void)
{
	uint8_t *precvBuf = NULL;

	g_adcSampleDataQueue.out = g_adcSampleDataQueue.in;
	g_adcSampleDataQueue.in++;
	if (g_adcSampleDataQueue.in == ADC_SAMPLE_BUFFER_NUM)
		g_adcSampleDataQueue.in = 0;

	precvBuf = (uint8_t *)&(g_adcSampleDataQueue.adc_smaple_data[g_adcSampleDataQueue.in]);

	// Start ADC conversion
	ADC_Start(ADC0, adcStartSingle);

	// Wait for conversion to be complete
	while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));

	// Get ADC result
	precvBuf[0] = ADC_DataSingleGet(ADC0);
}

void pollADCForBattery (void)
{
	uint32_t sample;
	float vol = 0.0;
//	ADC_Reset(ADC0);
	// Enable ADC0 clock
//	CMU_ClockEnable(cmuClock_ADC0, true);

	// Declare init structs
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

	// Modify init structs and initialize
	init.prescale = ADC_PrescaleCalc(1000000, 0); // Init to max ADC clock for Series 0

	initSingle.diff       = false;        // single ended
	initSingle.reference  = adcRef2V5;    // internal 2.5V reference
	initSingle.resolution = adcRes8Bit;  // 12-bit resolution
	initSingle.acqTime = adcAcqTime16;

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
//	initADC();
}
