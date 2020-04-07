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
#include "adc_ping_pong.h"
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

/***************************************************************************//**
 * @brief
 *   Calibrate ADC offset and gain for the specified reference.
 *   Supports currently only single ended gain calibration.
 *   Could easily be expanded to support differential gain calibration.
 *
 * @details
 *   The offset calibration routine measures 0 V with the ADC, and adjust
 *   the calibration register until the converted value equals 0.
 *   The gain calibration routine needs an external reference voltage equal
 *   to the top value for the selected reference. For example if the 2.5 V
 *   reference is to be calibrated, the external supply must also equal 2.5V.
 *
 * @param[in] adc
 *   Pointer to ADC peripheral register block.
 *
 * @param[in] ref
 *   Reference used during calibration. Can be both external and internal
 *   references.
 *
 * @return
 *   The final value of the calibration register, note that the calibration
 *   register gets updated with this value during the calibration.
 *   No need to load the calibration values after the function returns.
 ******************************************************************************/
//uint32_t ADC_Calibration(ADC_TypeDef *adc, ADC_Ref_TypeDef ref)
//{
//  int32_t  sample;
//  uint32_t cal;
//
//  /* Binary search variables */
//  uint8_t high;
//  uint8_t mid;
//  uint8_t low;
//
//  /* Reset ADC to be sure we have default settings and wait for ongoing */
//  /* conversions to be complete. */
//  ADC_Reset(adc);
//
//  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
//  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;
//
//  /* Init common issues for both single conversion and scan mode */
//  init.timebase = ADC_TimebaseCalc(0);
//  /* Might as well finish conversion as quickly as possibly since polling */
//  /* for completion. */
//  /* Set ADC clock to 7 MHz, use default HFPERCLK */
//  ADC_CLK = 1000000;
//  init.prescale = ADC_PrescaleCalc(ADC_CLK, 0);
//
//  /* Set an oversampling rate for more accuracy */
//  init.ovsRateSel = adcOvsRateSel4096;
//  /* Leave other settings at default values */
//  ADC_Init(adc, &init);
//
//  /* Init for single conversion use, measure diff 0 with selected reference. */
//  singleInit.reference = adcRef2V5;
//  singleInit.input     = adcSingleInputCh4;
//  singleInit.acqTime   = adcAcqTime16;
//  singleInit.diff      = false;
//  /* Enable oversampling rate */
//  singleInit.resolution = adcResOVS;
//
//  ADC_InitSingle(adc, &singleInit);
//
//  /* ADC is now set up for offset calibration */
//  /* Offset calibration register is a 7 bit signed 2's complement value. */
//  /* Use unsigned indexes for binary search, and convert when calibration */
//  /* register is written to. */
//  high = 128;
//  low  = 0;
//
//  /* Do binary search for offset calibration*/
//  while (low < high)
//  {
//    /* Calculate midpoint */
//    mid = low + (high - low) / 2;
//
//    /* Midpoint is converted to 2's complement and written to both scan and */
//    /* single calibration registers */
//    cal      = adc->CAL & ~(_ADC_CAL_SINGLEOFFSET_MASK | _ADC_CAL_SCANOFFSET_MASK);
//    cal     |= (mid - 63) << _ADC_CAL_SINGLEOFFSET_SHIFT;
//    cal     |= (mid - 63) << _ADC_CAL_SCANOFFSET_SHIFT;
//    adc->CAL = cal;
//
//    /* Do a conversion */
//    ADC_Start(adc, adcStartSingle);
//
//    /* Wait while conversion is active */
//    while (adc->STATUS & ADC_STATUS_SINGLEACT) ;
//
//    /* Get ADC result */
//    sample = ADC_DataSingleGet(adc);
//
//    /* Check result and decide in which part of to repeat search */
//    /* Calibration register has negative effect on result */
//    if (sample < 0)
//    {
//      /* Repeat search in bottom half. */
//      high = mid;
//    }
//    else if (sample > 0)
//    {
//      /* Repeat search in top half. */
//      low = mid + 1;
//    }
//    else
//    {
//      /* Found it, exit while loop */
//      break;
//    }
//  }
//
//  /* Now do gain calibration, only input and diff settings needs to be changed */
//  adc->SINGLECTRL &= ~(_ADC_SINGLECTRL_INPUTSEL_MASK | _ADC_SINGLECTRL_DIFF_MASK);
//  adc->SINGLECTRL |= (adcSingleInpCh4 << _ADC_SINGLECTRL_INPUTSEL_SHIFT);
//  adc->SINGLECTRL |= (false << _ADC_SINGLECTRL_DIFF_SHIFT);
//
//  /* ADC is now set up for gain calibration */
//  /* Gain calibration register is a 7 bit unsigned value. */
//
//  high = 128;
//  low  = 0;
//
//  /* Do binary search for gain calibration */
//  while (low < high)
//  {
//    /* Calculate midpoint and write to calibration register */
//    mid = low + (high - low) / 2;
//
//    /* Midpoint is converted to 2's complement */
//    cal      = adc->CAL & ~(_ADC_CAL_SINGLEGAIN_MASK | _ADC_CAL_SCANGAIN_MASK);
//    cal     |= mid << _ADC_CAL_SINGLEGAIN_SHIFT;
//    cal     |= mid << _ADC_CAL_SCANGAIN_SHIFT;
//    adc->CAL = cal;
//
//    /* Do a conversion */
//    ADC_Start(adc, adcStartSingle);
//
//    /* Wait while conversion is active */
//    while (adc->STATUS & ADC_STATUS_SINGLEACT) ;
//
//    /* Get ADC result */
//    sample = ADC_DataSingleGet(adc);
//
//    /* Check result and decide in which part to repeat search */
//    /* Compare with a value atleast one LSB's less than top to avoid overshooting */
//    /* Since oversampling is used, the result is 16 bits, but a couple of lsb's */
//    /* applies to the 12 bit result value, if 0xffe is the top value in 12 bit, this */
//    /* is in turn 0xffe0 in the 16 bit result. */
//    /* Calibration register has positive effect on result */
//    if (sample > 0xffe0)
//    {
//      /* Repeat search in bottom half. */
//      high = mid;
//    }
//    else if (sample < 0xffe0)
//    {
//      /* Repeat search in top half. */
//      low = mid + 1;
//    }
//    else
//    {
//      /* Found it, exit while loop */
//      break;
//    }
//  }
//
//  return adc->CAL;
//}

