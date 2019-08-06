#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rtc.h"
#include "em_gpio.h"
#include "timer.h"

#define DELAY_SECONDS 20.0
#define LFXOFREQ      32768
#define COMPARE_TOP   (DELAY_SECONDS * LFXOFREQ - 1)

/**************************************************************************//**
 * @brief RTCC interrupt service routine
 *****************************************************************************/
void RTC_IRQHandler(void)
{
	//Reset counter
	RTC_CounterReset();

	// Clear the interrupt source
	RTC_IntClear(RTC_IFC_COMP0);
}

/**************************************************************************//**
 * @brief GPIO initialization
 *****************************************************************************/
void initGPIO(void)
{
	//Turn on the clock for the GPIO
	CMU_ClockEnable(cmuClock_GPIO, true);
}

/*
 * RTC config for using LFRCO
 * */
void RtcSetup(void)
{
	// Enable the oscillator for the RTC
	CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);

	// Turn on the clock for Low Energy clocks
	//CMU_ClockEnable(cmuClock_HFLE, true);
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);

	// Turn on the RTC clock
	CMU_ClockEnable(cmuClock_RTC, true);

	// Set RTC compare value for RTC 0
	RTC_CompareSet(0, COMPARE_TOP);

	RTC_IntClear(RTC_IFC_COMP0 | RTC_IFC_COMP1);

	// Allow channel 0 to cause an interrupt
	RTC_IntEnable(RTC_IEN_COMP0);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_EnableIRQ(RTC_IRQn);

	// Configure the RTC settings
	RTC_Init_TypeDef rtc = RTC_INIT_DEFAULT;

	// Initialise RTC with pre-defined settings
	RTC_Init(&rtc);
}
