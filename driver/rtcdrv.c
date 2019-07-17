#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rtc.h"
#include "em_gpio.h"

#define DELAY_SECONDS 3.0
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

	// Toggle LED 0
	//GPIO_PinOutToggle(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
}

/**************************************************************************//**
 * @brief GPIO initialization
 *****************************************************************************/
void initGPIO(void)
{
	//Turn on the clock for the GPIO
	CMU_ClockEnable(cmuClock_GPIO, true);

	//Enable LED0
	//GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull, 0);
}

/**************************************************************************//**
 * @brief RTCC initialization
 *****************************************************************************/
void rtcSetup(void)
{
	// Enable the oscillator for the RTC
	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

	// Turn on the clock for Low Energy clocks
	CMU_ClockEnable(cmuClock_HFLE, true);
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

	// Turn on the RTC clock
	CMU_ClockEnable(cmuClock_RTC, true);

	// Set RTC compare value for RTC 0
	RTC_CompareSet(0, COMPARE_TOP);

	// Allow channel 0 to cause an interrupt
	RTC_IntEnable(RTC_IEN_COMP0);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_EnableIRQ(RTC_IRQn);

	// Configure the RTC settings
	RTC_Init_TypeDef rtc = RTC_INIT_DEFAULT;

	// Initialise RTC with pre-defined settings
	RTC_Init(&rtc);
}