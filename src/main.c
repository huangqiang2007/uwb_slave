#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "timer.h"
#include "udelay.h"
#include "hal-config.h"
#include "main.h"
#include "mainctrl.h"
#include "uartdrv.h"
#include "spidrv.h"
#include "Typedefs.h"


void Clock_config(void)
{
	SystemCoreClockUpdate();

	/*
	 * chose external crystal oscillator as clock source.
	 * */
	CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
	CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
	CMU_OscillatorEnable(cmuSelect_ULFRCO, true, true);
	CMU_ClockEnable(cmuClock_HFLE, true);

	/*
	 * Enable clocks required
	 * */
	//CMU_ClockDivSet(cmuClock_HF, cmuClkDiv_2);
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART0, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
}

int main(void)
{
	/* Chip errata */
	CHIP_Init();

	/*
	 * config needed clock
	 * */
	Clock_config();

	/*
	 * RS422 Uart init for delivering converted data
	 * */
	uartSetup();

	/*
	 * SPI master config
	 * */
	SPIConfig();

	/*
	 * config and start ADC via DMA
	 * */
	ADCStart();

	/*
	 * DW100 wireless device init, to do.
	 * */

	/*
	 * config timer0 and timer1
	 * */
	timer_init();

	UDELAY_Calibrate();
	Delay_ms(500);

	/*
	 * init RTC for LFRCO 32.768KHz
	 * */
	RtcSetup();

	/*
	 * some global configuration
	 * */
	global_init();

	while (1) {
		if (!g_received_cmd && g_Ticks > g_idle_wkup_timeout)
			EMU_EnterEM2(true);
		else if (g_received_cmd && g_Ticks > g_idle_cmd_timeout)
			EMU_EnterEM2(true);
		else {
			ParsePacket();
		}
	}
}





