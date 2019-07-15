#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "timer.h"
#include "udelay.h"
#include "hal-config.h"
#include "main.h"
#include "mainctrl.h"
#include "uartdrv.h"
#include "Typedefs.h"

extern volatile int8_t g_slaveWkup;

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
	g_slaveWkup = false;

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
	 * config one DMA channel for transferring ADC sample results
	 * to specific RAM buffer.
	 * */
	//DMAConfig();

	/*
	 * DW100 wireless device init, to do.
	 * */

  	UDELAY_Calibrate();
  	Delay_ms(500);

	while (1) {
		if (!g_slaveWkup)
			WakeupSlave();
		else
			RecvFromSlave();

		Delay_ms(2);
	}
}





