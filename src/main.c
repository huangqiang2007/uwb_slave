#include <string.h>
#include <timer.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "udelay.h"
#include "hal-config.h"
#include "main.h"
#include "mainctrl.h"
#include "uartdrv.h"
#include "spidrv.h"
#include "adcdrv.h"
#include "rtcdrv.h"
#include "Typedefs.h"
#include "libdw1000.h"

void clockConfig(void)
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

void adc_test(void)
{
	while (1)
		ADCPoll();
}

int main(void)
{
	/* Chip errata */
	CHIP_Init();

	/*
	 * some global configuration
	 * */
	globalInit();

	/*
	 * config needed clock
	 * */
	clockConfig();

	/*
	 * power up UWB, power down AD
	 * */
	powerADandUWB(1);

	/*
	 * config timer0 and timer1
	 * */
	timer_init();
	Delay_ms(5);
	GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 0);
	Delay_ms(5);
	GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 1);
	/*
	 * RS422 Uart init for delivering converted data
	 * */
	//uartSetup();


	/*
	 * SPI master config
	 * */
	SPIDMAInit();

	/*
	 * config and start ADC via DMA
	 * */
	initADC();

	/*
	 * DW1000 wireless device init, to do.
	 * */
	dwDeviceInit(&g_dwDev);

	UDELAY_Calibrate();
	Delay_ms(500);

	/*
	 * reset g_idle_wkup_timeout duration upon bootup.
	 * */
	g_idle_wkup_timeout = g_Ticks + IDLE_WKUP_TIMEOUT;
	/*
	 * init RTC for LFRCO 32.768KHz
	 * */
//	RtcSetup();
//	sleepAndRestore();

	while (1) {
		ADCPoll();
		switch (g_cur_mode)
		{
			case SLAVE_IDLEMODE:
				/*
				 * When the system is waken up, but it doesn't receive any CMD from
				 * main node or manual node during 'g_idle_wkup_timeout' time window.
				 * The system will enter into EM2 mode.
				 * */
				if (!g_received_cmd && g_Ticks > g_idle_wkup_timeout)
					g_cur_mode = SLAVE_RTCIDLEMODE;

				/*
				 * When the system had received CMD, but it doesn't received CMD again
				 * during 'g_idle_cmd_timeout' time window. The system will enter into
				 * EM2 mode.
				 * */
				if (g_received_cmd && g_Ticks > g_idle_cmd_timeout)
					g_cur_mode = SLAVE_CMDIDLEMODE;

				break;

			case SLAVE_SAMPLEMODE:
				ParsePacket(&g_dwDev, &g_dwMacFrameSend);
				break;

			case SLAVE_RTCIDLEMODE:
			case SLAVE_CMDIDLEMODE:
				sleepAndRestore();
				break;

			default:
				g_cur_mode = SLAVE_IDLEMODE;
		}

//		switch (g_cur_mode)
//		{
//			case SLAVE_IDLEMODE:
//				/*
//				 * When the system is waken up, but it doesn't receive any CMD from
//				 * main node or manual node during 'g_idle_wkup_timeout' time window.
//				 * The system will enter into EM2 mode.
//				 * */
//				if (!g_received_cmd && g_Ticks > g_idle_wkup_timeout)
//					g_cur_mode = SLAVE_RTCIDLEMODE;
//
//				/*
//				 * When the system had received CMD, but it doesn't received CMD again
//				 * during 'g_idle_cmd_timeout' time window. The system will enter into
//				 * EM2 mode.
//				 * */
//				if (g_received_cmd && g_Ticks > g_idle_cmd_timeout)
//					g_cur_mode = SLAVE_CMDIDLEMODE;
//
//				break;
//
//			case SLAVE_SAMPLEMODE:
//				ParsePacket(&g_dwDev, &g_dwMacFrameSend);
//				break;
//
//			case SLAVE_RTCIDLEMODE:
//			case SLAVE_CMDIDLEMODE:
//				dwLowPowerListenMode(&g_dwDev, 10000, 3000);
//				EMU_EnterEM2(true);
//				/*
//				 * re-init some global vars
//				 * */
//				globalInit();
//				break;
//				//sleepAndRestore();
//
//			default:
//				g_cur_mode = SLAVE_CMDIDLEMODE;
//		}
	}
}





