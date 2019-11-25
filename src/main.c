#include <string.h>
#include <timer.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "em_rtc.h"
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
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
	timer_init();
	Delay_ms(5);
	GPIO_PinModeSet(gpioPortA, 1, gpioModePushPull, 1);
	Delay_ms(5);

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
	ADC_SAMPLE_BUFFERDef *pSampleBuf = NULL;
	powerADandUWB(0);
	powerADandUWB(1);
	powerADandUWB(0);;
	initADC();
	while(1){
		pSampleBuf = dequeueSample(&g_adcSampleDataQueue);
	}
}

void rtc_test(void)
{
	RtcSetup();
	while (1) {
		//EMU_EnterEM2(false);
		Delay_ms(1000);
//		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
//		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
//		__WFI();
		EMU_EnterEM2(true);
		RtcSetup();
	}
}

//int main1(void)
//{
//	/* Chip errata */
//	CHIP_Init();
//
//	clockConfig();
//
//	timer_init();
//	Delay_ms(8000);
//
//	rtc_test();
//}
//extern volatile int g_cnt;
//extern int p_cnt;
int main(void)
{

//	uint32_t l_cnt = 0;
//	uint32_t m_cnt = 0;

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
	g_AD_start = false;

	/*
	 * config timer0 and timer1
	 * */
	timer_init();
	Delay_ms(5);

	/*
	 * SPI master config
	 * */
	SPIDMAInit();
	SET_NUM = 5;
	DEV_NUM = 3;
	UWB_Default.subnode_id = DEV_NUM + ((SET_NUM-1)<<2);
	if (DEV_NUM == 1 || DEV_NUM == 2) {
		AD_SHIFT = 8;
		UWB_Default.AD_Samples = 50;
	}

	else if (DEV_NUM == 3 || DEV_NUM == 4) {
		AD_SHIFT = 8;
		UWB_Default.AD_Samples = 5000;
	}
	/*
	 * configure and start ADC via interrupt
	 * */
//	adc_test();

	//initADC();
	ADCConfigForScan();

	/*
	 * init RTC for LFRCO 32.768KHz
	 * */
	RtcSetup();
//	rtc_test();
	/*
	 * DW1000 wireless device init, to do.
	 * */
	GPIO_PinModeSet(gpioPortC, 13, gpioModePushPull, 1);
	Delay_ms(2);
	GPIO_PinModeSet(gpioPortC, 13, gpioModePushPull, 0);

	GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 0);
	Delay_ms(5);
	GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 1);

	dwDeviceInit(&g_dwDev);

	UDELAY_Calibrate();
	Delay_ms(500);
	/*
	 * reset g_idle_wkup_timeout and g_idle_bat_ad_time duration upon boot.
	 * */
	g_idle_wkup_timeout = g_Ticks + IDLE_WKUP_TIMEOUT;
	g_idle_bat_ad_time = g_Ticks + BAT_AD_TIME;

	dwNewReceive(&g_dwDev);
	dwStartReceive(&g_dwDev);

	while (1) {
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
				if (g_received_cmd && g_Ticks > g_idle_cmd_timeout){
					g_AD_start = false;
					g_cur_mode = SLAVE_CMDIDLEMODE;
				}

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
	}
}





