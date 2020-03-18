#ifndef __INLCUDE_TIMER_H_
#define __INLCUDE_TIMER_H_

#include <stdint.h>
#include <stdlib.h>

volatile uint32_t g_Ticks;

/*
 * when the system is woken up, but there is no cmd coming
 * during the below timeout duration, the system enters into
 * sleep mode.
 *
 * 300 * 1ms = 300ms
 * */
#define IDLE_WKUP_TIMEOUT 300
volatile uint32_t g_idle_wkup_timeout;

/*
 * when it does not receive cmd in below duration since the last command,
 * the system enters into sleep mode.
 *
 * 300000 * 1ms = 5 minutes
 * */
#define IDLE_CMD_TIMEOUT 300000 //300 second
volatile uint32_t g_idle_cmd_timeout;
/*
 * when it does not receive cmd in below duration since the last command,
 * stops ADC.
 *
 * 1000 * 1ms = 1 s
 * */
#define ADC_IDLE_CMD_TIMEOUT 1000 //300 second
volatile uint32_t g_adc_idle_cmd_timeout;

/*
 * BAT Voltage ADC
 *
 * 300000 * 1ms = 5 minutes
 * */
//#define BAT_AD_TIME 	10000 //10 second
//volatile uint32_t g_idle_bat_ad_time;

extern void setupTimer0(void);
extern void setupTimer1(void);
extern void Delay_us(uint32_t us);
extern void timer_init(void);
extern void Delay_ms(uint32_t ms);

#endif /* INLCUDE_TIMER_H_ */
