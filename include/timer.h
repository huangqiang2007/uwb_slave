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
 * 200 * 1ms = 200ms
 * */
#define IDLE_WKUP_TIMEOUT 200
volatile uint32_t g_idle_wkup_timeout;

/*
 * when it does not receive cmd in below duration since the last command,
 * the system enters into sleep mode.
 *
 * 300000 * 1ms = 5 minutes
 * */
#define IDLE_CMD_TIMEOUT 300000 //300 second
volatile uint32_t g_idle_cmd_timeout;

extern void setupTimer0(void);
extern void setupTimer1(void);
extern void Delay_us(uint32_t us);
extern void timer_init(void);
extern void Delay_ms(uint32_t ms);

#endif /* INLCUDE_TIMER_H_ */
