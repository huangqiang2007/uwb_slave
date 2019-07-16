#include <stdint.h>
#include <stdlib.h>
#ifndef INLCUDE_TIMER_H_
#define INLCUDE_TIMER_H_

volatile uint32_t g_Ticks;

/*
 * when the system is woken up, but there is no cmd coming
 * during the below timeout duration, the system enters into
 * sleep mode.
 *
 * 100 * 10ms = 1s
 * */
#define IDLE_WKUP_TIMEOUT 100
volatile uint32_t g_idle_wkup_timeout;

/*
 * when it does not receive cmd in below duration since the last command,
 * the system enters into sleep mode.
 *
 * 30000 * 10ms = 5 minutes
 * */
#define IDLE_CMD_TIMEOUT 30000
volatile uint32_t g_idle_cmd_timeout;

void setupTimer0(void);
void setupTimer1(void);
void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us);
void timer_init(void);

#endif /* INLCUDE_TIMER_H_ */
