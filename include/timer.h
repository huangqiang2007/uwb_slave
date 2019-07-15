#include <stdint.h>
#include <stdlib.h>
#ifndef INLCUDE_TIMER_H_
#define INLCUDE_TIMER_H_

/*
 * CMD feedback timeout judge
 *
 * represent 100 * 10ms = 1s timeout
 * */
#define CMD_FEEDBACK_TIMEOUT 100
volatile uint32_t g_cmd_feedback_timeout; /* the count of 10ms unit */

/*
 * wakeup duration
 *
 * 30000 * 10ms = 5 minutes
 * */
#define WAKUP_DURATION 30000
volatile uint32_t g_wakup_timeout; /* the count of 10ms unit */

/*
 * slave does not receive new CMD during the below duration, enter into sleep mode.
 *
 * 30000 * 10ms = 5 minutes
 * */
#define IDLE_JUDGE 30000
volatile uint32_t g_idle_judge;

void setupTimer0(void);
void setupTimer1(void);
void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us);

#endif /* INLCUDE_TIMER_H_ */
