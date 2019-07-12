#include <stdint.h>
#include <stdlib.h>
#ifndef INLCUDE_TIMER_H_
#define INLCUDE_TIMER_H_


/*
 * represent 50 * 10ms = 500ms timeout
 * */
#define CMD_FEEDBACK_TIMEOUT 50
volatile uint32_t g_cmd_feedback_timeout; /* the count of 10ms unit */

/*
 * 30000 * 10ms = 5 minutes
 * */
#define WAKUP_DURATION 30000
volatile uint32_t g_wakup_timeout; /* the count of 10ms unit */


void setupTimer0(void);
void setupTimer1(void);
void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us);

#endif /* INLCUDE_TIMER_H_ */
