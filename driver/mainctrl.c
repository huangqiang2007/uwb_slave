#include "em_usart.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "timer.h"
#include "mainctrl.h"
#include <stdbool.h>

#define SLAVE0_WKUP (1 << 0)
#define SLAVE1_WKUP (1 << 1)
#define SLAVE2_WKUP (1 << 2)
#define SLAVE3_WKUP (1 << 3)
#define SLAVE_WKUP_MSK (0x0f)

volatile uint8_t g_slaveStatus = 0;

#define SLAVE_NUMS 4

extern volatile uint32_t g_Ticks;

void initFrame(struct MainCtrlFrame *mainCtrlFr, uint8_t src, uint8_t slave, uint8_t type)
{
	memset(mainCtrlFr, 0x00, sizeof(*mainCtrlFr));
	mainCtrlFr->head0 = 0x55;
	mainCtrlFr->head1 = 0xaa;
	mainCtrlFr->len = 0;
	mainCtrlFr->frameCtrl = ((src & 0x03) < 6) | (slave & 0x7);
	mainCtrlFr->frameType = type & 0xf;
}

uint8_t CheckSlave(uint8_t src, uint8_t slave, uint8_t type)
{
	initFrame(&g_mainCtrlFr, src, slave, type);

	/*
	 * to do:  wireless send loggic
	 * */

	while (g_Ticks < g_cmd_feedback_timeout) {

	}
}

void WakeupSlave(void)
{
	g_wakup_timeout = g_Ticks + WAKUP_DURATION;

	g_cmd_feedback_timeout = g_Ticks + CMD_FEEDBACK_TIMEOUT;

	while (g_Ticks < g_wakup_timeout) {

	}

}

void RecvFromSlave(void)
{

}
