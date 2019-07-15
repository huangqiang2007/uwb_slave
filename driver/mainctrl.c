#include "em_usart.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "timer.h"
#include "mainctrl.h"
#include <stdbool.h>
#include <string.h>

volatile uint8_t g_slaveStatus = 0;

#define SLAVE_NUMS 4

extern volatile uint32_t g_Ticks;

uint16_t CalFrameCRC(struct MainCtrlFrame *mainCtrlFr)
{
	int i = 0;
	uint16_t crc_sum = 0;

	for (; i < FRAME_DATA_LEN; i++)
		crc_sum += mainCtrlFr->data[i];

	return crc_sum;
}

void InitFrame(struct MainCtrlFrame *mainCtrlFr, uint8_t src, uint8_t slave,
		uint8_t type, uint8_t data[])
{
	uint16_t data_crc = 0;

	memset(mainCtrlFr, 0xff, sizeof(*mainCtrlFr));
	mainCtrlFr->head0 = 0x55;
	mainCtrlFr->head1 = 0xaa;
	mainCtrlFr->len = 0;
	mainCtrlFr->frameCtrl = ((src & 0x03) < 6) | (slave & 0x7);
	mainCtrlFr->frameType = type & 0xf;
	memcpy(mainCtrlFr->data, data, FRAME_DATA_LEN);

	data_crc = CalFrameCRC(mainCtrlFr);
	mainCtrlFr->crc0 = data_crc & 0xff;
	mainCtrlFr->crc1 = (data_crc >> 8) & 0xff;
}

/*
 * send a frame to slave and poll the corresponding back token.
 * @src: frame source, main node, manual node or slave node
 * @slave: talk to which slave node
 * @type£º frame type
 *
 * @ret: -1: talk timeout; 0: talk successfully.
 * */
uint8_t TalktoSlave(uint8_t src, uint8_t slave, uint8_t type, uint8_t data[])
{
	int8_t ret = -1;

	InitFrame(&g_mainCtrlFr, src, slave, type, data);

	/*
	 * reset command timeout
	 * */
	g_cmd_feedback_timeout = g_Ticks + CMD_FEEDBACK_TIMEOUT;

	/*
	 * to do:  wireless send logic
	 * */

	while (g_Ticks < g_cmd_feedback_timeout) {

	}

	return ret;
}

void WakeupSlave(void)
{
	int i = 0, ret = -1;
	uint8_t fr_data[FRAME_DATA_LEN];

	memset(fr_data, 0xff, FRAME_DATA_LEN);
	g_wakup_timeout = g_Ticks + WAKUP_DURATION;

	while (g_Ticks < g_wakup_timeout) {
		for (i = 0; i < SLAVE_NUMS; i++) {
			ret = 	TalktoSlave(MAIN_NODE, i, ENUM_SAMPLE_SET, fr_data);
			if (ret == 0)
				g_slaveStatus |= (1 << i);
		}

		/*
		 * all slaves are waken up
		 * */
		if ((g_slaveStatus & SLAVE_WKUP_MSK) == SLAVE_WKUP_MSK)
			break;
	}
}

void RecvFromSlave(void)
{

}
