#include "em_usart.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "timer.h"
#include "uartdrv.h"
#include "mainctrl.h"
#include <stdbool.h>
#include <string.h>

volatile uint8_t g_slaveStatus = 0;

extern volatile uint32_t g_Ticks;

/*
 * all 5 bytes data add each other and get the 16bits sum.
 * low 8bits store into crc0, high 8bits store into crc1.
 * */
uint16_t CalFrameCRC(uint8_t data[], int len)
{
	int i = 0;
	uint16_t crc_sum = 0;

	for (; i < len; i++)
		crc_sum += data[i];

	return crc_sum;
}

void InitFrame(struct MainCtrlFrame *mainCtrlFr, uint8_t src, uint8_t slave,
		uint8_t type, uint8_t data[])
{
	uint16_t data_crc = 0;

	mainCtrlFr->head0 = 0x55;
	mainCtrlFr->head1 = 0xaa;
	mainCtrlFr->len = 0;
	mainCtrlFr->frameCtrl = ((src & 0x03) < 6) | (slave & 0x7);
	mainCtrlFr->frameType = type & 0xf;
	memcpy(mainCtrlFr->data, data, FRAME_DATA_LEN);

	data_crc = CalFrameCRC(mainCtrlFr->data, FRAME_DATA_LEN);
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

/*
 * try to wake up all slave, woken slaves are marked in 'g_slaveStatus' var.
 * */
void WakeupSlave(void)
{
	int i = 0, ret = -1;
	uint8_t fr_data[FRAME_DATA_LEN];

	memset(fr_data, 0xff, FRAME_DATA_LEN);
	g_wakup_timeout = g_Ticks + WAKUP_DURATION;

	/*
	 * wake up duration is 10 minutes
	 * */
	while (g_Ticks < g_wakup_timeout) {
		for (i = 0; i < SLAVE_NUMS; i++) {
			ret = TalktoSlave(MAIN_NODE, i, ENUM_SAMPLE_SET, fr_data);
			if (ret == 0)
				g_slaveStatus |= (1 << i);
		}

		/*
		 * all slaves are waken up
		 * */
		if ((g_slaveStatus & SLAVE_WKUP_MSK) == SLAVE_WKUP_MSK)
			break;
	}

	/*
	 * if it exists woken slaves, it can begin to fetch data from slaves.
	 * */
	if ((g_slaveStatus & SLAVE_WKUP_MSK) > 0)
		g_slaveWkup = true;
}

/*
 * scan all slaves and fetch sample data.
 * */
void RecvFromSlave(void)
{
	uint16_t crc_sum = 0;
	int i = 0, ret = -1;
	uint8_t fr_data[FRAME_DATA_LEN] = {0};

	memset(&g_RS422DataFr, 0xff, sizeof(struct RS422DataFrame));
	g_RS422DataFr.head0 = 0x33;
	g_RS422DataFr.head1 = 0xcc;
	g_RS422DataFr.len = 0;

	/*
	 * scan each slave and receive sameple data
	 * */
	for (i = 0; i < SLAVE_NUMS; i++) {
		if ((g_slaveStatus & (1 << i)) == (1 << i)) {
			ret = TalktoSlave(MAIN_NODE, i, ENUM_SAMPLE_DATA, fr_data);
			if (ret == 0) {
				crc_sum = CalFrameCRC(g_mainCtrlFr.data, FRAME_DATA_LEN);
				if (g_mainCtrlFr.head0 != 0x55 || g_mainCtrlFr.head1 != 0xaa
					|| g_mainCtrlFr.crc0 != (crc_sum & 0xff) || g_mainCtrlFr.crc1 != ((crc_sum >> 8) & 0xff))
					continue;

				memcpy(&g_RS422DataFr.packets[g_RS422DataFr.len], &g_mainCtrlFr, sizeof(g_mainCtrlFr));
				g_RS422DataFr.len++;
			} else {
				g_slaveStatus &= ~(1 << i);

				/*
				 * if all slave is offline, reset flag 'g_slaveWkup' to begin wakeup logic.
				 * */
				if ((g_slaveStatus & SLAVE_WKUP_MSK) == 0)
					g_slaveWkup = false;
			}
		}
	}

	/*
	 * if it exists valid sample data coming from slaves,
	 * calculate CRC and send them to control computer.
	 * */
	if (g_RS422DataFr.len > 0) {
		crc_sum =  CalFrameCRC((uint8_t *)&g_RS422DataFr.packets[0], sizeof(struct MainCtrlFrame) * SLAVE_NUMS);
		g_RS422DataFr.crc0 = crc_sum & 0xff;
		g_RS422DataFr.crc1 = (crc_sum >> 8) & 0xff;

		uartPutData((uint8_t *)&g_RS422DataFr.packets[0], sizeof(struct RS422DataFrame));
	}
}
