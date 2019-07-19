#include <stdbool.h>
#include <string.h>
#include "em_usart.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "timer.h"
#include "uartdrv.h"
#include "mainctrl.h"
#include "em_core.h"
#include "adcdrv.h"

/*
 * slave device ID 0x0 - 0x3 *
 * */
int8_t SLAVE_ID = 0x0;

volatile uint8_t g_slaveStatus = 0;

void global_init(void)
{
	g_device_id = SLAVE_ID;
	g_received_cmd = false;
	g_idle_wkup_timeout = g_Ticks + IDLE_WKUP_TIMEOUT;
}

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

bool queue_full(struct ReceivedPacketQueue *frameQueue)
{
	if (frameQueue->len == Q_LEN)
		return true;
	else
		return false;
}

bool queue_empty(struct ReceivedPacketQueue *frameQueue)
{
	if (frameQueue->len == 0)
		return true;
	else
		return false;
}

void enqueue_frame(struct ReceivedPacketQueue *frameQueue, struct MainCtrlFrame *mainCtrlFr)
{
	if (queue_full(frameQueue))
		return;

	memcpy((uint8_t *)&(frameQueue->packets[frameQueue->p_in++]),
		(uint8_t *)mainCtrlFr, sizeof(struct MainCtrlFrame));

	frameQueue->len++;
}

struct MainCtrlFrame *dequeue_frame(struct ReceivedPacketQueue *frameQueue)
{
	struct MainCtrlFrame *pmainCtrlFrame = NULL;

	if (queue_empty(frameQueue))
		return NULL;

	pmainCtrlFrame = (struct MainCtrlFrame *)&(frameQueue->packets[frameQueue->p_out++]);

	CORE_CriticalDisableIrq();
	frameQueue->len--;
	CORE_CriticalEnableIrq();

	return pmainCtrlFrame;
}

void form_sample_set_token_frame(struct MainCtrlFrame *pMainCtrlFrame)
{
	uint16_t data_crc;

	//pMainCtrlFrame->frameCtrl &= ~(3 << 6);
	//pMainCtrlFrame->frameCtrl |= SLAVE_NODE;

	pMainCtrlFrame->frameType = ENUM_SAMPLE_SET_TOKEN;

	data_crc = CalFrameCRC(pMainCtrlFrame->data, FRAME_DATA_LEN);
	pMainCtrlFrame->crc0 = data_crc & 0xff;
	pMainCtrlFrame->crc1 = (data_crc >> 8) & 0xff;
}

void form_sample_data_token_frame(struct MainCtrlFrame *pMainCtrlFrame)
{
	uint16_t data_crc;

	//pMainCtrlFrame->frameCtrl &= ~(3 << 6);
	//pMainCtrlFrame->frameCtrl |= SLAVE_NODE;

	pMainCtrlFrame->frameType = ENUM_SAMPLE_DATA_TOKEN;

	/*
	 * to do
	 * */
	pMainCtrlFrame->data[0] = g_adcSampleDataQueue.adc_smaple_data[g_adcSampleDataQueue.out].adc_sample_buffer[0];

	data_crc = CalFrameCRC(pMainCtrlFrame->data, FRAME_DATA_LEN);
	pMainCtrlFrame->crc0 = data_crc & 0xff;
	pMainCtrlFrame->crc1 = (data_crc >> 8) & 0xff;
}

void form_slave_status_token_frame(struct MainCtrlFrame *pMainCtrlFrame)
{
	uint16_t data_crc;

	//pMainCtrlFrame->frameCtrl &= ~(3 << 6);
	//pMainCtrlFrame->frameCtrl |= SLAVE_NODE;

	pMainCtrlFrame->frameType = ENUM_SLAVE_STATUS_TOKEN;

	data_crc = CalFrameCRC(pMainCtrlFrame->data, FRAME_DATA_LEN);
	pMainCtrlFrame->crc0 = data_crc & 0xff;
	pMainCtrlFrame->crc1 = (data_crc >> 8) & 0xff;
}

void form_sleep_token_frame(struct MainCtrlFrame *pMainCtrlFrame)
{
	uint16_t data_crc;

	//pMainCtrlFrame->frameCtrl &= ~(3 << 6);
	//pMainCtrlFrame->frameCtrl |= SLAVE_NODE;

	pMainCtrlFrame->frameType = ENUM_SLAVE_SLEEP_TOKEN;

	data_crc = CalFrameCRC(pMainCtrlFrame->data, FRAME_DATA_LEN);
	pMainCtrlFrame->crc0 = data_crc & 0xff;
	pMainCtrlFrame->crc1 = (data_crc >> 8) & 0xff;
}

int ParsePacket(void)
{
	struct MainCtrlFrame *pMainCtrlFrame = NULL;
	int ret = -1;

	pMainCtrlFrame = dequeue_frame(&g_ReceivedPacketQueue);
	if (!pMainCtrlFrame)
		return ret;
	else
		ret = 0;

	switch (pMainCtrlFrame->frameType)
	{
		case ENUM_SAMPLE_SET:
			form_sample_set_token_frame(pMainCtrlFrame);
			break;

		case ENUM_SAMPLE_DATA:
			form_sample_data_token_frame(pMainCtrlFrame);
			break;

		case ENUM_SLAVE_STATUS:
			form_slave_status_token_frame(pMainCtrlFrame);
			break;

		case ENUM_SLAVE_SLEEP:
			form_sleep_token_frame(pMainCtrlFrame);
			break;

		default:
			break;
	}

	return ret;
}
