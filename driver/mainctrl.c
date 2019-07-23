#include <stdbool.h>
#include <string.h>
#include <timer.h>
#include "em_usart.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rtc.h"
#include "em_gpio.h"
#include "uartdrv.h"
#include "mainctrl.h"
#include "em_core.h"
#include "adcdrv.h"
#include "libdw1000.h"

/*
 * slave device ID 0x0 - 0x3 *
 * */
int8_t SLAVE_ID = 0x0;

volatile uint8_t g_slaveStatus = 0;

void globalInit(void)
{
	g_device_id = SLAVE_ID;
	g_received_cmd = false;
	g_cur_mode = SLAVE_IDLEMODE;
	memset(&g_recvSlaveFr, 0x00, sizeof(g_recvSlaveFr));
	memset(&g_dwMacFrameRecv, 0x00, sizeof(g_dwMacFrameRecv));
	memset(&g_dwMacFrameSend, 0x00, sizeof(g_dwMacFrameSend));
	memset((void *)&g_dwDev, 0x00, sizeof(g_dwDev));
}

void sleepAndRestore(void)
{
	/*
	 * reenable RTC
	 * */
	RTC_CounterReset();

	EMU_EnterEM2(true);

	/*
	 * re-init some global vars
	 * */
	globalInit();

	/*
	 * reset g_idle_wkup_timeout duration upon bootup.
	 * */
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

bool queueFull(struct ReceivedPacketQueue *frameQueue)
{
	if (frameQueue->len == Q_LEN)
		return true;
	else
		return false;
}

bool queueEmpty(struct ReceivedPacketQueue *frameQueue)
{
	if (frameQueue->len == 0)
		return true;
	else
		return false;
}

void enqueueFrame(struct ReceivedPacketQueue *frameQueue, struct MainCtrlFrame *mainCtrlFr)
{
	if (queueFull(frameQueue))
		return;

	memcpy((uint8_t *)&(frameQueue->packets[frameQueue->p_in++]),
		(uint8_t *)mainCtrlFr, sizeof(struct MainCtrlFrame));

	frameQueue->len++;

	/*
	 * update state machine
	 * */
	g_received_cmd = true;
	g_cur_mode = SLAVE_SAMPLEMODE;
}

struct MainCtrlFrame *dequeueFrame(struct ReceivedPacketQueue *frameQueue)
{
	struct MainCtrlFrame *pmainCtrlFrame = NULL;

	if (queueEmpty(frameQueue))
		return NULL;

	pmainCtrlFrame = (struct MainCtrlFrame *)&(frameQueue->packets[frameQueue->p_out++]);

	CORE_CriticalDisableIrq();
	frameQueue->len--;
	CORE_CriticalEnableIrq();

	return pmainCtrlFrame;
}

void sendTokenFrame(dwDevice_t *dev, dwMacFrame_t *dwMacFrame, struct MainCtrlFrame *pMainCtrlFrame)
{
	uint16_t pan_id = PAN_ID1, source_addr = SLAVE_ADDR1 + ((pMainCtrlFrame->frameCtrl & 0x0f) - 1);

	dwTxBufferFrameEncode(&g_dwMacFrameSend, 0, 1, pan_id, source_addr,
		source_addr, (uint8_t *)pMainCtrlFrame, sizeof(*pMainCtrlFrame));
	dwSendData(dev, (uint8_t *)dwMacFrame, sizeof(*dwMacFrame));
}

void form_sample_set_token_frame(dwDevice_t *dev, dwMacFrame_t *dwMacFrame, struct MainCtrlFrame *pMainCtrlFrame)
{
	uint16_t data_crc;

	//pMainCtrlFrame->frameCtrl &= ~(3 << 6);
	//pMainCtrlFrame->frameCtrl |= SLAVE_NODE;

	pMainCtrlFrame->frameType = ENUM_SAMPLE_SET_TOKEN;

	data_crc = CalFrameCRC(pMainCtrlFrame->data, FRAME_DATA_LEN);
	pMainCtrlFrame->crc0 = data_crc & 0xff;
	pMainCtrlFrame->crc1 = (data_crc >> 8) & 0xff;

	sendTokenFrame(dev, dwMacFrame, pMainCtrlFrame);
}

void form_sample_data_token_frame(dwDevice_t *dev, dwMacFrame_t *dwMacFrame, struct MainCtrlFrame *pMainCtrlFrame)
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

	sendTokenFrame(dev, dwMacFrame, pMainCtrlFrame);
}

void form_slave_status_token_frame(dwDevice_t *dev, dwMacFrame_t *dwMacFrame, struct MainCtrlFrame *pMainCtrlFrame)
{
	uint16_t data_crc;

	//pMainCtrlFrame->frameCtrl &= ~(3 << 6);
	//pMainCtrlFrame->frameCtrl |= SLAVE_NODE;

	pMainCtrlFrame->frameType = ENUM_SLAVE_STATUS_TOKEN;

	data_crc = CalFrameCRC(pMainCtrlFrame->data, FRAME_DATA_LEN);
	pMainCtrlFrame->crc0 = data_crc & 0xff;
	pMainCtrlFrame->crc1 = (data_crc >> 8) & 0xff;

	sendTokenFrame(dev, dwMacFrame, pMainCtrlFrame);
}

void form_sleep_token_frame(dwDevice_t *dev, dwMacFrame_t *dwMacFrame, struct MainCtrlFrame *pMainCtrlFrame)
{
	uint16_t data_crc;

	//pMainCtrlFrame->frameCtrl &= ~(3 << 6);
	//pMainCtrlFrame->frameCtrl |= SLAVE_NODE;

	pMainCtrlFrame->frameType = ENUM_SLAVE_SLEEP_TOKEN;

	data_crc = CalFrameCRC(pMainCtrlFrame->data, FRAME_DATA_LEN);
	pMainCtrlFrame->crc0 = data_crc & 0xff;
	pMainCtrlFrame->crc1 = (data_crc >> 8) & 0xff;

	sendTokenFrame(dev, dwMacFrame, pMainCtrlFrame);
}

int ParsePacket(dwDevice_t *dev, dwMacFrame_t *dwMacFrame)
{
	struct MainCtrlFrame *pMainCtrlFrame = NULL;
	int ret = -1;

	pMainCtrlFrame = dequeueFrame(&g_ReceivedPacketQueue);
	if (!pMainCtrlFrame)
		return ret;
	else
		ret = 0;

	switch (pMainCtrlFrame->frameType)
	{
		case ENUM_SAMPLE_SET:
			form_sample_set_token_frame(dev, dwMacFrame, pMainCtrlFrame);
			break;

		case ENUM_SAMPLE_DATA:
			form_sample_data_token_frame(dev, dwMacFrame, pMainCtrlFrame);
			break;

		case ENUM_SLAVE_STATUS:
			form_slave_status_token_frame(dev, dwMacFrame, pMainCtrlFrame);
			break;

		case ENUM_SLAVE_SLEEP:
			form_sleep_token_frame(dev, dwMacFrame, pMainCtrlFrame);
			break;

		default:
			break;
	}

	CORE_CriticalDisableIrq();
	if (g_ReceivedPacketQueue.len == 0) {
		g_idle_cmd_timeout = g_Ticks + IDLE_CMD_TIMEOUT;
		g_cur_mode = SLAVE_IDLEMODE;
	}
	CORE_CriticalEnableIrq();

	return ret;
}
