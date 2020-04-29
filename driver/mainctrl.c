#include <stdbool.h>
#include <string.h>
#include <timer.h>
#include "em_usart.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rtc.h"
#include "em_gpio.h"
#include "em_adc.h"
#include "uartdrv.h"
#include "mainctrl.h"
#include "em_core.h"
#include "adcdrv.h"
#include "libdw1000.h"
#include "em_timer.h"
/*
 * slave device ID 0x0 - 0x3 *
 * */
int8_t SLAVE_ID = 0x0;
bool free_frm_status = false;
uint32_t TE_reg = 0;
uint32_t TE_std = 0;
uint32_t sync_cnt = 0;
uint16_t cnt = 0;

devInfo_t g_devInfo = {
		.devId = SLAVE_ADDR1,
		.panId = PAN_ID1,
		.srcId = SLAVE_ADDR1,
};



void globalInit(void)
{
	g_device_id = g_devInfo.devId;
	g_received_cmd = false;
	g_cur_mode = SLAVE_IDLEMODE;
	g_idle_cmd_timeout = g_Ticks + IDLE_CMD_TIMEOUT;
	memset(&g_recvSlaveFr, 0x00, sizeof(g_recvSlaveFr));
//	memset(&g_dwMacFrameRecv, 0x00, sizeof(g_dwMacFrameRecv));
//	memset(&g_dwMacFrameSend, 0x00, sizeof(g_dwMacFrameSend));
	memset((void *)&g_dwDev, 0x00, sizeof(g_dwDev));

	frm_cnt = 0;
	slave_adc_index = 0;
	s_index_chg = false;
	free_frm_status = false;
	g_dataRecv_time = 0;
	g_dataSend_time = 0;
}

void sleepAndRestore(void)
{
	/*
	 * reenable RTC
	 * */
	dwIdle(&g_dwDev);
	Delay_ms(2);
	dwSetSleep(&g_dwDev);
	Delay_ms(2);
	RTC_CounterReset();
	Delay_ms(2);

	/*
	 * chose HF RCO as clock source.
	 * */
	CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
	CMU_OscillatorEnable(cmuOsc_HFXO, false, false);
	/*
	 * power down external crystal oscillator, OPA and reset.
	 * */
	powerADandUWB(0);
	GPIO_PinModeSet(gpioPortA, 1, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortB, gpioPortB_11, gpioModeInputPullFilter, 0);


	EMU_EnterEM2(true);

	/*
	 * chose HF external crystal oscillator as clock source.
	 * */
	Delay_ms(1);
	/*
	 * power up external crystal oscillator.
	 * */
	GPIO_PinModeSet(gpioPortA, 1, gpioModePushPull, 1);
	Delay_ms(2);
	CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
	CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);
	Delay_ms(2);
	GPIO_PinModeSet(gpioPortC, 13, gpioModePushPull, 1);
	Delay_ms(2);
	GPIO_PinModeSet(gpioPortC, 13, gpioModePushPull, 0);
	Delay_ms(2);
	GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 1);
	/*
	 * re-init some global vars
	 * */
	globalInit();
	ADC0_Reset();
	dwDeviceInit(&g_dwDev);
	Delay_ms(2);
//	UDELAY_Calibrate();
//	Delay_ms(5);
	/*
	 * reset g_idle_wkup_timeout duration upon bootup.
	 * */
	dwNewReceive(&g_dwDev);
	dwStartReceive(&g_dwDev);
	g_Ticks = 0;
	g_idle_wkup_timeout = g_Ticks + IDLE_WKUP_TIMEOUT;
	//g_idle_bat_ad_time = g_Ticks + BAT_AD_TIME;
}

int32_t time_sync(struct MainCtrlFrame *pMainCtrlFrame){

	int32_t t0, t1;
//	int32_t TE_dif_std_temp;
	static uint32_t TE_mean = 0;
	static uint32_t TE_temp = 0;
//	static int32_t TE_dif_std=0;
//	static uint32_t TE_reg_temp=0;
	int32_t TE_dif_temp;
	static int32_t TE_dif=0;

	TE = TIMER_CounterGet(TIMER1);
	if (sync_cnt>2){
		TE_dif_temp = TE - TE_std;

		if (TE_dif_temp > 40 || TE_dif_temp < -40){
			TE = TE_temp;
		}
	}

	TE_mean = TE_mean + TE;
	cnt++;
	TE_temp = TE;

	if (cnt == 32){
		cnt = 0;
		sync_cnt++;
		TE_reg = TE_mean >> 5;
		TE_mean = 0;
		if (sync_cnt<2)
			TE_std = TE_reg;
		else{
			if (sync_cnt==2){
				TE_std = TE_reg;//pMainCtrlFrame->frameCtrl_blank[2] + (pMainCtrlFrame->adcIndex<<8);
			}
//			else{
//				TE_dif_std_temp = TE_reg - TE_std - TE_dif_std;
//				if (TE_dif_std_temp > 12 || TE_dif_std_temp < -12)
//					TE_reg = TE_reg_temp;
//			}
//			TIMER_Enable(TIMER0, false);
//			TIMER_Enable(TIMER1, false);
			TE_dif = TE_reg - TE_std;
			if (TE_dif > 0){
				//TE_dif = TE_reg - TE_std;
				t0 = TIMER_CounterGet(TIMER0);
				t1 = TIMER_CounterGet(TIMER1);

				if (t0 > TE_dif){
					TIMER_CounterSet(TIMER0,t0-TE_dif);
					if (t1 >= TE_dif){
						TIMER_CounterSet(TIMER1,t1-TE_dif);
					}
					else{
						TIMER_CounterSet(TIMER1,MS_COUNT - TE_dif + t1);
						g_Ticks--;
					}
				}
			}
			else{
				//TE_dif = TE_std - TE_reg;
				t0 = TIMER_CounterGet(TIMER0) - TE_dif;
				t1 = TIMER_CounterGet(TIMER1) - TE_dif;

				if (t0 < US200_COUNT - 1){
					TIMER_CounterSet(TIMER0, t0);
					if (t1 <= MS_COUNT - 1){
						TIMER_CounterSet(TIMER1,t1);
					}
					else{
						TIMER_CounterSet(TIMER1,t1-MS_COUNT);
						g_Ticks++;
					}
				}
			}
//			TIMER_Enable(TIMER0, true);
//			TIMER_Enable(TIMER1, true);
		}
//		TE_dif_std = TE_reg - TE_std;
//		TE_reg_temp = TE_reg;
	}

	return TE_dif;
}

uint32_t time_get(uint8_t TOA_Low, uint8_t TOA_High){
	uint32_t TOA_R;
	uint32_t TOA_T;
	uint32_t TE;

	TOA_R = g_Ticks * MS_COUNT + TIMER_CounterGet(TIMER1) - g_dataSend_time;
	TOA_T = TOA_Low + (TOA_High<<8);
	TE = (TOA_T - TOA_R)>>1;
//	TE = TIMER_CounterGet(TIMER1);
	return TE;
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

	mainCtrlFr->head0 = 0xaa;
	mainCtrlFr->head1 = 0x55;
	mainCtrlFr->len = 0;
	mainCtrlFr->frameCtrl = ((src & 0x03) < 6) | (slave & 0x7);
	mainCtrlFr->frameType = type & 0xf;
	memcpy(mainCtrlFr->data, data, FRAME_DATA_LEN);

	data_crc = CalFrameCRC(mainCtrlFr->data, FRAME_DATA_LEN);
	mainCtrlFr->crc0 = data_crc & 0xff;
	//mainCtrlFr->crc1 = (data_crc >> 8) & 0xff;
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

	if (frameQueue->p_in == Q_LEN)
		frameQueue->p_in = 0;


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

	if (frameQueue->p_out == Q_LEN)
		frameQueue->p_out = 0;

	//CORE_CriticalDisableIrq();
	frameQueue->len--;
	//CORE_CriticalEnableIrq();

	return pmainCtrlFrame;
}

void sendTokenFrame(dwDevice_t *dev, struct MainCtrlFrame *pMainCtrlFrame, uint32_t resp_time_us)
{
//	uint16_t pan_id = PAN_ID1, source_addr = SLAVE_ADDR1, dest_addr = CENTER_ADDR1;
//
//	dwTxBufferFrameEncode(&g_dwMacFrameSend, 1, 0, pan_id, dest_addr,
//		source_addr, (uint8_t *)pMainCtrlFrame, sizeof(*pMainCtrlFrame));
//	dwSendData(dev, (uint8_t *)dwMacFrame, sizeof(*dwMacFrame));
	dwSendData(dev, (uint8_t *)pMainCtrlFrame, sizeof(*pMainCtrlFrame), resp_time_us);
}

void form_sample_set_token_frame(dwDevice_t *dev, struct MainCtrlFrame *pMainCtrlFrame)
{
	uint16_t data_crc;

	//pMainCtrlFrame->frameCtrl &= ~(3 << 6);
	//pMainCtrlFrame->frameCtrl |= SLAVE_NODE;

	pMainCtrlFrame->frameType = ENUM_SAMPLE_SET_TOKEN;

	data_crc = CalFrameCRC(pMainCtrlFrame->data, FRAME_DATA_LEN);
	pMainCtrlFrame->crc0 = data_crc & 0xff;
	//pMainCtrlFrame->crc1 = (data_crc >> 8) & 0xff;
	frm_cnt = 0;

	sendTokenFrame(dev, pMainCtrlFrame, RECV_TRUNON_TIME);
}

void form_sample_data_token_frame(dwDevice_t *dev, struct MainCtrlFrame *pMainCtrlFrame)
{
	ADC_SAMPLE_BUFFERDef *pSampleBuf = NULL;
	uint16_t data_crc;
	int32_t dif = 0;

	//TE = time_get(pMainCtrlFrame->frameCtrl_blank[0], pMainCtrlFrame->frameCtrl_blank[1]);
//	uint32_t TOA_R = g_Ticks * MS_COUNT + TIMER_CounterGet(TIMER1) - g_dataSend_time;
//	uint32_t TOA_T = pMainCtrlFrame->frameCtrl_blank[0]+ (pMainCtrlFrame->frameCtrl_blank[1]<<8);
//	TE = (TOA_T - TOA_R)>>1;

	pMainCtrlFrame->frameType = ENUM_SAMPLE_DATA_TOKEN;
	//adc_index = pMainCtrlFrame->adcIndex;
	pSampleBuf = dequeueSample(&g_adcSampleDataQueue);

	if (!pSampleBuf) {
		memset(pMainCtrlFrame->data, 0xff, FRAME_DATA_LEN);
		pMainCtrlFrame->len = 0;
		free_frm_status = true;
		//delay_us = 8500;
	} else {
		memcpy(pMainCtrlFrame->data, &pSampleBuf->adc_sample_buffer[0], FRAME_DATA_LEN);
		pMainCtrlFrame->len = FRAME_LEN;
		frm_cnt++;
		free_frm_status = false;
	}
	pMainCtrlFrame->frameType |= (g_batteryVol & 0x0f) << 4;
	//pMainCtrlFrame->adcIndex = slave_adc_index;
	dif = time_sync(pMainCtrlFrame);

	pMainCtrlFrame->frameCtrl_blank[0] = dif;
	pMainCtrlFrame->frameCtrl_blank[1] = dif>>8;
	pMainCtrlFrame->frameCtrl_blank[2] = dif>>16;//TE_std;
	pMainCtrlFrame->adcIndex = dif>>24;//TE_std>>8;
	pMainCtrlFrame->serial = frm_cnt;
	data_crc = CalFrameCRC(pMainCtrlFrame->data, FRAME_DATA_LEN);
	pMainCtrlFrame->crc0 = data_crc & 0xff;
	//pMainCtrlFrame->crc1 = (data_crc >> 8) & 0xff;

	sendTokenFrame(dev, pMainCtrlFrame, RECV_TRUNON_TIME);
	//g_dataSend_time = g_Ticks * MS_COUNT + TIMER_CounterGet(TIMER1);
	g_dataRecv_time = g_Ticks;
	//TE_temp = TE;
}

void form_repeat_data_token_frame(dwDevice_t *dev, struct MainCtrlFrame *pMainCtrlFrame)
{
	ADC_SAMPLE_BUFFERDef *pSampleBuf = NULL;
	uint16_t data_crc;
	uint32_t times;
	int32_t dif = 0;
	//uint32_t TE;

	times = g_Ticks - g_dataRecv_time;
//	uint32_t TOA_R = g_Ticks * MS_COUNT + TIMER_CounterGet(TIMER1) - g_dataSend_time;
//	uint32_t TOA_T = pMainCtrlFrame->frameCtrl_blank[0]+ (pMainCtrlFrame->frameCtrl_blank[1]<<8);
//	TE = (TOA_T - TOA_R)>>1;
//	TE = TIMER_CounterGet(TIMER1);

	pMainCtrlFrame->frameType = ENUM_REPEAT_DATA_TOKEN;
	//adc_index = pMainCtrlFrame->adcIndex;
	if (times <= 23 && (!free_frm_status)){
		if (g_adcSampleDataQueue.out == 0)
			g_adcSampleDataQueue.out = Q_LEN - 1;
		else
			g_adcSampleDataQueue.out--;

		if (g_adcSampleDataQueue.samples < Q_LEN - 1)
			g_adcSampleDataQueue.samples++;
	}
	pSampleBuf = dequeueSample(&g_adcSampleDataQueue);

	if (!pSampleBuf) {
		memset(pMainCtrlFrame->data, 0xff, FRAME_DATA_LEN);
		pMainCtrlFrame->len = 0;
		//delay_us = 8500;
	} else {
		memcpy(pMainCtrlFrame->data, &pSampleBuf->adc_sample_buffer[0], FRAME_DATA_LEN);
		pMainCtrlFrame->len = FRAME_LEN;
		if (times > 23 || free_frm_status){
			frm_cnt++;
		}
	}
	pMainCtrlFrame->frameType |= (g_batteryVol & 0x0f) << 4;

	dif = time_sync(pMainCtrlFrame);

	pMainCtrlFrame->frameCtrl_blank[0] = dif;
	pMainCtrlFrame->frameCtrl_blank[1] = dif>>8;

	pMainCtrlFrame->frameCtrl_blank[2] = dif>>16;
	pMainCtrlFrame->adcIndex = dif>>24;
	pMainCtrlFrame->serial = frm_cnt;
	data_crc = CalFrameCRC(pMainCtrlFrame->data, FRAME_DATA_LEN);
	pMainCtrlFrame->crc0 = data_crc & 0xff;
	//pMainCtrlFrame->crc1 = (data_crc >> 8) & 0xff;

	sendTokenFrame(dev, pMainCtrlFrame, RECV_TRUNON_TIME);
	//g_dataSend_time = g_Ticks * MS_COUNT + TIMER_CounterGet(TIMER1);
}

void form_slave_status_token_frame(dwDevice_t *dev, struct MainCtrlFrame *pMainCtrlFrame)
{
	uint16_t data_crc;

	//pMainCtrlFrame->frameCtrl &= ~(3 << 6);
	//pMainCtrlFrame->frameCtrl |= SLAVE_NODE;

	pMainCtrlFrame->frameType = ENUM_SLAVE_STATUS_TOKEN;
	pMainCtrlFrame->data[0] = g_cur_mode;

	data_crc = CalFrameCRC(pMainCtrlFrame->data, FRAME_DATA_LEN);
	pMainCtrlFrame->crc0 = data_crc & 0xff;
	//pMainCtrlFrame->crc1 = (data_crc >> 8) & 0xff;

	sendTokenFrame(dev, pMainCtrlFrame, RECV_TRUNON_TIME);
}

void form_sleep_token_frame(dwDevice_t *dev, struct MainCtrlFrame *pMainCtrlFrame)
{
	uint16_t data_crc;

	//pMainCtrlFrame->frameCtrl &= ~(3 << 6);
	//pMainCtrlFrame->frameCtrl |= SLAVE_NODE;

	pMainCtrlFrame->frameType = ENUM_SLAVE_SLEEP_TOKEN;
	pMainCtrlFrame->frameType |= (g_batteryVol & 0x0f) << 4;
	data_crc = CalFrameCRC(pMainCtrlFrame->data, FRAME_DATA_LEN);
	pMainCtrlFrame->crc0 = data_crc & 0xff;
	//pMainCtrlFrame->crc1 = (data_crc >> 8) & 0xff;

	sendTokenFrame(dev, pMainCtrlFrame, 50);
}

void Sync_Slave_Bat_Get(dwDevice_t *dev, struct MainCtrlFrame *pMainCtrlFrame)
{
	//int ADC_calibration_value = 0;
	//CORE_CriticalDisableIrq();
	TIMER_Enable(TIMER1, false);
	TIMER_CounterSet(TIMER1, 0);
	g_Ticks = 0;
	TIMER_Enable(TIMER1, true);

	pollADCForBattery();
		//ADC_calibration_value = ADC_Calibration(ADC0, adcRef2V5);
	prsTimerAdc();
	//CORE_CriticalEnableIrq();
	dwNewReceive(dev);
	dwStartReceive(dev);
	sync_cnt=0;
	frm_cnt = 0;
}

//int p_cnt=0;

int ParsePacket(dwDevice_t *dev)
{
	struct MainCtrlFrame *pMainCtrlFrame = NULL;
	int ret = -1;

	pMainCtrlFrame = dequeueFrame(&g_ReceivedPacketQueue);
	if (!pMainCtrlFrame)
		return ret;
	else
		ret = 0;
//	if (pMainCtrlFrame->frameType == 0x03){
//		p_cnt += 1;
//	}
	switch (pMainCtrlFrame->frameType)
	{
		case ENUM_SAMPLE_SET:
			form_sample_set_token_frame(dev, pMainCtrlFrame);
			if (!g_AD_start){
				g_AD_start = true;
				powerADandUWB(1);
			}
			break;

		case ENUM_SLAVE_SYNC:
			Sync_Slave_Bat_Get(dev, pMainCtrlFrame);
			break;

		case ENUM_SAMPLE_DATA:
			form_sample_data_token_frame(dev, pMainCtrlFrame);
			break;

		case ENUM_REPEAT_DATA:
			form_repeat_data_token_frame(dev, pMainCtrlFrame);
			break;

		case ENUM_SLAVE_STATUS:
			form_slave_status_token_frame(dev, pMainCtrlFrame);
			break;

		case ENUM_SLAVE_SLEEP:
			if ((g_recvSlaveFr.frameCtrl & 0xff) == UWB_Default.subnode_id){
				form_sleep_token_frame(dev, pMainCtrlFrame);
				Delay_ms(100);
				g_AD_start = false;
				g_cur_mode = SLAVE_RTCIDLEMODE;
				return ret;
			}
			dwNewReceive(dev);
			dwStartReceive(dev);
			break;
		default:
			break;
	}


	//CORE_CriticalDisableIrq();
	if (g_ReceivedPacketQueue.len == 0) {
		g_idle_cmd_timeout = g_Ticks + IDLE_CMD_TIMEOUT;
		g_adc_idle_cmd_timeout = g_Ticks + ADC_IDLE_CMD_TIMEOUT;
		g_cur_mode = SLAVE_IDLEMODE;
	}
	//CORE_CriticalEnableIrq();

	return ret;
}

void powerADandUWB(uint8_t master)
{
	if (master == 1) {
		GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);
	} else {
		GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 0);
	}
}
