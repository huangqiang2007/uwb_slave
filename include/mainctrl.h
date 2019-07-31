#ifndef INLCUDE_MAINCTRL_H_
#define INLCUDE_MAINCTRL_H_

#include <stdint.h>
#include <stdlib.h>
#include "libdw1000Types.h"

#define Q_LEN 4

enum {SLAVE_IDLEMODE = 0, SLAVE_RTCIDLEMODE, SLAVE_CMDIDLEMODE, SLAVE_SAMPLEMODE};

/*
 * the IDs for 4 slaves
 * [0, 1, 2, 3]
 * */
int8_t g_device_id;

/*
 * slave waken up flag
 * */
#define SLAVE0_WKUP (1 << 0)
#define SLAVE1_WKUP (1 << 1)
#define SLAVE2_WKUP (1 << 2)
#define SLAVE3_WKUP (1 << 3)
#define SLAVE_WKUP_MSK (0x0f)

/*
 * CMD frame information
 * */
#define MAIN_NODE (0 << 6)
#define MANUAL_NODE (1 << 6)
#define SLAVE_NODE (2 << 6)

#define SLAVE_NODE_0 0x00
#define SLAVE_NODE_1 0x01
#define SLAVE_NODE_2 0x02
#define SLAVE_NODE_3 0x03
#define SLAVE_BROADCAST 0x07

/*
 * frame type
 * */
enum {
	ENUM_SAMPLE_SET = 0x01,
	ENUM_SAMPLE_SET_TOKEN,
	ENUM_SAMPLE_DATA,
	ENUM_SAMPLE_DATA_TOKEN,
	ENUM_SLAVE_STATUS,
	ENUM_SLAVE_STATUS_TOKEN,
	ENUM_SLAVE_SLEEP,
	ENUM_SLAVE_SLEEP_TOKEN
};

typedef struct {
	int8_t devId;
	uint16_t panId;
	uint16_t srcId;
} devInfo_t;

#define FRAME_DATA_LEN 5

struct MainCtrlFrame {
	uint8_t head0; //0x55
	uint8_t head1; //0xaa
	uint8_t len; // data len
	uint8_t serial; // serial num: 0-255
	uint8_t frameCtrl;
	uint8_t frameType;
	uint8_t data[FRAME_DATA_LEN];
	uint8_t crc0; // crc[7:0]
	uint8_t crc1; // crc[15:8]
};

struct BackTokenFrame {
	uint8_t head0; //0x55
	uint8_t head1; //0xaa
	uint8_t len; // data len
	uint8_t serial; // serial num: 0-255
	uint8_t frameCtrl;
	uint8_t frameType;
	uint8_t data[FRAME_DATA_LEN];
	uint8_t crc0; // crc[7:0]
	uint8_t crc1; // crc[15:8]
};

/*
 * the queue for storing packets coming from
 * main control node and manual node.
 * */
struct ReceivedPacketQueue {
	volatile int8_t len; // data packets num
	int8_t p_in, p_out;
	struct MainCtrlFrame packets[Q_LEN]; // data packet
};

struct MainCtrlFrame g_mainCtrlFr, g_recvSlaveFr;
dwMacFrame_t g_dwMacFrameSend, g_dwMacFrameRecv;
struct ReceivedPacketQueue g_ReceivedPacketQueue;

volatile uint8_t g_cur_mode;

/*
 * true: received new cmd
 * false: received no cmd
 * */
bool g_received_cmd;

bool g_dataRecvDone;

extern void globalInit(void);
extern void sleepAndRestore(void);
extern uint16_t CalFrameCRC(uint8_t data[], int len);
extern int ParsePacket(dwDevice_t *dev, dwMacFrame_t *dwMacFrame);
extern void enqueueFrame(struct ReceivedPacketQueue *frameQueue, struct MainCtrlFrame *mainCtrlFr);
extern void powerADandUWB(uint8_t master);

#endif
