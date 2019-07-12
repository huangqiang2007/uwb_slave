#include <stdint.h>
#include <stdlib.h>
#ifndef INLCUDE_MAINCTRL_H_
#define INLCUDE_MAINCTRL_H_

struct MainCtrlFrame {
	uint8_t head0; //0x55
	uint8_t head1; //0xaa
	uint8_t len; // data len
	uint8_t serial; // serial num: 0-255
	uint8_t frameCtrl;
	uint8_t frameType;
	uint8_t data[5];
	uint8_t crc0; // crc[7:0]
	uint8_t crc1; // crc[15:8]
};

struct MainCtrlFrame g_mainCtrlFr;

struct BackTokenFrame {
	uint8_t head0; //0x55
	uint8_t head1; //0xaa
	uint8_t len; // data len
	uint8_t serial; // serial num: 0-255
	uint8_t frameCtrl;
	uint8_t frameType;
	uint8_t data[5];
	uint8_t crc0; // crc[7:0]
	uint8_t crc1; // crc[15:8]
};

struct BackTokenFrame g_backTokenFr;

#endif
