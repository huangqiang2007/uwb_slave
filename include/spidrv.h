#ifndef _SPIDRV_H_
#define _SPIDRV_H_

#include "em_usart.h"
//#include "main.h"

#define SPI_CLK 200000

#define UWB_MAX_SPI_LEN 150
typedef struct {
	int txlen;
	int rxlen;
	uint8_t txBuf[UWB_MAX_SPI_LEN];
	uint8_t *rxBuf;
	volatile bool recvDone;
	volatile bool sendDone;
} SPITransDes_t;

SPITransDes_t g_spiTransDes;

USART_TypeDef *usart_spi;

extern void SPIConfig(uint32_t spiclk);
extern void SPISendByte(uint8_t ucData);
extern int SPISendNbytes(uint8_t *str, int n);
extern uint8_t SPIRecvNBytes(uint8_t dst[], int n);

extern uint8_t TxBuffer1[], TxBuffer2[], TxBuffer3[], RxBuffer[];
void initTransferDma(void);
void initReceiveDma(void);
void spiTransferForRead(SPITransDes_t *spiTransDes, uint8_t *txbuf, int txlen,
		uint8_t *rxbuf, int rxlen);
void spiTransferForWrite(SPITransDes_t *spiTransDes, uint8_t *txbuf, int txlen);
void SPIDMAInit(void);

#endif
