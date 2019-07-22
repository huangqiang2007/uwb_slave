#ifndef _SPIDRV_H_
#define _SPIDRV_H_

#include "em_usart.h"
//#include "main.h"

#define SPI_CLK 200000

USART_TypeDef *usart_spi;

extern void SPIConfig(uint32_t spiclk);
extern void SPISendByte(uint8_t ucData);
extern int SPISendNbytes(uint8_t *str, int n);
extern uint8_t SPIRecvNBytes(uint8_t dst[], int n);

#endif
