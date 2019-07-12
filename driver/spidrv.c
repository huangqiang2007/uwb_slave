#include "em_usart.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "spidrv.h"
#include <stdbool.h>

#define SPI_CLK 200000

/*
 * @brief
 *   Configure SPI master
 * */
void SPIConfig(void)
{
	USART_InitSync_TypeDef SPI_init = USART_INITSYNC_DEFAULT;

	usart_spi = USART1;

	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART1, true);

	SPI_init.baudrate = SPI_CLK;
	USART_InitSync(usart_spi, &SPI_init);

	GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 1); //tx
	GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 1);    //rx
	GPIO_PinModeSet(gpioPortB, 7, gpioModePushPull, 1); //clk
	GPIO_PinModeSet(gpioPortB, 8, gpioModePushPull, 1); //cs

	usart_spi->ROUTE |= USART_ROUTE_CSPEN | USART_ROUTE_CLKPEN |
		USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_LOCATION_LOC1;
}

/*
 * send one byte once
 * */
void SPISendByte(uint8_t ucData)
{
	USART_Tx(usart_spi, ucData);
}

/*
 * send n bytes via SPI
 * */
int SPISendNbytes(uint8_t *str, int n)
{
	int8_t i = 0;

	while (i < n) {
		SPISendByte(str[i]);
		i++;
	}

	return i;
}

/*
 * receive one byte
 * */
uint8_t SPIRecvByte(void)
{
	return USART_Rx(usart_spi);
}

/*
 * receive n bytes via SPI
 * */
uint8_t SPIRecvNBytes(uint8_t dst[], int n)
{
	int8_t i = 0;

	while (i < n) {
		dst[i] = SPIRecvByte();
		i++;
	}

	return i;
}
