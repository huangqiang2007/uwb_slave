#ifndef INCLUDE_MAIN_H_
#define INCLUDE_MAIN_H_

#include <stdint.h>

/*
 * DMA channel for ADC scan mode
 * */
#define ADC_SCAN_DMA_CH    	2

/*
 * SPI Rx & Tx DMA channel
 * */
#define SPI_RX_DMA_CH	0
#define SPI_TX_DMA_CH	1


#define PORT_UART           						 		 gpioPortE
#define PIN_RX            									 11
#define PIN_TX            									 10
#endif /* INCLUDE_MAIN_H_ */
