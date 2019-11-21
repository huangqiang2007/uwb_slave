#ifndef INCLUDE_MAIN_H_
#define INCLUDE_MAIN_H_

#include <stdint.h>

typedef struct UWB_Config {
  uint8_t  subnode_id;
  uint32_t AD_Samples;
} UWB_Config;

UWB_Config UWB_Default;
uint8_t SET_NUM;
uint8_t DEV_NUM;
uint8_t AD_SHIFT;
/*
 * DMA channel for ADC scan mode
 * */
#define ADC_SCAN_DMA_CH    	2

/*
 * SPI Rx & Tx DMA channel
 * */
#define SPI_RX_DMA_CH	0
#define SPI_TX_DMA_CH	1


#define PORT_UART       gpioPortE
#define PIN_RX          11
#define PIN_TX          10
#endif /* INCLUDE_MAIN_H_ */
