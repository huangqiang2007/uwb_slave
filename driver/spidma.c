#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "timer.h"
#include "udelay.h"
#include "hal-config.h"
#include "main.h"
#include "mainctrl.h"
#include "uartdrv.h"
#include "spidrv.h"
#include "Typedefs.h"
#include "libdw1000.h"
#include "em_dma.h"

#if ( (DMA_CHAN_COUNT > 0) && (DMA_CHAN_COUNT <= 4) )
#define DMACTRL_CH_CNT      4
#define DMACTRL_ALIGNMENT   128

#elif ( (DMA_CHAN_COUNT > 4) && (DMA_CHAN_COUNT <= 8) )
#define DMACTRL_CH_CNT      8
#define DMACTRL_ALIGNMENT   256

#elif ( (DMA_CHAN_COUNT > 8) && (DMA_CHAN_COUNT <= 12) )
#define DMACTRL_CH_CNT      16
#define DMACTRL_ALIGNMENT   256

#else
#error "Unsupported DMA channel count (dmactrl.c)."
#endif

/** DMA control block array, requires proper alignment. */
SL_ALIGN(DMACTRL_ALIGNMENT)
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMACTRL_CH_CNT * 2] SL_ATTRIBUTE_ALIGN(DMACTRL_ALIGNMENT);

uint32_t channelNumTX = SPI_TX_DMA_CH;
uint32_t channelNumRX = SPI_RX_DMA_CH;

/**************************************************************************//**
 * @brief callback function that will trigger after a completed DMA transfer
 *****************************************************************************/
void refreshRxTransfer(uint32_t channelNum,
                     bool isPrimaryDescriptor,
                     void *userPtr)
{
	g_spiTransDes.recvDone = true;
}

/**************************************************************************//**
 * @brief callback function that will trigger after a completed DMA transfer
 *****************************************************************************/
void refreshTxTransfer(uint32_t channelNum,
                     bool isPrimaryDescriptor,
                     void *userPtr)
{
	g_spiTransDes.sendDone = true;
}

/**************************************************************************//**
 * @brief
 *    Initialize the DMA module to transfer packets via USART whenever there
 *    is room in the TX Register
 *
 * @note
 *    The callback object needs to at least have static scope persistence so
 *    that the reference to the object is valid beyond its first use in
 *    initialization. This is because the handler needs access to the callback
 *    function. If reference isn't valid anymore, then all dma transfers after
 *    the first one will fail.
 *****************************************************************************/
//void initTransferDma(uint8_t *txbuf, int txlen)
void initTransferDma(void)
{
	// Callback configuration for TX
	static DMA_CB_TypeDef callbackTX;
	callbackTX.cbFunc = (DMA_FuncPtr_TypeDef) refreshTxTransfer;
	callbackTX.userPtr = NULL;

	// Channel configuration for TX transmission
	DMA_CfgChannel_TypeDef channelConfigTX;
	channelConfigTX.highPri   = true;                // Set high priority for the channel
	channelConfigTX.enableInt = false;                // Interrupt used to reset the transfer
	channelConfigTX.select    = DMAREQ_USART1_TXBL;   // Select DMA trigger
	channelConfigTX.cb        = &callbackTX;  	    // Callback to refresh the DMA transfer
	DMA_CfgChannel(channelNumTX, &channelConfigTX);

	// Channel descriptor configuration for TX transmission
	static DMA_CfgDescr_TypeDef descriptorConfigTX;
	descriptorConfigTX.dstInc  = dmaDataIncNone;  // Destination doesn't move
	descriptorConfigTX.srcInc  = dmaDataInc1;     // Source doesn't move
	descriptorConfigTX.size    = dmaDataSize1;    // Transfer 8 bits each time
	descriptorConfigTX.arbRate = dmaArbitrate1;   // Arbitrate after every DMA transfer
	descriptorConfigTX.hprot   = 0;               // Access level/protection not an issue
	DMA_CfgDescr(channelNumTX, true, &descriptorConfigTX);
}

/**************************************************************************//**
 * @brief
 *    Initialize the DMA module to receive packets via USART and transfer
 *    them to a buffer
 *****************************************************************************/
//void initReceiveDma(uint8_t *rxbuf, int rxlen)
void initReceiveDma(void)
{
	// Callback configuration for RX
	static DMA_CB_TypeDef callbackRX;
	callbackRX.cbFunc = (DMA_FuncPtr_TypeDef) refreshRxTransfer;
	callbackRX.userPtr = NULL;

	// Channel configuration for RX transmission
	DMA_CfgChannel_TypeDef channelConfigRX;
	channelConfigRX.highPri   = true;                    // Set high priority for the channel
	channelConfigRX.enableInt = false;                    // Interrupt used to reset the transfer
	channelConfigRX.select    = DMAREQ_USART1_RXDATAV;    // Select DMA trigger
	channelConfigRX.cb        = &callbackRX;  	        // Callback to refresh the DMA transfer

	DMA_CfgChannel(channelNumRX, &channelConfigRX);

	// Channel descriptor configuration for RX transmission
	static DMA_CfgDescr_TypeDef descriptorConfigRX;
	descriptorConfigRX.dstInc  = dmaDataInc1;     // Destination moves one spot in the buffer every transmit
	descriptorConfigRX.srcInc  = dmaDataIncNone;  // Source doesn't move
	descriptorConfigRX.size    = dmaDataSize1;    // Transfer 8 bits each time
	descriptorConfigRX.arbRate = dmaArbitrate1;   // Arbitrate after every DMA transfer
	descriptorConfigRX.hprot   = 0;               // Access level/protection not an issue
	DMA_CfgDescr(channelNumRX, true, &descriptorConfigRX);
}

/**************************************************************************//**
 * @brief Initialize USART1
 *****************************************************************************/
void initUSART1 (int SpiClk)
{
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART1, true);

	// Configure GPIO mode
	GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 1); //tx
	GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 1);    //rx
	GPIO_PinModeSet(gpioPortB, 7, gpioModePushPull, 0); //clk
	GPIO_PinModeSet(gpioPortB, 8, gpioModePushPull, 1); //cs

	// Start with default config, then modify as necessary
	USART_InitSync_TypeDef config = USART_INITSYNC_DEFAULT;
	//config.master       = true;            // master mode
	config.baudrate     = SpiClk;         // CLK freq is 1 MHz
	config.autoCsEnable = true;            // CS pin controlled by hardware, not firmware
	config.clockMode    = usartClockMode0; // clock idle low, sample on rising/first edge
	config.msbf         = true;            // send MSB first
	config.enable       = usartDisable;    // making sure USART isn't enabled until we set it up
	USART_InitSync(USART1, &config);

	// Set and enable USART pin locations
	USART1->ROUTE = USART_ROUTE_CLKPEN | USART_ROUTE_CSPEN | USART_ROUTE_TXPEN
		| USART_ROUTE_RXPEN | USART_ROUTE_LOCATION_LOC0;

	// Enable USART1
	USART1->CMD |= (usartEnable | USART_CMD_MASTEREN);
}

void spiTransferForRead(SPITransDes_t *spiTransDes, uint8_t *txbuf, int txlen,
		uint8_t *rxbuf, int rxlen)
{
	uint32_t status = 0, status_txc = 0x20;

	memset(spiTransDes, 0x00, sizeof(*spiTransDes));
	memcpy(spiTransDes->txBuf, txbuf, txlen);
	spiTransDes->rxBuf = rxbuf;

	DMA_ActivateBasic(channelNumRX,
						true,
						false,
						(void *) spiTransDes->rxBuf,         // Destination address to transfer to
						(void *) &USART1->RXDATA,  // Source address to transfer from
						txlen + rxlen - 1);       // Number of DMA transfers minus 1

	DMA_ActivateBasic(channelNumTX,
						true,
						false,
						(void *) &USART1->TXDATA,  // Destination address to transfer to
						(void *) spiTransDes->txBuf,         // Source address to transfer from
						txlen + rxlen - 1);       // Number of DMA transfers minus 1

	status = USART1->STATUS;
	while (!(status & status_txc)) {
		status = USART1->STATUS;
		if (spiTransDes->uwbIRQOccur)
			break;
	}
}

void spiTransferForWrite(SPITransDes_t *spiTransDes, uint8_t *txbuf, int txlen)
{
	uint32_t status = 0, status_txc = 0x20;

	memset(spiTransDes, 0x00, sizeof(*spiTransDes));
	memcpy(spiTransDes->txBuf, txbuf, txlen);

	DMA_ActivateBasic(channelNumTX,
					true,
					false,
					(void *) &USART1->TXDATA,  // Destination address to transfer to
					(void *) spiTransDes->txBuf,         // Source address to transfer from
					txlen - 1);       // Number of DMA transfers minus 1

	status = USART1->STATUS;
	while (!(status & status_txc)) {
		status = USART1->STATUS;
		if (spiTransDes->uwbIRQOccur)
			break;
	}
}

void SPIDMAInit()
{
	// Initialize USART1 as SPI slave
	initUSART1(dwSpiSpeedLow);

	// Initializing the DMA
	DMA_Init_TypeDef init;
	init.hprot = 0;                      // Access level/protection not an issue
	init.controlBlock = dmaControlBlock; // Make sure control block is properly aligned
	DMA_Init(&init);

	// Setup LDMA channels for transfer across SPI
	initReceiveDma();
	initTransferDma();
}
