/*
 * Driver for decaWave DW1000 802.15.4 UWB radio chip.
 *
 * Copyright (c) 2016 Bitcraze AB
 * Converted to C from  the Decawave DW1000 library for arduino.
 * which is Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string.h>
#include "libdw1000Spi.h"
#include "spidrv.h"

//void dwSpiRead_01(dwDevice_t *dev, uint8_t regid, uint32_t address,
//                                void* data, int length) {
//  uint8_t header[10] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//  uint32_t status = 0;
//  uint8_t *pdata = data;
//
//  status = usart_spi->STATUS;
//  //USART_Enable(usart_spi, usartEnable);
//  usart_spi->CMD |= (usartEnable);
//  usart_spi->CMD |= 0xc00;
//  usart_spi->CTRL |= USART_CTRL_AUTOTX;
//  SPISendNbytes(header, 1);
//  SPIRecvNBytes(data, length);
////  for (int i = 0; i < 10; i++)
////  	  pdata[i] = USARTSpiTransfer(usart_spi, header[i]);
//  //USART_Enable(usart_spi, usartDisable);
//  usart_spi->CTRL &= ~USART_CTRL_AUTOTX;
//  usart_spi->CMD |= (USART_CMD_RXDIS | USART_CMD_TXDIS);
//  status = usart_spi->STATUS;
//}

void dwSpiRead(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                void* data, int length) {
  uint8_t header[Header_Size];
  int headerLength;


  header[0] = regid & 0x3f; //set read mode, set sub-index = 0 by default, set register file ID to header buffer 0
  headerLength = 1; //set header length = 1 by default,

  if (address != 0) {  //if sub-address != 0
    header[0] |= 0x40; //set sub-index = 1

    header[1] = address & 0x7f; //set extend-address = 0, set lower order 7 bits sub-address to header buffer 1
    address >>= 7;
    headerLength = 2; //set header length = 2

    if (address != 0) { //if sub-address > 127
      header[1] |= 0x80; //set extend-address = 1
      header[2] = address & 0xff; //set higher order 8 bits sub-address to header buffer 2
      headerLength = 3; //set header length = 3
    }
  }
  USART1->CMD |= 0x00000C00; //clear buffer
  spiTransferForRead(&g_spiTransDes, header, headerLength, dataRead, length);
  memcpy(data, &dataRead[headerLength],length);
}

uint16_t dwSpiRead16(dwDevice_t *dev, uint8_t regid, uint32_t address)
{
  uint16_t data;
  dwSpiRead(dev, regid, address, &data, sizeof(data));

  return data;
}

uint32_t dwSpiRead32(dwDevice_t *dev, uint8_t regid, uint32_t address)
{
  uint32_t data;
  dwSpiRead(dev, regid, address, &data, sizeof(data));

  return data;
}

void dwSpiWrite(dwDevice_t *dev, uint8_t regid, uint32_t address,
	const void* data, int length) {
  //uint8_t header[Header_Size];
  int headerLength;

  txbuffer[0] = regid | 0x80; //set write mode, set sub-index = 0 by default, set register file ID to header buffer 0
  headerLength = 1; //set header length = 1 by default,

  if (address != 0) { //if sub-address != 0
	txbuffer[0] |= 0x40; //set sub-index = 1

	txbuffer[1] = address & 0x7f; //set extend-address = 0, set lower order 7 bits sub-address to header buffer 1
    address >>= 7;
    headerLength = 2; //set header length = 2

    if (address != 0) { //if sub-address > 127
      txbuffer[1] |= 0x80; //set extend-address = 1
      txbuffer[2] = address & 0xff; //set higher order 8 bits sub-address to header buffer 2
      headerLength = 3; //set header length = 3
    }
  }

  memcpy(&txbuffer[headerLength], data, length);
  USART1->CMD |= 0x00000C00; //clear buffer
  spiTransferForWrite(&g_spiTransDes, txbuffer, length+headerLength);
}

void dwSpiWrite8(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                   uint8_t data) {
	dwSpiWrite(dev, regid, address, &data, sizeof(data));
}

void dwSpiWrite16(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                   uint16_t data) {
	dwSpiWrite(dev, regid, address, &data, sizeof(data));
}

void dwSpiWrite32(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                   uint32_t data) {
	dwSpiWrite(dev, regid, address, &data, sizeof(data));
}
