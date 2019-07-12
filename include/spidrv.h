#ifndef _SPIDRV_H_
#define _SPIDRV_H_

//#include "main.h"

extern void SPIConfig(void);
extern void SPISendByte(uint8_t ucData);
extern int SPISendNbytes(uint8_t *str, int n);

#endif
