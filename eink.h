#ifndef __EINK_H
#define __EINK_H

#include <stdint.h>

extern void Eink_Initialize(void);
extern int Eink_GetInput(uint32_t num);
extern void Eink_SetOutputs(uint32_t val);
extern void Eink_SPITransmit(uint8_t first, uint8_t data);
extern void Eink_SPIStop(void);
extern void Eink_SPIChipSelect(uint8_t which, uint8_t state);

#define FLASH_CS 8
#define EINK_CS 5


#endif /* __EINK_H */
