#ifndef __REGISTERS_H
#define __REGISTERS_H

#include <stdint.h>

#define NUMBER_OF_REGISTERS 3

extern volatile int registersChanged;
extern volatile uint8_t registers[NUMBER_OF_REGISTERS];

#endif /* __REGISTERS_H */
