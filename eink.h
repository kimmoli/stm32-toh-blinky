#ifndef __EINK_H
#define __EINK_H

#include <stdint.h>

extern void Eink_Initialize(void);
extern int Eink_GetInput(uint32_t num);
extern void Eink_SetOutputs(uint32_t val);

#endif /* __EINK_H */
