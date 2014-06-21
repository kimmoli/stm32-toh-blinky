/*-----------------------------------------------------------------------------
 * Name:    Keyboard.c
 * Purpose: Low level Keyboard functions
 *-----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include "Keyboard.h"
#include "GPIO_STM32F4xx.h"

const GPIO_PIN_ID Pin_Key[] = {
  { GPIOC, 03 },  /* Button right       */
  { GPIOB, 14 }	/* Button left (derp) */
};

#define NUM_KEYS (sizeof(Pin_Key)/sizeof(GPIO_PIN_ID))


/*-----------------------------------------------------------------------------
 *      Keyboard_Initialize:  Initialize keyboard/buttons
 *
 * Parameters: (none)
 * Return:     (none)
 *----------------------------------------------------------------------------*/
void Keyboard_Initialize (void) {
	
  uint32_t n;

  /* Configure pins: Input Mode (25 MHz) */
  for (n = 0; n < NUM_KEYS; n++) 
	{
    GPIO_PortClock   (Pin_Key[n].port, true);
    GPIO_PinWrite    (Pin_Key[n].port, Pin_Key[n].num, 0);
    GPIO_PinConfigure(Pin_Key[n].port, Pin_Key[n].num,
                    GPIO_MODE_INPUT,
                    GPIO_OUTPUT_PUSH_PULL,
                    GPIO_OUTPUT_SPEED_25MHz,
                    GPIO_NO_PULL_UP_DOWN);  
	}	
}


/*-----------------------------------------------------------------------------
 *      Keyboard_GetKeys:  Get keyboard state
 *
 * Parameters: (none)
 * Return:      Keys bitmask
 *----------------------------------------------------------------------------*/
uint32_t Keyboard_GetKeys (void) {
  /* Read board keyboard inputs */
  uint32_t val = 0;
	uint32_t n;

  /* Configure pins: Input Mode (25 MHz) */
  for (n = 0; n < NUM_KEYS; n++) 
	{
  if (GPIO_PinRead(Pin_Key[n].port, Pin_Key[n].num) != 0) {
    /* User button */
    val |= 1<<n;
		}
	}
  return (val);
}

/*
 * Get status of one key
 */
int Keyboard_GetKey(uint32_t num)
{
	return (GPIO_PinRead(Pin_Key[num].port, Pin_Key[num].num) != 0);
}


/*-----------------------------------------------------------------------------
 *      Keyboard_NumKeys:  Get number of available keys
 *
 * Parameters: (none)
 * Return:      number of keys
 *----------------------------------------------------------------------------*/
uint32_t Keyboard_NumKeys (void) {
  return (NUM_KEYS);
}
