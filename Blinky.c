/*----------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2004-2013 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include "STM32F4xx.h"
#include "led.h"
#include "keyboard.h"

volatile uint32_t msTicks;                      /* counts 1ms timeTicks       */

/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) 
{
   msTicks++;
}

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) 
{
   uint32_t curTicks;

   curTicks = msTicks;
   while ((msTicks - curTicks) < dlyTicks);
}


/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
int main (void) 
{
   SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
   if (SysTick_Config(SystemCoreClock / 1000))   /* SysTick 1 msec interrupts  */
   {
      while (1);                                  /* Capture error              */
   }

   LED_Initialize();
   Keyboard_Initialize();

   while(1)                                     /* Loop forever               */
   {
      if (Keyboard_GetKey(RIGHTBUTTON)) 
         LED_On(REDLED);  /* Right button -> red led  */
      
      if (Keyboard_GetKey(LEFTBUTTON))
         LED_On(GREENLED);  /* Left button -> green led */
      
      Delay( 50); 

      LED_Off(0);
      LED_Off(1);
      Delay(200); 
  }
}
