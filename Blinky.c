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
#include "i2c.h"


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
   int dummy = 0;
//   uint32_t curTicks;
   int go = 0;
   int writeCount = 0;
   int readCount = 0;
   
   int dataPointer = 0;
   
   int i2c1sr1;
   
   unsigned char controlRegister = 0x00;
   unsigned char statusRegister = 0x00;
   
   SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
   if (SysTick_Config(SystemCoreClock / 1000))   /* SysTick 1 msec interrupts  */
   {
      while (1);                                  /* Capture error              */
   }

   LED_Initialize();
   Keyboard_Initialize();
   I2C_Initialize(0x24);
   
   LED_Off(REDLED);
   LED_Off(GREENLED);
   
   while(1)                                     /* Loop forever               */
   {
      i2c1sr1 = I2C1->SR1;
      if ((i2c1sr1 & 0x2) == 0x2) /* ADDR */
      {
         dummy |= I2C1->SR1;
         dummy |= I2C1->SR2; /* this clears ADDR */
         go = 1; /* This was for us */
         writeCount = 0;
         readCount = 0;
      }
      else if (go && ((i2c1sr1 & 0x80) == 0x80)) /* TxE */
      {
         switch (dataPointer + readCount)
         {
            case 0x00 : I2C1->DR = (I2C1->DR & 0xFF00) | controlRegister;
                        break;
            case 0x01 : I2C1->DR = (I2C1->DR & 0xFF00) | statusRegister;
                        break;
            default : I2C1->DR = (I2C1->DR & 0xFF00) | 0xFF;
                      break;
         }
         readCount++;  
      }
      else if (go && ((i2c1sr1 & 0x40) == 0x40)) /* RxNE */
      {
         switch (writeCount)
         {
            case 0x00 : dataPointer = I2C1->DR & 0xFF; /* First byte written is always dataPointer */
                        break;
            case 0x01 : if (dataPointer == 0x00)
                           controlRegister = I2C1->DR & 0xFF;
                        break;
            default : dummy |= I2C1->DR; 
                      break;
         }
         writeCount++;
      }
      else if ((i2c1sr1 & 0x0010)) /* STOPF */
      {
         dummy |= I2C1->SR1;
         I2C1->CR1 = I2C1->CR1 & 0xFFFF;
         go = 0;
      }
      else if ((i2c1sr1 & 0xDF00)) /* Anything to clear, then clear them */
         I2C1->SR1 = 0;
      
      /* For testing */
      statusRegister = Keyboard_GetKeys();
      if (controlRegister & 0x01) LED_On(REDLED); else LED_Off(REDLED);
      if (controlRegister & 0x02) LED_On(GREENLED); else LED_Off(GREENLED);
     
  }
}
