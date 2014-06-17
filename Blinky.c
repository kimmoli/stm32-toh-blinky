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
   uint32_t curTicks;
   int go = 0;
   
   unsigned char testData = 0x00;
   int m = 0;
   //char testString[] = "ABCDEFGHIJKL";
   
   
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
      if (msTicks - curTicks > 100)
      {
         LED_Off(REDLED);
         LED_Off(GREENLED);
      }
      
      if ((I2C1->SR1 & 0x2) == 0x2) /* ADDR */
      {
         dummy |= I2C1->SR2; /* this clears ADDR */
         I2C1->DR = (I2C1->DR & 0xFF00)| 0xAA;
         LED_On(GREENLED );
         curTicks = msTicks;
         go = 1;
      }
      if (go && ((I2C1->SR1 & 0x80) == 0x80)) /* TxE */
      {
         LED_On(REDLED);
         
         I2C1->SR1 = 0;
         
         if (m==0)
            testData = 0xFF;
         else if (m==2)
            testData = I2C1->SR2 & 0xFF;
         else if (m==1)
            testData = (I2C1->SR2>>8) & 0xFF;
         else 
            testData = m;
         
         I2C1->DR = (I2C1->DR & 0xFF00) | testData;
         m=(m+1)%10;
      }
      
      if (go && ((I2C1->SR1 & 0x40) == 0x40)) /* RxNE */
      {
         testData = 0xFF & I2C1->DR;
      }

      if ((I2C1->SR1 & 0x410)) /* AF, STOPF */
      {
         LED_Off(REDLED);
         go = 0;
         I2C1->SR1 = 0;
      }
      

      
//      if (Keyboard_GetKey(RIGHTBUTTON)) 
//         LED_On(REDLED);  /* Right button -> red led  */
//      
//      if (Keyboard_GetKey(LEFTBUTTON))
//         LED_On(GREENLED);  /* Left button -> green led */
//      
//      Delay( 50); 

//      LED_Off(REDLED);
//      LED_Off(GREENLED);
//      Delay(200); 
  }
}
