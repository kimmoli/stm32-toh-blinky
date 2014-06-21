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

#include "registers.h"

volatile int registersChanged;
volatile uint8_t registers[NUMBER_OF_REGISTERS];

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
   uint32_t currentTicks = 0;
   int a = 0;
   
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
      
      if (msTicks > (currentTicks+1000))
      {
         a++;
         currentTicks = msTicks;
         if (a&1) LED_On(GREENLED); else LED_Off(GREENLED);
      }
      
      if (registersChanged)
      {
         registersChanged = 0;
         if (registers[0] & 0x01) LED_On(REDLED); else LED_Off(REDLED);
         if (registers[1] & 0x02) LED_On(GREENLED); else LED_Off(GREENLED);
      }
  }
}


/* 
   ***NOTES***

   PC13 = INTOUT <<>> Interrupt out to Jolla
   PA8 =  FLASH_CSL <<>> GPIO // SPI CSL for Flash
   PB5 =  EINK_CSL <<>> GPIO Output // SPI CSL for EINK

   EINK pins                                             einkCtrlRegister bit

   PA15 = EINK_RESETL <<>> GPIO Output                   0
   PB9 =  EINK_ON <<>> GPIO Output !! Must wirewrap      1
   PC7 =  EINK_BORDER <<>> GPIO Output                   2
   PC8 =  EINK_DISCHARGE <<>> GPIO Output                3

   PC6 =  EINK_PWM <<>> Connect to TIM3_CH1 = AF2        einkPWMRegister

   PB4 =  EINK_BUSY <<>> GPIO Input

   PC10 = SPI_SCLK <<>> SPI3 = AF6
   PC11 = SPI_MISO <<>> SPI3 = AF6
   PC12 = SPI_MOSI <<>> SPI3 = AF6
   

epd_fuse.c:     GPIO_mode(panel_on_pin, GPIO_OUTPUT);
epd_fuse.c:     GPIO_mode(border_pin, GPIO_OUTPUT);
epd_fuse.c:     GPIO_mode(discharge_pin, GPIO_OUTPUT);
epd_fuse.c:     GPIO_mode(pwm_pin, GPIO_PWM);
epd_fuse.c:     GPIO_mode(reset_pin, GPIO_OUTPUT);
epd_fuse.c:     GPIO_mode(busy_pin, GPIO_INPUT);


*/
