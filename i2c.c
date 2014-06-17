#include "i2c.h"
#include "STM32F4xx.h"


extern void I2C_Initialize(char myadd)
{
   /* i2c peripheral clocks */
   RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
   /* I2C1(PB6/7) AF4 - seems that need to configure all parameters.. */ 
   GPIOB->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
   GPIOB->OTYPER |= (GPIO_OTYPER_ODR_6 | GPIO_OTYPER_ODR_7);
   GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR6 |GPIO_OSPEEDER_OSPEEDR7);
   GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0);
   GPIOB->AFR[0] |= ((4<<28) | (4<<24));
   I2C1->CR2 = 0x28; /* 40 MHz */
   I2C1->OAR1 = 0x4000 | (myadd<<1);  /* Slave address */
   I2C1->CR1 = 0x0401; /* Ack, Enable clock stretch, enable peripheral */
}
