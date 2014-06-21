#include "i2c.h"
#include "STM32F4xx.h"

#include "registers.h"

volatile uint16_t writeCount = 0;
volatile uint16_t readCount = 0;
volatile uint8_t dataPointer = 0;


void I2C_Initialize(char myadd)
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
   
   NVIC_EnableIRQ(I2C1_EV_IRQn);
   
   I2C1->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN; /* Enable buffer and event interrupts */
   I2C1->CR1 = I2C_CR1_ACK | I2C_CR1_PE;  /* Ack, clock stretch, enable peripheral */
}

void I2C1_EV_IRQHandler(void)
{
   uint16_t i2c1sr1;
   uint16_t dummy;
   
   i2c1sr1 = I2C1->SR1;

   if ((i2c1sr1 & I2C_SR1_ADDR) == I2C_SR1_ADDR)
   {
      dummy |= I2C1->SR1; /* sequence according to databook */
      dummy |= I2C1->SR2; /* this clears ADDR */
      writeCount = 0;
      readCount = 0;
   }
   
   if (((i2c1sr1 & I2C_SR1_TXE) == I2C_SR1_TXE))
   {
      if ((dataPointer + readCount) < NUMBER_OF_REGISTERS)
      {
         I2C1->DR = (I2C1->DR & 0xFF00) | registers[dataPointer + readCount];
      }
      else
      {
         I2C1->DR = (I2C1->DR & 0xFF00) | 0xFF;
      }
      readCount++;  
   }
   
   if (((i2c1sr1 & I2C_SR1_RXNE) == I2C_SR1_RXNE))
   {
      if (writeCount == 0)
      {
         dataPointer = I2C1->DR & 0xFF; /* First byte written is always dataPointer */
      }
      else if ((dataPointer + writeCount-1) < NUMBER_OF_REGISTERS) 
      {
         registers[(dataPointer + writeCount-1)] = I2C1->DR & 0xFF;
      }
      else
      {
         dummy |= I2C1->DR; 
      }
      writeCount++;
   }
   
   if ((i2c1sr1 & I2C_SR1_STOPF) == I2C_SR1_STOPF)
   {
      dummy |= I2C1->SR1; /* sequence according to databook */
      I2C1->CR1 = I2C1->CR1;
     
      if (writeCount > 0)
      {
         registersChanged = 1;
      }
   }
   

   if ((i2c1sr1 & I2C_SR1_BTF) == I2C_SR1_BTF)
   {
      dummy |= I2C1->DR;
   }

   
   if ((i2c1sr1 & 0xDF00)) /* Clear errors otg */
   {
      I2C1->SR1 = 0;
   }

   
}
