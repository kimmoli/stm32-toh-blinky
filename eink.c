#include "eink.h"
#include "STM32F4xx.h"
#include "GPIO_STM32F4xx.h"
#include "registers.h"

/*
   PC6 =  EINK_PWM <<>> Connect to TIM3_CH1 = AF2
*/
#define PWM_ARR 80

/*
   126 gives 126 kHz
   106 gives 150 kHz
   80  gives 198 kHz
*/


const GPIO_PIN_ID Eink_Outputs[] = 
{
  { GPIOA, 15 }, /* 0 - EINK_RESETL */
  { GPIOB, 9 },  /* 1 - EINK_ON */
  { GPIOC, 7 },  /* 2 - EINK_BORDER */
  { GPIOC, 8 }   /* 3 - EINK_DISCHARGE */
                 /* 4 - EINK_PWM */
};

#define NUM_OUTPUTS (sizeof(Eink_Outputs)/sizeof(GPIO_PIN_ID))

const GPIO_PIN_ID Eink_Inputs[] = 
{
  { GPIOB, 4 } /* EINK_RESETL */
};

#define NUM_INPUTS (sizeof(Eink_Inputs)/sizeof(GPIO_PIN_ID))


/*
 * Eink processor side initializations (gpio, pwm)
 */

void Eink_Initialize(void)
{
   uint32_t n;

   /* Configure pins: Push-pull Output Mode (50 MHz) with Pull-down resistors */
   for (n = 0; n < NUM_OUTPUTS; n++) 
   {
      GPIO_PortClock   (Eink_Outputs[n].port, true);
      GPIO_PinWrite    (Eink_Outputs[n].port, Eink_Outputs[n].num, 0);
      GPIO_PinConfigure(Eink_Outputs[n].port, Eink_Outputs[n].num,
                     GPIO_MODE_OUTPUT,
                     GPIO_OUTPUT_PUSH_PULL,
                     GPIO_OUTPUT_SPEED_50MHz,
                     GPIO_PULL_DOWN);
   }


   /* Configure pins: Input Mode (25 MHz) */
   for (n = 0; n < NUM_INPUTS; n++) 
   {
       GPIO_PortClock   (Eink_Inputs[n].port, true);
       GPIO_PinWrite    (Eink_Inputs[n].port, Eink_Inputs[n].num, 0);
       GPIO_PinConfigure(Eink_Inputs[n].port, Eink_Inputs[n].num,
                    GPIO_MODE_INPUT,
                    GPIO_OUTPUT_PUSH_PULL,
                    GPIO_OUTPUT_SPEED_25MHz,
                    GPIO_PULL_DOWN);  
	}   
   

   /* PWM Output */ 
   /* Accroding to repaper.org, PWM is 100~300kHz 50% */
   
   /* TIM3 and GPIOC peripheral clocks */
   RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
   /* PC6 =  EINK_PWM <<>> Connect to TIM3_CH1 = AF2 */
   GPIOC->MODER |= (GPIO_MODER_MODER6_1);
   GPIOC->OTYPER &= ~GPIO_OTYPER_ODR_6;
   GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6;
   GPIOC->AFR[0] |= (2<<24);

   TIM3->ARR = PWM_ARR;    /* frequency */ 
   TIM3->CCR1 = PWM_ARR/2; /* duty cycle ~50% */
   TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
   TIM3->CCER |= TIM_CCER_CC1E;
   
   TIM3->CR1 |= TIM_CR1_CKD_1; /* clockdiv4 */ /* PWM Is enabled over i2c register write */
   
   RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
   
   SYSCFG->EXTICR[2] |= SYSCFG_EXTICR2_EXTI4_PC;
   
   NVIC_EnableIRQ(EXTI4_IRQn);
   
   /* Enable interrupts from both rising and falling edge of buttons */
   EXTI->IMR |= EXTI_IMR_MR4;
   EXTI->RTSR |= EXTI_RTSR_TR4;
   EXTI->FTSR |= EXTI_FTSR_TR4;
   
   /* SPI */
   
/*   
   PC10 = SPI_SCLK <<>> SPI3 = AF6
   PC11 = SPI_MISO <<>> SPI3 = AF6
   PC12 = SPI_MOSI <<>> SPI3 = AF6
   
   PA8 =  FLASH_CSL <<>> GPIO // SPI CSL for Flash
   PB5 =  EINK_CSL <<>> GPIO Output // SPI CSL for EINK
*/

   /*   GPIOA, GPIOB, GPIOC clocks enable (fur sure these are already enabled, but for reusability sake...) */
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
   /* SPI3 Clock enable */
   RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
   
   /* PA8 Output*/
   GPIOA->MODER |= (GPIO_MODER_MODER8_0);
   GPIOA->OTYPER &= ~GPIO_OTYPER_ODR_8;
   GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;
   GPIOA->ODR |= (1<<8); /* Set initially high*/

   /* PB5 Output*/
   GPIOB->MODER |= (GPIO_MODER_MODER5_0);
   GPIOB->OTYPER &= ~GPIO_OTYPER_ODR_5;
   GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5;
   GPIOB->ODR |= (1<<5); /* Set initially high*/

   /* PC10,11,12 SPI3 AF6 */
   GPIOC->MODER |= (GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1) | GPIO_MODER_MODER12_1;
   GPIOC->OTYPER &= ~(GPIO_OTYPER_ODR_10 | GPIO_OTYPER_ODR_11 | GPIO_OTYPER_ODR_12);
   GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12;
   GPIOC->AFR[1] |= (6<<8) | (6<<12) | (6<<16);
   
   SPI3->CR1 = 0;  
   SPI3->CR2 = 0;  

   /*
     bitrate = pclk/16 (011)    pclk/4 (001)
     cpol=0 - low when idle
     cpha=0 - sample on rising
     mastermode=1
     DFF=0 - 8bit mode
     LSBFIRST=0 - MSB First

     SSOE=1 use NSS pin
   */

    SPI3->CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;  // 1.0 MHz SPI
    /*SPI3->CR1 |= SPI_CR1_BR_0 | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;  */           // ??  MHz SPI
    /*SPI3->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;      */                      // ?? MHz SPI
    
    /*SPI3->CR2 |= SPI_CR2_SSOE;*/
    
    SPI3->CR1 |= SPI_CR1_SPE; // enable SPI1
}

void EXTI4_IRQHandler(void)
{
   if (EXTI->PR & EXTI_PR_PR3)
   EXTI->PR |= EXTI_PR_PR3;
   
   registers[1] = (registers[1] & 0x7F) | (Eink_GetInput(0) << 7);
}

int Eink_GetInput(uint32_t num)
{
	return (GPIO_PinRead(Eink_Inputs[num].port, Eink_Inputs[num].num) != 0) ? 1 : 0;
}

void Eink_SetOutputs(uint32_t val)
{
   uint32_t n;

   for (n = 0; n < NUM_OUTPUTS; n++) 
   {
      if (val & (1<<n)) 
      {
         GPIO_PinWrite(Eink_Outputs[n].port, Eink_Outputs[n].num, 1);
      }
      else
      {
         GPIO_PinWrite(Eink_Outputs[n].port, Eink_Outputs[n].num, 0);
      }
   }
   
   /* last bit of outputs is the PWM output, enable/disable TIM3 accordingly */
   if (val & (1<<NUM_OUTPUTS))
   {
      TIM3->CR1 |= TIM_CR1_CEN;
   }
   else
   {
      TIM3->CR1 &= ~TIM_CR1_CEN;
   }
      
}

/* SPI Functions */

void Eink_SPIChipSelect(uint8_t which, uint8_t state)
{
   /*
   PA8 =  FLASH_CSL <<>> GPIO // SPI CSL for Flash
   PB5 =  EINK_CSL <<>> GPIO Output // SPI CSL for EINK
   */
   
   if (which == EINK_CS)
      GPIO_PinWrite(GPIOB, EINK_CS, state);
   else if (which == FLASH_CS)
      GPIO_PinWrite(GPIOA, FLASH_CS, state);
   
}

void Eink_SPITransmit(uint8_t first, uint8_t data)
{
   uint32_t dummy = 0;
   
   if (first)
      Eink_SPIChipSelect(EINK_CS, 0);

   while ((SPI3->SR & SPI_SR_TXE) != SPI_SR_TXE); 
   SPI3->DR = (0xff & data);
   while ((SPI3->SR & SPI_SR_RXNE) != SPI_SR_RXNE); 
   dummy += SPI3->DR;
}

void Eink_SPIStop(void)
{
   Eink_SPIChipSelect(EINK_CS, 1);
}
