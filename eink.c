#include "eink.h"
#include "STM32F4xx.h"
#include "GPIO_STM32F4xx.h"

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
                    GPIO_NO_PULL_UP_DOWN);  
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
   
   TIM3->CR1 |= TIM_CR1_CKD_1; /* clockdiv4 */
}


int Eink_GetInput(uint32_t num)
{
	return (GPIO_PinRead(Eink_Inputs[num].port, Eink_Inputs[num].num) != 0);
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

