#include "Keyboard.h"
#include "GPIO_STM32F4xx.h"
#include "STM32F4xx.h"
#include "registers.h"

const GPIO_PIN_ID Pin_Key[] = 
{
  { GPIOC, 03 },  /* Button right       */
  { GPIOB, 14 }	/* Button left (derp) */
};

#define NUM_KEYS (sizeof(Pin_Key)/sizeof(GPIO_PIN_ID))


void Keyboard_Initialize (void) 
{
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
   RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
   
   SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PC; /* button right */
   SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI14_PB; /* button left */
   
   NVIC_EnableIRQ(EXTI3_IRQn);
   NVIC_EnableIRQ(EXTI15_10_IRQn);
   
   /* Enable interrupts from both rising and falling edge of buttons */
   EXTI->IMR |= EXTI_IMR_MR3 | EXTI_IMR_MR14;
   EXTI->RTSR |= EXTI_RTSR_TR3 | EXTI_RTSR_TR14;
   EXTI->FTSR |= EXTI_FTSR_TR3 | EXTI_FTSR_TR14;
   
}

/* 
 * Interrupt handlers 
 */
void EXTI3_IRQHandler(void)
{
   if (EXTI->PR & EXTI_PR_PR3)
      EXTI->PR |= EXTI_PR_PR3;
   
   registers[1] = (registers[1] & 0xFC) | Keyboard_GetKeys();
}

void EXTI15_10_IRQHandler(void)
{
   if (EXTI->PR & EXTI_PR_PR14)
      EXTI->PR |= EXTI_PR_PR14;
   
   registers[1] = (registers[1] & 0xFC) | Keyboard_GetKeys();
}

/*
 * Get all keys
 */
uint32_t Keyboard_GetKeys (void) 
{
   /* Read board keyboard inputs */
   uint32_t val = 0;
   uint32_t n;

   for (n = 0; n < NUM_KEYS; n++) 
   {
      if (GPIO_PinRead(Pin_Key[n].port, Pin_Key[n].num) != 0) 
      {
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