#include "led.h"
#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include <stdio.h>

/**
  * @brief  Configures and initializes the user LED GPIO
  * @param  None
  * @retval None
  */
void initLED(int ledx)
{
  /*After reset, the peripheral clock (used for registers read/write access) is disabled
    and the application software has to enable this clock before using it.*/

    GPIO_InitTypeDef GPIO_Structure;                              //Instantiate a GPIO_InitTypeDef used for setting up GPIO
    switch(ledx)
    {
      case(0):
        GPIO_Structure.GPIO_Pin = LED0;
        break;
      case(1):
        GPIO_Structure.GPIO_Pin = LED1;
        break;
      case(2):
        GPIO_Structure.GPIO_Pin = LED2;
        break;
      case(3):
        GPIO_Structure.GPIO_Pin = LED3;
        break;
      case(4):
        GPIO_Structure.GPIO_Pin = LED4;
        break;
    }

    GPIO_Structure.GPIO_Mode = GPIO_Mode_OUT;                     //Set output mode of LED pins to output
    GPIO_Structure.GPIO_Speed = GPIO_Speed_40MHz;
    
    if(ledx == 2)
    {
      GPIO_Init(GPIOD, &GPIO_Structure);                            //Initialize GPIOG with the set parameters
      return;
    }
    else
    {
      GPIO_Init(GPIOC, &GPIO_Structure);                            //Initialize GPIOG with the set parameters
    }
    turnLED(ledx,OFF);
}

/**
  * @brief  Turns the specified LED to the specified state (either ON or OFF)
  * @param  led - the LED to be affected (LEDx where x is 0,1,2,3 or 4)
  * @param  state - the state of the affected LED (ON or OFF)
  * @retval None
  */
void turnLED(int led, State_TypeDef state)
{
  if(state == ON)
  {
    switch(led)
    {
      case(0):
        GPIOC->ODR &= ~(0x00000000 | LED0);
        break;
      case(1):
        GPIOC->ODR &= ~(0x00000000 | LED1);
        break;
      case(2):
        GPIOD->ODR &= ~(0x00000000 | LED2);
        break;
      case(3):
        GPIOC->ODR &= ~(0x00000000 | LED3);
        break;
      case(4):
        GPIOC->ODR &= ~(0x00000000 | LED4);
        break;
    }
  }
  else if(state == OFF)
  {
    switch(led)
    {
      case(0):
        GPIOC->ODR |= LED0;
        break;
      case(1):
        GPIOC->ODR |= LED1;
        break;
      case(2):
        GPIOD->ODR |= LED2;
        break;
      case(3):
        GPIOC->ODR |= LED3;
        break;
      case(4):
        GPIOC->ODR |= LED4;
        break;
    }
  }
}
