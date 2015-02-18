#ifndef LED_H__
#define LED_H__

#include <stdint.h>

/* Parameters */
#define LED0   GPIO_Pin_0       //Part of GPIOC
#define LED1   GPIO_Pin_1       //Part of GPIOC
#define LED3   GPIO_Pin_4       //Part of GPIOC
#define LED4   GPIO_Pin_5       //Part of GPIOC
#define LED2   GPIO_Pin_2       //Part of GPIOD

typedef enum
{ 
  OFF = 0x0,
  ON  = 0x1
}State_TypeDef;

void initLED(int ledx);
void turnLED(int led, State_TypeDef state);

#endif