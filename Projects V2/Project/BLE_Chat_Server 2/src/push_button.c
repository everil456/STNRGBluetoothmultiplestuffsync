#include "push_button.h"
#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_exti.h"
#include "led.h"

boolean buttonOK = false;
/*
  * @brief  Configures and initializes the user button
  * @param  None
  * @retval None
  */
void initButton()
{
  buttonOK = true;
  GPIO_InitTypeDef GPIO_Structure1;
  GPIO_Structure1.GPIO_Pin = BUTTON;
  GPIO_Structure1.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Structure1.GPIO_Speed = GPIO_Speed_40MHz;                //Set GPIO speed to 40MHz
  GPIO_Init(GPIOC, &GPIO_Structure1);
}


/*
  * @brief  Enables the user button interrupt
  * @param  None
  * @retval None
  */
void enableButtonInterrupt()
{
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource6);
  EXTI_InitTypeDef EXTI_InitStruct1;
  NVIC_InitTypeDef NVIC_InitStruct1;

  EXTI_InitStruct1.EXTI_Line = EXTI_Line6;
  EXTI_InitStruct1.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct1.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStruct1.EXTI_LineCmd = ENABLE;
  EXTI_StructInit(&EXTI_InitStruct1);

  NVIC_InitStruct1.NVIC_IRQChannel = EXTI9_5_IRQn ;
  NVIC_InitStruct1.NVIC_IRQChannelPreemptionPriority = 0x0f;
  NVIC_InitStruct1.NVIC_IRQChannelSubPriority = 0x0f;
  NVIC_InitStruct1.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct1);
}


/**
  * @brief  Button interrupt handler
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
  GPIO_ToggleBits(GPIOC, LED3);
    if ( EXTI_GetITStatus(EXTI_Line6) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line6);
        
        //Code goes here
        
    }
}