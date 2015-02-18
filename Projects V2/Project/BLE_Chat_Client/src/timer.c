#include "timer.h"
#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "led.h"
#include "push_button.h"

/**
  * @brief  Configures and initializes a timer
  * @param  maxCount - maximum timer count in miliseconds
  * @param  timx - the counter number (2,4,5,6, or 7)
  * @retval None
  */
void initTimer(int timx,int maxCount)
{
  TIM_TypeDef* timer;
  switch(timx)
  {
    case(2):
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
      timer = TIM2;
      break;
    case(4):
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
      timer = TIM4;
      break;
    case(5):
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
      timer = TIM5;
      break;
    case(6):
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
      timer = TIM6;
      break;
    case(7):
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
      timer = TIM7;
      break;
  }

    TIM_TimeBaseInitTypeDef timerInitStructure; 
    timerInitStructure.TIM_Prescaler = 40000;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = maxCount;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(timer, &timerInitStructure);
    TIM_ITConfig(timer, TIM_IT_Update, ENABLE);
}


/**
  * @brief  Starts a given a timer
  * @param  timx - the counter number (2,4,5,6, or 7)
  * @retval None
  */
void startTimer(int timx)
{
  switch(timx)
  {
    case(2):
      TIM2->CNT = 0;
      TIM_Cmd(TIM2, ENABLE);
      break;
    case(4):
      TIM4->CNT = 0;
      TIM_Cmd(TIM4, ENABLE);
      break;
    case(6):
      TIM6->CNT = 0;
      TIM_Cmd(TIM6, ENABLE);
      break;
    case(7):
      TIM7->CNT = 0;
      TIM_Cmd(TIM7, ENABLE);
      break;
  }
}


/**
  * @brief  Stops a given a timer
  * @param  timx - the counter number (2,4,5,6, or 7)
  * @retval None
  */
void stopTimer(int timx)
{
  switch(timx)
  {
    case(2):
      TIM_Cmd(TIM2, DISABLE);
      break;
    case(4):
      TIM_Cmd(TIM4, DISABLE);
      break;
    case(6):
      TIM_Cmd(TIM6, DISABLE);
      break;
    case(7):
      TIM_Cmd(TIM7, DISABLE);
      break;
  }
}

/**
  * @brief  Enables a timer interrupt
  * @param  None
  * @retval None
  */
void enableTimerInterrupt(int timx)
{
    NVIC_InitTypeDef nvicStructure;
    switch(timx)
  {
    case(2):
      nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
      break;
    case(4):
      nvicStructure.NVIC_IRQChannel = TIM4_IRQn;
      break;
    case(5):
      nvicStructure.NVIC_IRQChannel = TIM5_IRQn;
      break;
    case(6):
      nvicStructure.NVIC_IRQChannel = TIM6_IRQn;
      break;
    case(7):
      nvicStructure.NVIC_IRQChannel = TIM7_IRQn;
      break;
  }
    
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 2;
    nvicStructure.NVIC_IRQChannelSubPriority = 1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}


/**
  * @brief  Timer interrupt handler
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
    
        stopTimer(2);
        startTimer(4);
        startTimer(6);
        GPIO_ToggleBits(GPIOC,LED4);
    }
}


/**
  * @brief  Timer interrupt handler
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler()
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        
        GPIO_ToggleBits(GPIOC, LED4);
    }
}

/**
  * @brief  Timer interrupt handler
  * @param  None
  * @retval None
  */
void TIM5_IRQHandler()
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        
        GPIO_ToggleBits(GPIOD, LED2);
    }
}

/**
  * @brief  Timer interrupt handler
  * @param  None
  * @retval None
  */
void TIM6_IRQHandler()
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        
        //Put code here
        stopTimer(4);
        stopTimer(6);
    }
}

/**
  * @brief  Timer interrupt handler
  * @param  None
  * @retval None
  */
void TIM7_IRQHandler()
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
        
        stopTimer(7);
        buttonOK = true;
    }
}