#ifndef TIMER_H__
#define TIMER_H__

#define TIM5_IRQn   46

void initTimer(int timx,int maxCount);
void enableTimerInterrupt(int timx);
void startTimer(int timx);
void stopTimer(int timx);
void TIM2_IRQHandler();
void TIM3_IRQHandler();
void TIM4_IRQHandler();
void TIM5_IRQHandler();
void TIM6_IRQHandler();
void TIM7_IRQHandler();

#endif