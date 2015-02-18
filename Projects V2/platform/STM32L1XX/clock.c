#include "clock.h"
#include "hal.h"

/* Added define for clock settings */
#include "SDK_EVAL_Clock.h"

#include "stm32l1xx_systick.h"

// The value that will be load in the SysTick value register.
#define RELOAD_VALUE        (SYSCLK_FREQ/1000)-1   // One clock each 1 ms
#define RELOAD_VALUE_SLEEP  (SYSCLK_FREQ_SLEEP/1000)-1   // One clock each 1 ms

static volatile tClockTime count = 0;

const uint32_t CLOCK_SECOND = 1000;

/*---------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
  count++;
}

/*---------------------------------------------------------------------------*/

void Clock_Init(void)
{
  ATOMIC_SECTION_BEGIN();

  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  SysTick_SetReload(RELOAD_VALUE);
  SysTick_CounterCmd(SysTick_Counter_Enable);
  SysTick_ITConfig(ENABLE);

  ATOMIC_SECTION_END();
}

void Clock_Suspend(void)
{
  SysTick->CTRL = 0;
  SysTick_CounterCmd(SysTick_Counter_Clear);
}

void Clock_Resume(Clock_TypeDef ClockType)
{
  uint32_t reload_value;
  
  if(ClockType == CLOCK_SLEEP){
    reload_value = RELOAD_VALUE_SLEEP;
  }
  else {
    reload_value = RELOAD_VALUE;
  }  
  
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  SysTick_SetReload(reload_value);
  SysTick_CounterCmd(SysTick_Counter_Enable);
  SysTick_ITConfig(ENABLE);
}

/*---------------------------------------------------------------------------*/

tClockTime Clock_Time(void)
{
  return count;
}

/*---------------------------------------------------------------------------*/
/**
 * Wait for a multiple of 1 ms.
 *
 */
void Clock_Wait(uint32_t i)
{
  tClockTime start;

  start = Clock_Time();
  while(Clock_Time() - start < (tClockTime)i);
}
/*---------------------------------------------------------------------------*/

