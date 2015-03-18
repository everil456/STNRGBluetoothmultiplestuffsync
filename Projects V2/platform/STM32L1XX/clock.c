#include "clock.h"
#include "hal.h"
#include <stdio.h>

/* Added define for clock settings */
#include "SDK_EVAL_Clock.h"

#include "stm32l1xx_systick.h"

// The value that will be load in the SysTick value register.
#define RELOAD_VALUE        (SYSCLK_FREQ/500000)-1   // One clock each 2 us
#define RELOAD_VALUE_SLEEP  (SYSCLK_FREQ_SLEEP/1000)-1   // One clock each 1 ms

void processInputData(uint8_t * rx_data, uint16_t data_size);

volatile tClockTime count, countr, counter, eventr = 6500000; //defined in Main
volatile u8 eventEanble = 0;

const uint32_t CLOCK_SECOND = 1000;

/*---------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
  countr++;
  counter++;
  
  if(counter >= 500)
  {
    counter = 0;
    count++;
  }
  if(countr >= eventr)//&&(eventEanble)
  {
    //printf("%d\n\r%d\n\r%d\n\r", count, eventr, countr);
    //startTimer(2);
    uint8_t a[3]={'0','T','\n'};
    uint8_t b[3]={'1','T','\n'};
    uint16_t size = sizeof(a)/sizeof(a[0]);
    //eventr += 7654321;
  }
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
<<<<<<< .mine
=======

>>>>>>> .r36
  return count;
<<<<<<< .mine
=======
  ////printf("inside Clock_Time\r\n");
>>>>>>> .r36
}
/*---------------------------------------------------------------------------*/

tClockTime Clock_Timeus(void)
{
<<<<<<< .mine
  return countr;
=======

  return count;
  
>>>>>>> .r36
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
<<<<<<< .mine
=======

>>>>>>> .r36
  while(Clock_Time() - start < (tClockTime)(i));
<<<<<<< .mine
=======
  //while(Clock_Time() - start < (tClockTime)(i*1000));

  while(Clock_Time() - start < (tClockTime)(i*1000));

  //while(Clock_Time() - start < (tClockTime)(i*1000));
  while(Clock_Time() - start < (tClockTime)(i));

>>>>>>> .r36
}
/*---------------------------------------------------------------------------*/
/**
 * Wait for a multiple of 1 us.
 *
 */
void Clock_Waitus(uint32_t i)
{
  tClockTime startus;

  startus = Clock_Timeus();
  while(Clock_Timeus() - startus < (tClockTime)(i));
}
/*---------------------------------------------------------------------------*/

