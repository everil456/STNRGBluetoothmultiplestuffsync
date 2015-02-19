#include "syncr.h"
#include <stdio.h>
#include <stm32l1xx_systick.h>
#include <stm32l1xx_type.h>

//variables
uint n = 20; //number to divide offset error by before adding it to offfsetp
uint32_t tolerance = 800; // 800 at 8 MHz = 100 us, tolerance to call a value good
uint32_t tolerance2 = 24000000; // 3 seconds, we need to reset offsetp

/*vocabulary
bs = base station
hh = hand held device
ss = sync server, this will be the master bluetooth device
p for permanent, or pervaisive
i for instantaneous, momentary, discrete, etc
bstimei = base station time, instantaneous, discrete, value to be sent to ss
hhtimei = "    "       "       "       "       "       "       "       "
bstimep = clock value on base station, accessed with getTime() function
hhtimep = clocktime on the handheld device
offseti = momentary offset between bstimei and hhtimei. 
offsetp = the assumed real offset between the bstimep and the hhtimep

*/
//for bs, comment out in hh ss code
//***on event, do this
{
  bstimei = getTime();
  //***send bstimei to ss
  //message to ss that has bstimei count and variable name
}

//set bstimep and hhtimep on bs
{
  bstimep = getTime();
  hhtimep = bstimep + offsetp;
}

//for hh, comment out in bs ss code
//***on event, do this
{
  hhtimei = getTime();
  //***send hhtimei to ss
  //message to ss that has hhtimei count and variable name
}

//for ss, comment out in bs hh code
{
  /*on receiving bstimei or hhtimei compare bstimei + offset with hhtimei, if
  close to sync interval, do nothing, if close to clock overflow, try bstimei + offset - clock overflow
  and if that is close, then do the calculation and update offsetp*/
  
uint32_t offseti = bstimei - hhtimei; 

// if offseti is very close to offsetp
if (((offseti - offsetp) < tolerance)&((offseti - offsetp) > - tolerance)) 
{
uint32_t offsetp = offsetp + (offseti - offsetp)/n; 
}

//offsetp needs to be initialized, so offseti is not close to offsetp or offsetp + overflow val.
if (((offseti - offsetp) > tolerance2)||((offseti - offsetp) < - tolerance2)) 
{
uint32_t offsetp = offseti; 
}


/*offsetp for offset permanent, 
or offfset pervaisive, offseti for offset instantaneous, or offset momentary */
  //***send offsetp to bs
}

// functions
uint32_t getTime()
{
  u32 timei = SysTick_GetCounter(); // timei for instantaneous time
  uint32_t ret = (uint32_t)timei;    
  return(ret);
}

/*******************************************************************************
* Function Name  : SysTick_GetCounter
* Description    : Gets SysTick counter value.
* Input          : None
* Output         : None
* Return         : SysTick current value
*******************************************************************************/
/*
u32 SysTick_GetCounter(void)
{
  return(SysTick->VAL);
}
*/
