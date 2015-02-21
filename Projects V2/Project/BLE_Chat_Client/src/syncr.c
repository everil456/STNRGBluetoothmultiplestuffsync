#include "syncr.h"
#include <stdio.h>
#include <stm32l1xx_systick.h>
#include <stm32l1xx_type.h>

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
//*** we need to initialize offsetp to 0 somewhere that happens only once on startup or reset.
//for bs, comment out in hh ss code
//***on event, do this
{
  bstimei = get_time();
  //***send bstimei to ss
  //message to ss that has bstimei count and variable name
}

//set bstimep and hhtimep on bs
{
  bstimep = get_time();
  hhtimep = bstimep + offsetp;
}

//for hh, comment out in bs ss code
//***on event, do this
{
  hhtimei = get_time();
  //***send hhtimei to ss
  //message to ss that has hhtimei count and variable name
}

//for ss, comment out in bs hh code
{
  /*on receiving bstimei or hhtimei compare bstimei + offset with hhtimei, if
  close to sync interval, do nothing, if close to clock overflow, try bstimei + offset - clock overflow
  and if that is close, then do the calculation and update offsetp*/
  
uint32_t offseti = bstimei - hhtimei; 
uint32_t offsetp = update_offsetp(offsetp,offseti);//this function prints, so if you dont like it, comment out the printing inside the function

/*offsetp for offset permanent, 
or offfset pervaisive, offseti for offset instantaneous, or offset momentary */
  //***send offsetp to bs
}

// functions
uint32_t get_time()
{
  u32 timei = SysTick_GetCounter(); // timei for instantaneous time
  uint32_t ret = (uint32_t)timei;    
  return(ret);
}

uint32_t update_offsetp(offsetp,offseti)
{
  //variables
uint n = 10; //number to divide offset error by before adding it to offfsetp
uint32_t tolerance = 200; // 800 at 8 MHz = 100 us, tolerance to call a value good
uint n2 = 4; //number to divide offset error by before adding it to offfsetp
uint32_t tolerance2 = 24000000; // 3 seconds, we need to reset offsetp
// if offseti is very close to offsetp
if (((offseti - offsetp) < tolerance)&((offseti - offsetp) > - tolerance)) 
{
  uint32_t offsetp = offsetp + (offseti - offsetp)/n; 
  PRINTF("offsetp: %10d\n\r",offsetp);
  return offsetp;
}
// if offseti is semi close to offsetp
elseif (((offseti - offsetp) < tolerance2)&((offseti - offsetp) > - tolerance2)) 
{
  uint32_t offsetp = offsetp + (offseti - offsetp)/n2; 
  return offsetp;
}
//offsetp needs to be initialized
elseif (offsetp == 0) 
{
  uint32_t offsetp = offseti; 
  return offsetp;
}
// Error
else
{
  PRINTF("Error: offseti not within %10d clock ticks of offsetp.\n\r  offseti: %10d\n\r",tolerance2,offseti);
  return offsetp;
}
}
  
