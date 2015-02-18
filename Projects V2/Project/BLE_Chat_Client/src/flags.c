#include "flags.h"
#include <stdio.h>

int flagIsSet(uint8_t flag)
{
  int ret = flag && 1;
  return ret;
}


void changeFlag(int state,uint8_t *flag)
{
  *flag = state;
}

void initFlags(uint8_t size)
{
  extern Flags arrayFlags[size];
}