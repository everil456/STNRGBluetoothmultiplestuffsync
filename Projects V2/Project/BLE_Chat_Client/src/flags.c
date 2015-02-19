#include "flags.h"
#include "slave_devices.h"
#include <stdio.h>
#include <string.h>

Flags arrayFlags[numSlaves];

int flagIsSet(uint8_t flag)
{
  int ret = flag && 1;
  return ret;
}


void changeFlag(int state,uint8_t *flag)
{
  *flag = state;
}

void initFlags(void)
{
  memset(&arrayFlags, 0, sizeof(arrayFlags));
  for(int i = 0; i < numSlaves; i++)
    arrayFlags[i].set_connectable = 1;
}