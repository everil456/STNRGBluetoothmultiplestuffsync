#include "slave_devices.h"
#include "flags.h"
#include <string.h>
#include <stdio.h>


/* Slave devices */
sDevice slave1,slave2;
sDevice slaves[2];

/* Slave device addresses */
const int num_slaves = 2;
const tBDAddr slave1_addr = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02};
const tBDAddr slave2_addr = {0xcc, 0x00, 0x00, 0xE1, 0x80, 0x02};

/**
  * @brief  Initializes the slaves array with the corresponding addresses
  * @param  None 
  * @retval None
  */
void initDevices(void)
{
  memcpy(slave1.bdaddr,slave1_addr,sizeof(tBDAddr));
  memcpy(slave2.bdaddr,slave2_addr,sizeof(tBDAddr));
  slaves[0] = slave1;
  slaves[1] = slave2;
  
  uint8_t size = sizeof(slaves)/sizeof(sDevice);
  initFlags(size);
  
}