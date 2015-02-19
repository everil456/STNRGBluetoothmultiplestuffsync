#include "slave_devices.h"
#include "flags.h"
#include <string.h>
#include <stdio.h>


/* Slave devices */
sDevice slaves[numSlaves];

/* Slave device addresses */
uint8_t slaveAddresses[numSlaves][sizeof(tBDAddr)] = {
  {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02},
  {0xcc, 0x00, 0x00, 0xE1, 0x80, 0x02}
};
  ;

/**
  * @brief  Initializes the slaves array with the corresponding addresses
  * @param  None 
  * @retval None
  */
void initDevices(void)
{
  for(int i = 0; i < numSlaves; i++)
  {
    memcpy(slaves[i].bdaddr,slaveAddresses[i],sizeof(tBDAddr));
  }
  
  //uint8_t size = sizeof(slaves)/sizeof(sDevice);
  initFlags();
  
}