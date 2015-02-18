#ifndef SLAVE_DEVICES_H__
#define SLAVE_DEVICES_H__

#include <stdio.h>
#include "link_layer.h"

/* Device struct definition*/
typedef struct
{ 
  tBDAddr bdaddr;
  uint16_t connection_handle;
  uint16_t tx_handle;
  uint16_t rx_handle;
}sDevice;

/* Slave Devices */
extern sDevice slave1,slave2;

/* Slave device addresses */
extern const tBDAddr slave1_addr;
extern const tBDAddr slave2_addr;

extern const int num_slaves;
extern sDevice slaves[2];

/* Methods */
void initDevices(void);

#endif