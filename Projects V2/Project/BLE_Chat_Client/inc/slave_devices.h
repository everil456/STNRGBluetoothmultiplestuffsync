#ifndef SLAVE_DEVICES_H__
#define SLAVE_DEVICES_H__

#include <stdio.h>
#include "link_layer.h"

#define numSlaves       2       //Total number of slave devices

/* Device struct definition*/
typedef struct
{ 
  tBDAddr bdaddr;
  uint16_t connection_handle;
  uint16_t tx_handle;
  uint16_t rx_handle;
}sDevice;


/* Slave device array */
extern sDevice slaves[numSlaves];

/* Methods */
void initDevices(void);

#endif