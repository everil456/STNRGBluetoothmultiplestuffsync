#ifndef FLAGS_H__
#define FLAGS_H__

#define SET     1
#define CLEAR   0

#include <stdio.h>
#include "link_layer.h"
/* Device struct definition*/
typedef struct
{ 
  uint8_t connected;
  uint8_t set_connectable;
  uint8_t notifications_enabled;
  uint8_t conn_param_upd_sent;
  uint8_t l2cap_param_upd_sent;
  uint8_t start_read_TX_char_handle;
  uint8_t end_read_TX_char_handle;
  uint8_t start_read_RX_char_handle;
  uint8_t end_read_RX_char_handle;
 }Flags;

/* Function Declarations */
int flagIsSet(uint8_t flag);
void changeFlag(int state,uint8_t *flag);

void initFlags(uint8_t size);


#endif