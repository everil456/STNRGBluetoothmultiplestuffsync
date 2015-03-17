#include "hal_types.h"
#include "gatt_server.h"
#include "gap.h"
#include "string.h"
#include "bluenrg_aci.h"
#include "bluenrg_aci_const.h"
#include "app_state.h"
#include <stdio.h>
#include <stdlib.h>
#include "syncr.h"

extern uint32_t offset=0;

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
  do {\
  	uuid_struct.uuid128[0] = uuid_0; uuid_struct.uuid128[1] = uuid_1; uuid_struct.uuid128[2] = uuid_2; uuid_struct.uuid128[3] = uuid_3; \
	uuid_struct.uuid128[4] = uuid_4; uuid_struct.uuid128[5] = uuid_5; uuid_struct.uuid128[6] = uuid_6; uuid_struct.uuid128[7] = uuid_7; \
	uuid_struct.uuid128[8] = uuid_8; uuid_struct.uuid128[9] = uuid_9; uuid_struct.uuid128[10] = uuid_10; uuid_struct.uuid128[11] = uuid_11; \
	uuid_struct.uuid128[12] = uuid_12; uuid_struct.uuid128[13] = uuid_13; uuid_struct.uuid128[14] = uuid_14; uuid_struct.uuid128[15] = uuid_15; \
	}while(0)

uint16_t chatServHandle, TXCharHandle, RXCharHandle;
/*  User Function where serial received data should be processed */
void processInputData(uint8_t * rx_data, uint16_t data_size);

/*******************************************************************************
* Function Name  : Add_Chat_Service
* Description    : Add the 'Accelerometer' service.
* Input          : None
* Return         : Status.
*******************************************************************************/
tBleStatus Add_Chat_Service(void)
{
  	tBleStatus ret;
    
    /*
    UUIDs:
    D973F2E0-B19E-11E2-9E96-0800200C9A66
    D973F2E1-B19E-11E2-9E96-0800200C9A66
    D973F2E2-B19E-11E2-9E96-0800200C9A66
    */
    
    const uint8_t service_uuid[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9};
    const uint8_t charUuidTX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
    const uint8_t charUuidRX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};
    
    ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid, PRIMARY_SERVICE, 7, &chatServHandle); /* original is 9?? */
    if (ret != BLE_STATUS_SUCCESS) goto fail;    
        
    ret =  aci_gatt_add_char(chatServHandle, UUID_TYPE_128, charUuidTX, 20, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                      16, 1, &TXCharHandle);
    if (ret != BLE_STATUS_SUCCESS) goto fail;
    
    ret =  aci_gatt_add_char(chatServHandle, UUID_TYPE_128, charUuidRX, 20, CHAR_PROP_WRITE|CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                      16, 1, &RXCharHandle);
    if (ret != BLE_STATUS_SUCCESS) goto fail;
    
	PRINTF("Chat Service added.\r\nTX Char Handle %04X, RX Char Handle %04X\r\n", TXCharHandle, RXCharHandle);
	return BLE_STATUS_SUCCESS; 
	
fail:
  	PRINTF("Error while adding Chat service.\n");
	return BLE_STATUS_ERROR ;
}


void Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data)
{
  tClockTime ct = Clock_Timeus();
    if(handle == RXCharHandle + 1){
      uint32_t got_time = (uint32_t) ct;
      printf("got_time: %d\n\r",got_time);
      printf("reveived: ");
        for(int i = 0; i < data_length; i++)
          printf("%c", att_data[i]);
        printf("\r\n");
        if(att_data[0] == '0'){
          printf("at index 0 received a 0\n\r");
          if(att_data[1] == 'O'){
            printf("at index 1 received an O\n\r");
            char number[4];
            int p = 0;
            for(int q = 2; q < 6; q++){
              number[p] = att_data[q];
              p++;
            }
            uint32_t time = atoi(number);
            //need to store offset somewhere. Rex where do you want it?
            offset = time;
          }
          else if(att_data[1] == 'T'){
            printf("at index 1 received a T\n\r");           
            uint32_t bstime = got_time;
            printf("bstime: %d\n\r", bstime);
            char message_t[8];
            char number[5];
            message_t[0] = 'B';
            message_t[1] = 'T';
            sprintf(number, "%d", bstime);
            int p = 0;
            for(int q = 2; q < 6; q++){
              message_t[q] = number[p];
              p++;
            }
            message_t[6] = '\n';
            message_t[7] = '\0';
            Clock_Wait(500);
            printf(message_t);
            processInputData(message_t, 8);  
          
          }
          else if(att_data[1] == 'E'){
            printf("at index 1 received an E\n\r");
            char number[4];
            int p = 0;
            for(int q = 2; q < 6; q++){
              number[p] = att_data[q];
              p++;
            }
            uint32_t time = atoi(number);
            //trigger event at time
            printf("Event at time: %d\n\r", time);
          }
          
        }
          
    }
      else if(handle == TXCharHandle + 2){        
        if(att_data[0] == 0x01)
            APP_FLAG_SET(NOTIFICATIONS_ENABLED);
    }
}


