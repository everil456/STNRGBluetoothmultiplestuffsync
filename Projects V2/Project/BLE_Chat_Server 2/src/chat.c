#include <stdio.h>
#include <string.h>

#include "stm32l1xx.h"
#include "hal_types.h"
#include "gap.h"
#include "gatt_server.h"
#include "hci.h"
#include "osal.h"
#include "clock.h"
#include "gp_timer.h"
#include "bluenrg_aci.h"
#include "SDK_EVAL_Io.h"


#define MIN(a,b)            ((a) < (b) )? (a) : (b) 

#define CMD_BUFF_SIZE 512

static char cmd[CMD_BUFF_SIZE];

extern uint16_t chatServHandle, TXCharHandle, RXCharHandle;
extern volatile uint16_t connection_handle;


#ifdef CLIENT
extern uint16_t rx_handle;
#endif 
/*******************************************************************************
* Function Name  : processInputData.
* Description    : Process a command. It should be called when data are received.
* Input          : data_buffer: data address.
*	           Nb_bytes: number of received bytes.
* Return         : none.
*******************************************************************************/

void processInputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
    static uint16_t end = 0;
    uint8_t i;
    
    
    for (i = 0; i < Nb_bytes; i++)
    {
        if(end >= CMD_BUFF_SIZE-1){
            end = 0;
        }
        
        cmd[end] = data_buffer[i];
        SdkEval_IO_Send_Data(data_buffer[i]);
        end++;
        
        if((cmd[end-1] == '\r') || (cmd[end-1] == '\n')){
          cmd[end-1] = '\n';
          printf("\r\n");
            if(end != 1){
                int j = 0;
                cmd[end] = '\0';
                    
                while(j < end){
                    uint32_t len = MIN(20, end - j);
                    struct timer t;
                    Timer_Set(&t, CLOCK_SECOND*10);
                    
#if SERVER                      
                    while(aci_gatt_update_char_value(chatServHandle,TXCharHandle,0,len,(uint8_t *)cmd+j)==BLE_STATUS_INSUFFICIENT_RESOURCES){
#elif CLIENT
                    while(aci_gatt_write_without_response(connection_handle, rx_handle+1, len, (uint8_t *)cmd+j)==BLE_STATUS_NOT_ALLOWED){
#else
#error "Define SERVER or CLIENT"
#endif
                        // Radio is busy (buffer full).
                        if(Timer_Expired(&t))
                            break;
                    }                    
                    j += len;            
                }
            }
            end = 0;
        }
        else{
        }
    }
}