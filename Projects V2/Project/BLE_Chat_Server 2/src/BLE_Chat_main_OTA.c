/******************** (C) COPYRIGHT 2014 STMicroelectronics ********************
* File Name          : BLE_Chat_main_OTA.c
* Author             : AMS - AAS Division
* Version            : V1.0.1
* Date               : 10-February-2014
* Description        : BlueNRG main file for Chat demo with OTA support
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/**
 * @file  BLE_Chat_main_OTA.c
 * @brief This is a Chat demo that shows how to implement a simple 2-way communication between two BlueNRG devices.
 * It also provides a reference example about how using the Over-The-Air (OTA) bootloader capability. 
 *
 * NOTEs: OTA service support can be enabled through ST_OTA_BTL (preprocessor & linker) options and btl.[ch] files.
 *        OTA Service Manager support requires to build application by enabling only
 *        ST_OTA_BASIC_APPLICATION (preprocessor & linker) options and through btl.[ch] files.
 * <!-- Copyright 2014 by STMicroelectronics.  All rights reserved.       *80*-->

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM.
  -# From the File->Open->Workspace menu, open the IAR workspace
     <tt> ...\\Projects\\Project\\BLE_Chat\\EWARM\\BLE_Chat.eww </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect STLink to JTAG connector  in your board (if available).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG GUI, put the board in DFU mode and download the built binary image.
  -# Connect the application board to a PC USB port. Open a hyperterminal on the
     corresponding USB virtual COMx port with the configuration as described in \ref Serial_IO.

* \subsection IAR_project_configurations IAR project configurations

  - \c Client - Client role configuration 
  - \c Server - Server role configuration
  - \c Server_LowerApp_OTA - Server configuration with OTA service support (Start address 0x08003800)
  - \c Server_HigherApp_OTA - Server configuration with OTA service support (Start address 0x0800E000)
  - \c Server_OTA_Basic - Configuration to be used with OTA Service Manager (standalone BLE OTA bootloader). Start address 0x08006200
  - \c Client_throughput - Client role configuration for throughput test
  - \c Server_throughput - Server role configuration for throughput test

* \section Prebuilt_images Prebuilt images
  - BLE_Chat_Client.hex, BLE_Chat_Server.hex

* \section Jumper_settings Jumper settings
@table
------------------------------------------------------
| Jumper name       |  Description                   | 
------------------------------------------------------
| JP1, if available | USB or Battery supply position | 

@endtable 


* \section Board_supported Board supported
@table
| Board name      | Board revision | NOTE        |
--------------------------------------------------
| STEVAL-IDB002V1 | 3.0            | Eval Kit    |
| STEVAL-IDB003V1 | 1.0            | USB Dongle  |
@endtable

* \section Serial_IO Serial I/O
  The application will listen for keys typed in one node and, on return press, it will send them to the remote node.
  The remote node will listen for RF messages and it will output them in the serial port.
  In other words everything typed in one node will be visible to the other node and viceversa.
@table
| Parameter name  | Value          | Unit        |
--------------------------------------------------
| Baudrate        | 115200         | bit/sec     |
| Data bits       | 8              | bit         |
| Parity          | None           | bit         |
| Stop bits       | 1              | bit         |
@endtable

* \section LEDs_description LEDs description
@table
|                  | STEVAL-IDB002V1 |                     |                      |                  | STEVAL-IDB003V1 |                     |                      |                  |
| LED name         | Client/Server   | Server_LowerApp_OTA | Server_HigherApp_OTA | Server_OTA_Basic | Client/Server   | Server_LowerApp_OTA | Server_HigherApp_OTA | Server_OTA_Basic |           
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| D1               | Not used        | Blinking            |  ON during OTA       | Not used         | NA              | NA                  | NA                   | NA               |
| D2               | Not used        | ON during OTA       |  Blinking            | Blinking         | Not used        | Blinking            | ON during OTA        | Not used         |         
| D3               | Not used        | Not used            |  Not used            | Not used         | Not used        | ON during OTA       | Blinking             | Blinking         |       
| D4               | Not used        | Not used            |  Not used            | Not used         | NA              | NA                  | NA                   | NA               |      
| D5               | Not used        | Not used            |  Not used            | Not used         | NA              | NA                  | NA                   | NA               |          
| D6               | Not used        | Not used            |  Not used            | Not used         | NA              | NA                  | NA                   | NA               |       
@endtable
- NA:   Not Applicable;
- NOTE: When device is connected the led blinking period is faster. 

* \section Buttons_description Buttons description
@table
|                  | STEVAL-IDB002V1 |                     |                      |                  | STEVAL-IDB003V1 |                     |                      |                  |
| Button name      | Client/Server   | Server_LowerApp_OTA | Server_HigherApp_OTA | Server_OTA_Basic | Client/Server   | Server_LowerApp_OTA | Server_HigherApp_OTA | Server_OTA_Basic |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| RESET            | X               | X                   |  X                   | Not used         | NA              | NA                  | NA                   | NA               |
| Push Button      | Not used        | Not used            |  Not used            | (2)              | NA              | NA                  | NA                   | NA               |
| Jostick Sel      | Not used        |(1)                  |  (1)                 | Not used         | NA              | NA                  | NA                   | NA               |
| SW1              | NA              | NA                  |  NA                  | NA               | Not used        | Not used            | Not used             | (2)              |
| SW2              | NA              | NA                  |  NA                  | NA               | Not used        | (1)                 | (1)                  | Not used         | 
@endtable
 - NA : Not Applicable;
 - (1): On Power up or Reset, pressing this key the OTA EEPROM application base address information are cleaned
        (Flash address where last valid BLE application has been downloaded through OTA service);
 - (2): On OTA_Basic scenario, this key is used to jump to the OTA_ServiceManager previously 
        stored on Flash after the OTA_ResetManager. 

* \section DFU_Activation  DFU activation
BlueNRG boards are preprogrammed with a DFU application which allows to upload the 
STM32L micro with a selected binary image through USB. Follow list of actions 
for activating DFU on each supported platforms
@table
| Board  name          | Event                                             | Note               |
------------------------------------------------------------------------------------------------|
| STEVAL-IDB002V1      | Press RESET and Push Button. Release RESET button | LED D2 is toggling |  
| STEVAL-IDB003V1      | Press SW1 button and plug USB dongle on a PC port | LED D3 is toggling |  
@endtable

* \section Usage Usage

This Chat demo has  are 2 roles:
 - The server that expose the Chat service. It is the slave.
 - The client that uses the Chat service. It is the master.

The Chat Service contains 2 Characteristics:
 -# The TX Characteristic: the client can enable notifications on this characteristic. When the server has data to be sent, it will send notifications which will contains the value of the TX Characteristic
 -# The RX Characteristic: it is a writable caracteristic. When the client has data to be sent to the server, it will write a value into this characteristic.

The maximum length of the characteristic value is 20 bytes.

NOTES:
 - The Client_throughput and Server_throughput workspaces allow to target a throughput test (THROUGHPUT_TEST define on the preprocessor options on both workspaces).
 - Program the client on one BlueNRG platform and reset it. The platform will be seen on the PC as a virtual COM port. Open the port in a serial terminal emulator. Client will start 4 seconds after reset.
 - Program the server on a second BlueNRG platform and reset it. The two platforms will try to establish a connection. As soon as they get connected, the slave will 
   send notification to the client of 20 bytes (200 notifications). 
 - After all the notifications are received, the client (master) will print the measured application throughput.

**/
/** @addtogroup BlueNRG_demonstrations_applications
 * BlueNRG Chat demo with OTA capability\see BLE_Chat_main_OTA.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "usb_lib.h"
#include "hw_config.h"
#include "hal_types.h"
#include "hci.h"
#include "bluenrg_aci.h"
#include "gp_timer.h"
#include "hal.h"
#include "osal.h"
#include "gatt_db.h"
#include "gatt_server.h"
#include "hci_const.h"
#include "gap.h"
#include "sm.h"
#include "app_state.h"
#include <stdio.h>
#include "SDK_EVAL_Config.h"

#if defined (ST_OTA_BTL) || defined (ST_OTA_BASIC_APPLICATION)
#ifdef SERVER
/* include OTA bootloader header file */
#include "btl.h"
#endif
#endif



/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** 
  * @brief  Enable febug printf's
  */ 
#ifndef DEBUG
#define DEBUG 1
#endif
      
#define REQUEST_CONN_PARAM_UPDATE 0

/* Private macros ------------------------------------------------------------*/
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define NUM_PACKETS 200 // Only used for throughput test (define THROUGHPUT_TEST)

#ifndef VECTOR_TABLE_BASE_ADDRESS /* Default configuration: no OTA bootloader */
/* default configuration: DFU upgrade is supported */
#define VECTOR_TABLE_BASE_ADDRESS            (0x3000)
#endif

#ifdef ST_OTA_BASIC_APPLICATION
#define APPLICATION_JUMP_ADDRESS (0x08000000+ VECTOR_TABLE_BASE_DFU_OFFSET + VECTOR_TABLE_BASE_RESET_MANAGER_OFFSET)
#endif 

/* Private variables ---------------------------------------------------------*/
volatile int app_flags = SET_CONNECTABLE;
volatile uint16_t connection_handle = 0;
extern uint16_t chatServHandle, TXCharHandle, RXCharHandle;
struct timer l2cap_req_timer;

#if defined (ST_OTA_BTL) || defined (ST_OTA_BASIC_APPLICATION)
#ifdef SERVER
static uint16_t led_blinking_rate = 500; 
#endif
#endif

/** 
  * @brief  Handle of TX,RX  Characteristics.
  */ 
#ifdef CLIENT
uint16_t tx_handle;
uint16_t rx_handle;
#endif 


/* Private function prototypes -----------------------------------------------*/
void Make_Connection(void);
void User_Process(void);

/* Private functions ---------------------------------------------------------*/

/*  User Function where serial received data should be processed */
void processInputData(uint8_t * rx_data, uint16_t data_size);

/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
    int ret;
    
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, VECTOR_TABLE_BASE_ADDRESS);
    
    /* Identify BlueNRG platform */
    SdkEvalIdentification();

    RCC_Configuration();
    
    /* Init I/O ports */
    Init_GPIOs ();
    
    PWR_PVDCmd(DISABLE);
    
    /* Disable FLASH during Sleep  */
    FLASH_SLEEPPowerDownCmd(ENABLE);
    
    /* Enable Ultra low power mode */
    PWR_UltraLowPowerCmd(ENABLE);
    
    PWR_FastWakeUpCmd(ENABLE);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    
    Clock_Init();
    
    /* Delay needed only to be able to acces the JTAG interface after reset
    if it will be disabled later. */
    Clock_Wait(500);    
    
    /* Configure I/O communication channel:
       It requires the void IO_Receive_Data(uint8_t * rx_data, uint16_t data_size) function
       where user received data should be processed */
    SdkEval_IO_Config(processInputData);
    
    /* Delay for debug purpose, in order to see printed data at startup. */
    for(int i = 0; i < 100 ; i++){
        printf(".");
        Clock_Wait(40);
    }

    HCI_Init();
    
    /* Init SPI interface */
    SdkEvalSpiInit(SPI_MODE_EXTI);
    BlueNRG_RST();
#if defined (ST_OTA_BTL) || defined (ST_OTA_BASIC_APPLICATION)
#ifdef SERVER
    SdkEvalLedInit(LED1);
    SdkEvalLedInit(LED2);
#endif 
#endif
    
#ifdef ST_OTA_BASIC_APPLICATION
    /* Initialize the Key push-button */
    SdkEvalPushButtonInit(BUTTON_SCM_PS, BUTTON_MODE_GPIO);    
#endif 
    
    {
#if CLIENT
        uint8_t bdaddr[] = {0xbb, 0x00, 0x00, 0xE1, 0x80, 0x02};
#else
        uint8_t bdaddr[] = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02};
#endif
        ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN,
                                        bdaddr);
        if(ret){
            PRINTF("Setting BD_ADDR failed.\n");
        }
    }
    
    ret = aci_gatt_init();    
    if(ret){
        PRINTF("GATT_Init failed.\n");
    }
    
    {
        uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
#if SERVER
        ret = aci_gap_init(GAP_PERIPHERAL_ROLE, &service_handle, &dev_name_char_handle, &appearance_char_handle);
#else
        ret = aci_gap_init(GAP_CENTRAL_ROLE, &service_handle, &dev_name_char_handle, &appearance_char_handle);
#endif
        if(ret){
            PRINTF("GAP_Init failed.\n");
        }
    }
  
    ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                       OOB_AUTH_DATA_ABSENT,
                                       NULL,
                                       7,
                                       16,
                                       USE_FIXED_PIN_FOR_PAIRING,
                                       123456,
                                       BONDING);
    PRINTF("BLE Stack Initialized.\n");
    
#if  SERVER
    PRINTF("SERVER: BLE Stack Initialized (platform: %d)\n", SdkEvalGetVersion());
    ret = Add_Chat_Service();
    
    if(ret == BLE_STATUS_SUCCESS)
        PRINTF("Service added successfully.\n");
    else
        PRINTF("Error while adding service.\n");
    
#else
    PRINTF("CLIENT: BLE Stack Initialized  (platform: %d)\n", SdkEvalGetVersion());
#endif 
    
#if defined (ST_OTA_BTL) && defined (SERVER)
    /* NOTE: Since Chat application is using hardcoding TX and RX characteristic handles, add BTL service +
       characteristic after the Chat ones */
    ret = Add_Btl_Service();
    if(ret == BLE_STATUS_SUCCESS)
        PRINTF("Btl service added successfully.\n");
    else
        PRINTF("Error while adding Btl service.\n");
#endif
    

    /* -2 dBm output power */
    ret = aci_hal_set_tx_power_level(1,4);
        
    while(1)
    {
#if defined (ST_OTA_BTL) || defined (ST_OTA_BASIC_APPLICATION)
#ifdef SERVER
      static tClockTime startTime = 0;
      //static uint8_t toggle = 0;

      if (Clock_Time() - startTime >led_blinking_rate){    
#if defined(HIGHER_APP) || defined (ST_OTA_BASIC_APPLICATION)
       SdkEvalLedToggle(LED2);
#else
       SdkEvalLedToggle(LED1);
#endif      
       startTime = Clock_Time();
      }
#endif /* end SERVER */
#endif /* end ST_OTA_BTL || ST_OTA_BASIC_APPLICATION*/

#ifdef ST_OTA_BASIC_APPLICATION
      /* Use button to switch to the basic OTA service Manager */
      if (SdkEvalPushButtonGetState(BUTTON_SCM_PS) == RESET)
      {
        /* Add delay to avoid conlict with DFU activation */
        Clock_Wait(2000);
        Switch_To_OTA_Service_Manager_Application(APPLICATION_JUMP_ADDRESS);
      }
#endif 
      
        HCI_Process();
        User_Process();
    }
}

/**
  * @brief  Make the device connectable
  * @param  None 
  * @retval None
  */
void Make_Connection(void)
{  
    tBleStatus ret;
    
    
#if CLIENT
    tBDAddr bdaddr = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02};
    
    ret = aci_gap_create_connection(0x4000, 0x4000, PUBLIC_ADDR, bdaddr, PUBLIC_ADDR, 40, 40, 0, 60, 2000 , 2000); 
    
    if (ret != 0){
        PRINTF("Error while starting connection.\n");
        Clock_Wait(100);        
	}
#else
    
    const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G','_','C','h','a','t'};
    
#if defined (ST_OTA_BTL) && defined (SERVER)
    /* Add OTA service UUID to scan response */
    hci_le_set_scan_resp_data(18,BTLServiceUUID4Scan); 
#else
    /* disable scan response */
    hci_le_set_scan_resp_data(0,NULL);
#endif 
    
    PRINTF("General Discoverable Mode ");
    ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                   13, local_name, 0, NULL, 0, 0);

    PRINTF("%d\n",ret);
#endif
}

/**
  * @brief  This function is called when there is a LE Connection Complete event.
  * @param  addr Address of peer device
  * @param  handle Connection handle
  * @retval None
  */
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{    
    APP_FLAG_SET(CONNECTED); 
    connection_handle = handle;
    
    PRINTF("Connected to device:");
    for(int i = 5; i > 0; i--){
        PRINTF("%02X-", addr[i]);
    }
    PRINTF("%02X\n", addr[0]);
#if defined (ST_OTA_BTL) || defined (ST_OTA_BASIC_APPLICATION)
#ifdef SERVER
    led_blinking_rate = 125;
#endif
#endif
    
#if REQUEST_CONN_PARAM_UPDATE
    APP_FLAG_CLEAR(L2CAP_PARAM_UPD_SENT);
    Timer_Set(&l2cap_req_timer, CLOCK_SECOND*2);
#endif
    
}

/**
  * @brief  This function is called when the peer device get disconnected.
  * @param  None 
  * @retval None
  */
void GAP_DisconnectionComplete_CB(void)
{
    APP_FLAG_CLEAR(CONNECTED);
    PRINTF("Disconnected\n");
    /* Make the device connectable again. */
    APP_FLAG_SET(SET_CONNECTABLE);
    APP_FLAG_CLEAR(NOTIFICATIONS_ENABLED);
    
    APP_FLAG_CLEAR(START_READ_TX_CHAR_HANDLE);
    APP_FLAG_CLEAR(END_READ_TX_CHAR_HANDLE); 
    APP_FLAG_CLEAR(START_READ_RX_CHAR_HANDLE);
    APP_FLAG_CLEAR(END_READ_RX_CHAR_HANDLE); 
#if defined (ST_OTA_BTL) || defined (ST_OTA_BASIC_APPLICATION)
#ifdef SERVER
    led_blinking_rate = 500;
#endif
#endif
}

/**
  * @brief  This function is called when there is a notification from the sever.
  * @param  attr_handle Handle of the attribute
  * @param  attr_len    Length of attribute value in the notification
  * @param  attr_value  Attribute value in the notification
  * @retval None
  */
void GATT_Notification_CB(uint16_t attr_handle, uint8_t attr_len, uint8_t *attr_value)
{
#if THROUGHPUT_TEST && CLIENT
    static tClockTime time, time2;
    static int packets=0;     
    
    if(attr_handle == tx_handle+1){ 
        if(packets==0){
            printf("Test start\n");
            time = Clock_Time();
        }
        
        for(int i = 0; i < attr_len; i++)
            printf("%c", attr_value[i]);
        
        packets++;
        
        if(packets == NUM_PACKETS){
            time2 = Clock_Time();
            tClockTime diff = time2-time;
            printf("\n%d packets. Elapsed time: %d ms. App throughput: %.2f kbps.\n", NUM_PACKETS, diff, (float)NUM_PACKETS*20*8/diff);
        }        
        
    }
#elif CLIENT
    
    if(attr_handle == tx_handle+1){
      for(int i = 0; i < attr_len; i++)
          printf("%c", attr_value[i]);
    }
#endif
}

void User_Process(void)
{
    if(APP_FLAG(SET_CONNECTABLE)){
        Make_Connection();
        APP_FLAG_CLEAR(SET_CONNECTABLE);
    }

#if REQUEST_CONN_PARAM_UPDATE    
    if(APP_FLAG(CONNECTED) && !APP_FLAG(L2CAP_PARAM_UPD_SENT) && Timer_Expired(&l2cap_req_timer)){
        aci_l2cap_connection_parameter_update_request(connection_handle, 8, 16, 0, 600);
        APP_FLAG_SET(L2CAP_PARAM_UPD_SENT);
    }
#endif
    
#if CLIENT
    
    /* Start TX handle Characteristic discovery if not yet done */
    if (APP_FLAG(CONNECTED) && !APP_FLAG(END_READ_TX_CHAR_HANDLE))
    {
      if (!APP_FLAG(START_READ_TX_CHAR_HANDLE))
      {
        /* Discovery TX characteristic handle by UUID 128 bits */
        
         const uint8_t charUuid128_TX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
         
         aci_gatt_discovery_characteristic_by_uuid(connection_handle, 0x0001, 0xFFFF,UUID_TYPE_128,
                                                   charUuid128_TX);
         APP_FLAG_SET(START_READ_TX_CHAR_HANDLE);
      }
    }
    /* Start RX handle Characteristic discovery if not yet done */
    else if (APP_FLAG(CONNECTED) && !APP_FLAG(END_READ_RX_CHAR_HANDLE))
    {
      /* Discovery RX characteristic handle by UUID 128 bits */
      if (!APP_FLAG(START_READ_RX_CHAR_HANDLE))
      {
        /* Discovery TX characteristic handle by UUID 128 bits */
        
         const uint8_t charUuid128_RX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};
         aci_gatt_discovery_characteristic_by_uuid(connection_handle, 0x0001, 0xFFFF,UUID_TYPE_128,
                                                   charUuid128_RX);
         APP_FLAG_SET(START_READ_RX_CHAR_HANDLE);
       }
    }
    
    if(APP_FLAG(CONNECTED) && APP_FLAG(END_READ_TX_CHAR_HANDLE) && APP_FLAG(END_READ_RX_CHAR_HANDLE) && !APP_FLAG(NOTIFICATIONS_ENABLED)){
        uint8_t client_char_conf_data[] = {0x01, 0x00}; // Enable notifications
        struct timer t;
        Timer_Set(&t, CLOCK_SECOND*10);
        
        while(aci_gatt_write_charac_descriptor(connection_handle, tx_handle+2, 2, client_char_conf_data)==BLE_STATUS_NOT_ALLOWED){ 
            // Radio is busy.
            if(Timer_Expired(&t)) break;
        }
        APP_FLAG_SET(NOTIFICATIONS_ENABLED);
      }
#endif
    
    
#if THROUGHPUT_TEST && SERVER
    
    static uint8_t test_done = FALSE;
    
    if(APP_FLAG(CONNECTED) && APP_FLAG(END_READ_RX_CHAR_HANDLE) && APP_FLAG_SET(END_READ_RX_CHAR_HANDLE) && !test_done && APP_FLAG(NOTIFICATIONS_ENABLED)){
    
        uint8_t data[20] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','G','H','I','J'};
        
        static tClockTime time, time2;
        time = Clock_Time();
        
        for(int i = 0; i < NUM_PACKETS; i++){
            
            struct timer t;
            Timer_Set(&t, CLOCK_SECOND*10);
                            
            while(aci_gatt_update_char_value(chatServHandle, TXCharHandle, 0, 20, data)==BLE_STATUS_INSUFFICIENT_RESOURCES)
            {
              // Radio is busy (buffer full).
              if(Timer_Expired(&t))
                  break;
            }
        
        }
        
        time2 = Clock_Time();
        tClockTime diff = time2-time;
        printf("\n%d packets. Elapsed time: %d ms. App throughput: %.2f kbps.\n", NUM_PACKETS, diff, (float)NUM_PACKETS*20*8/diff);
        
        test_done = TRUE;
    }
#endif
    
}

/**
  * @brief  This function is called whenever there is an ACI event to be processed.
  * @note   Inside this function each event must be identified and correctly
  *         parsed.
  * @param  pckt  Pointer to the ACI packet
  * @retval None
  */
void HCI_Event_CB(void *pckt)
{
    hci_uart_pckt *hci_pckt = pckt;
    hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
    
    if(hci_pckt->type != HCI_EVENT_PKT)
        return;
    
    switch(event_pckt->evt){
        
    case EVT_DISCONN_COMPLETE:
        {
            GAP_DisconnectionComplete_CB();
        }
        break;
        
    case EVT_LE_META_EVENT:
        {
            evt_le_meta_event *evt = (void *)event_pckt->data;
            
            switch(evt->subevent){
            case EVT_LE_CONN_COMPLETE:
                {
                    evt_le_connection_complete *cc = (void *)evt->data;
                    GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
                }
                break;
            }
        }
        break;
        
    case EVT_VENDOR:
        {
            evt_blue_aci *blue_evt = (void*)event_pckt->data;
            switch(blue_evt->ecode){
                
            case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
                {
                    evt_gatt_attr_modified *evt = (evt_gatt_attr_modified*)blue_evt->data;
                    
#if defined (ST_OTA_BTL) && defined (SERVER)        
                 
                    OTA_Write_Request_CB(evt);  
#endif
                    Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);                    
                }
                break;
            case EVT_BLUE_GATT_NOTIFICATION:
                {
                    evt_gatt_attr_notification *evt = (evt_gatt_attr_notification*)blue_evt->data;
                    GATT_Notification_CB(evt->attr_handle, evt->event_data_length - 2, evt->attr_value);
                }
                break;
#if defined (ST_OTA_BTL) && defined (SERVER) 
            /* We need to indicate to the stack that the response can be sent to the client */
            case EVT_BLUE_GATT_READ_PERMIT_REQ:
                {         
                    if(connection_handle !=0)
                        aci_gatt_allow_read(connection_handle);                 
                }
                break;      
#endif
             case EVT_BLUE_L2CAP_CONN_UPD_RESP:
                {
                    evt_l2cap_conn_upd_resp *resp = (void*)blue_evt->data;
                    if(resp->result){
                        PRINTF("> Connection parameters rejected.\n");
                    }
                    else{
                        PRINTF("> Connection parameters accepted.\n");
                    }
                }
                break;
#ifdef CLIENT           
              case EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP:
                {
                    evt_gatt_disc_read_char_by_uuid_resp *resp = (void*)blue_evt->data;
                    
                    if (APP_FLAG(START_READ_TX_CHAR_HANDLE) && !APP_FLAG(END_READ_TX_CHAR_HANDLE))
                    {
                      tx_handle = resp->attr_handle;
                      PRINTF("TX Char Handle %04X\n", tx_handle);
                    }
                    else if (APP_FLAG(START_READ_RX_CHAR_HANDLE) && !APP_FLAG(END_READ_RX_CHAR_HANDLE))
                    {
                      rx_handle = resp->attr_handle;
                      PRINTF("RX Char Handle %04X\n", rx_handle);
                    }
                }
                break;  
                
                case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
                {
                  /* Wait for gatt procedure complete event trigger related to Discovery Charac by UUID */
                  evt_gatt_procedure_complete *pr = (void*)blue_evt->data;
                  
                  if (APP_FLAG(START_READ_TX_CHAR_HANDLE) && !APP_FLAG(END_READ_TX_CHAR_HANDLE))
                  {
                    APP_FLAG_SET(END_READ_TX_CHAR_HANDLE);
                  }
                  else if (APP_FLAG(START_READ_RX_CHAR_HANDLE) && !APP_FLAG(END_READ_RX_CHAR_HANDLE))
                  {
                    APP_FLAG_SET(END_READ_RX_CHAR_HANDLE);
                  }
                }
                break;
#endif         
            }
        }
        break;
    }
    
}


#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    
    /* Infinite loop */
    while (1)
    {}
}
#endif

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
