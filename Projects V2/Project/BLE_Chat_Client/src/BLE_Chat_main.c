/******************** (C) COPYRIGHT 2014 STMicroelectronics ********************
* File Name          : BLE_Chat_main.c
* Author             : AMS - AAS Division
* Version            : V1.0.1
* Date               : 10-February-2014
* Description        : BlueNRG main file for Chat demo
*********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/**
 * @file  BLE_Chat_main.c
 * @brief This is a Chat demo that shows how to implement a simple 2-way communication between two BlueNRG devices.
 *
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
|                  | STEVAL-IDB002V1 | STEVAL-IDB003V1 |                 
| LED name         | Client/Server   | Client/Server   | 
--------------------------------------------------------
| D1               | Not used        | NA              |        
| D2               | Not used        | Not used        |     
| D3               | Not used        | Not used        |     
| D4               | Not used        | NA              |      
| D5               | Not used        | NA              |       
| D6               | Not used        | NA              |       
@endtable
 - NA : Not Applicable;

* \section Buttons_description Buttons description
@table
|                  | STEVAL-IDB002V1 | STEVAL-IDB003V1 |                 
| Button name      | Client/Server   | Client/Server   | 
--------------------------------------------------------
| RESET            | X               | NA              |  
| Push Button      | Not used        | NA              |   
| Jostick Sel      | Not used        | NA              |    
| SW1              | NA              | Not used        |     
| SW2              | NA              | Not used        |       
@endtable
 - NA : Not Applicable;

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
 - After all the notifications are received, the measured application throughput will be displayed.

**/
/** @addtogroup BlueNRG_demonstrations_applications
 * BlueNRG Chat demo \see BLE_Chat_main.c for documentation.
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
#include "led.h"
#include "push_button.h"
#include "timer.h"
#include "slave_devices.h"
#include "flags.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "syncr.h"

#include "SDK_EVAL_Config.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** 
  * @brief  Enable debug printf's
  */ 
#ifndef DEBUG
#define DEBUG 1
#endif
      
#define REQUEST_CONN_PARAM_UPDATE 0     //Flag used to check connection parameters

/* Private macros ------------------------------------------------------------*/
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define NUM_PACKETS 200 // Only used for throughput test (define THROUGHPUT_TEST)

#ifndef VECTOR_TABLE_BASE_ADDRESS 
/* default configuration: DFU upgrade is supported */
#define VECTOR_TABLE_BASE_ADDRESS            (0x3000)   //Vector table bass address
#endif

/* Private variables ---------------------------------------------------------*/
volatile int app_flags = SET_CONNECTABLE;                       //Set the devices connectable by setting a flag
volatile uint16_t connection_handle = 0;                        //Unique connection handle for a given connection
extern uint16_t chatServHandle, TXCharHandle, RXCharHandle, syncReq = 0;     //Unique handles for the chat service, TX characteristic and RX characteristic
struct timer l2cap_req_timer;                                   //Timer used when connection parameters are requested
//volatile int numConnected = 0;
volatile int flag_scan_complete = 0;
volatile int flag_connection_complete = 0;
volatile int index = 0;
volatile uint32_t bstimei = 0;
volatile uint32_t hhtimei = 0;
extern uint32_t offsetp = 0;


/** 
  * @brief  Handle of TX,RX  Characteristics.
  */ 
#ifdef CLIENT
uint16_t tx_handle;     //Unique TX characteristic handle
uint16_t rx_handle;     //Unique RX characteristic handle
#endif 


/* Private function prototypes -----------------------------------------------*/
void Make_Connection(void);     //Used to make BLE connections
void User_Process(void);        //Used to get characteristic handles and enable notifications
void send_times();              // Check times and send them to syncr if both are set

/* Private functions ---------------------------------------------------------*/

/*  User Function where serial received data should be processed */
void processInputData(uint8_t * rx_data, uint16_t data_size);   //Used to process received data

/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
    int ret;    //Used to check status
    
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, VECTOR_TABLE_BASE_ADDRESS); //Set the Nested Vector Interrupt Controller table bass address
    
    /* Identify BlueNRG platform */
    SdkEvalIdentification();    //Figure out what BlueNRG platform is being used

    RCC_Configuration();        //Configure the Reset and Clock Control (definition located in hw_config.h)
    
    /* Init I/O ports */
    Init_GPIOs ();      //Configure the General Purpose Input/Output ((definition located in hw_config.h))
    
    PWR_PVDCmd(DISABLE);        //Disable Programmable Voltage Detector
    
    /* Initialize LEDs and button */
    initLED(0); //check LED.c for more info
    initLED(3);
    initLED(4);
    initButton();
    
    /* Initialize debouncer timer */
    initTimer(7,200);   //Timer used to debounce button
    enableTimerInterrupt(7);
    
    /* Initialize (measuring)event timer */
    initTimer(2,1000);  //Check timer.c for more info
    enableTimerInterrupt(2);
    initTimer(4,100);
    enableTimerInterrupt(4);
    initTimer(6,5000);
    enableTimerInterrupt(6);
    
    /* Initialize slave addresses */
    initDevices();
    
    /* Disable FLASH during Sleep  */
    FLASH_SLEEPPowerDownCmd(ENABLE);    //Disable FLASH memory access during sleep
    
    /* Enable Ultra low power mode */
    PWR_UltraLowPowerCmd(ENABLE);       //Enable ultra low power mode
    
    PWR_FastWakeUpCmd(ENABLE);  //Enable fast wake up
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);     //Configure NVIC priority
    
    Clock_Init();       //Initialize clock library that gives a sample time reference to the BLE stack
    
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
        Clock_Wait(70);
    }

    HCI_Init(); //Initialize the Host Controller Interface (this is needed for communication)
    
    /* Init SPI interface */
    SdkEvalSpiInit(SPI_MODE_EXTI);      //Initialize the SPI communication which uses an external interrupt
    BlueNRG_RST();      //Reset the BlueNRG
    
    {
#if CLIENT
        uint8_t bdaddr[] = {0xbb, 0x00, 0x00, 0xE1, 0x80, 0x02};        //CLIENT ADDRESS
        uint8_t role[] = {0x03};        //Master role (can connect with up to 8 slaves)
        ret = aci_hal_write_config_data(CONFIG_DATA_ROLE, 0x01, role);  //Configure client as master (allowing up to 8 slave connections)
        if(ret){
            PRINTF("Configuration as master failed.\n");        //If configuration fails, print a message
        }
#else
        uint8_t bdaddr[] = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02};        //Slave address (essentially commented out in the client code)
#endif
        ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN,    //Configure device address
                                        bdaddr);
        if(ret){
            PRINTF("Setting BD_ADDR failed.\n");        //Failure message
        }
    }
    
    ret = aci_gatt_init();    //Initialize the Generic Attribute Profile
    if(ret){
        PRINTF("GATT_Init failed.\n");  //Failure message
    }
    
    {
        uint16_t service_handle, dev_name_char_handle, appearance_char_handle;  //Handles for a service that each BLE device must implement
#if SERVER
        ret = aci_gap_init(GAP_PERIPHERAL_ROLE, &service_handle, &dev_name_char_handle, &appearance_char_handle);       //Initialize the Generic Access Profile 
#else
        ret = aci_gap_init(GAP_CENTRAL_ROLE, &service_handle, &dev_name_char_handle, &appearance_char_handle);          //The client is initialized in the central role and the servers are initialized in peripheral role
#endif
        if(ret){
            PRINTF("GAP_Init failed.\n");       //Error message
        }
    }
  
    ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,        //Make devices use a pin for connection
                                       OOB_AUTH_DATA_ABSENT,
                                       NULL,
                                       7,
                                       16,
                                       USE_FIXED_PIN_FOR_PAIRING,
                                       123456,
                                       BONDING);
    PRINTF("BLE Stack Initialized.\r\n");
    
#if  SERVER
    PRINTF("SERVER: BLE Stack Initialized (platform: %d)\n\r", SdkEvalGetVersion());
    ret = Add_Chat_Service();   //Add the chat service along with its TX and RX characteristics
    
    if(ret == BLE_STATUS_SUCCESS)
        PRINTF("Service added successfully.\n");
    else
        PRINTF("Error while adding service.\n");
    
#else
    PRINTF("CLIENT: BLE Stack Initialized  (platform: %d)\n\r", SdkEvalGetVersion());
#endif 
    
    /* -2 dBm output power */
    ret = aci_hal_set_tx_power_level(1,4);      //Set transmission power level
        
/****************************** Main Execution Loop ***************************/
    while(1)
    { 
        //printf("main\r\n");
       
        HCI_Process();          //Process any transmission,reception,etc.
        User_Process();         //Update connections and get characteristic handles if needed
        
        /*//Check for button presses
        if (!GPIO_ReadInputDataBit(GPIOC, BUTTON) && buttonOK)       //If the button has been pressed
        {
            buttonOK = false;
            startTimer(7);      //Debounce timer
            //Send stimulate command to other blueNRG
            uint8_t data_buffer[] = {0x45,0x56,0x45,0x4e,0x54,0x0d};    //ASCII for "EVENT"
            uint16_t Nb_bytes = 6;      //Data buffer size
            //processInputData(data_buffer, Nb_bytes);    //Transmit the data buffer
            //Schedule measuring event
            startTimer(2);
        }*/
        if(syncReq == 1)
        { 
        syncReq = 0;  
        uint8_t a[3]={'0','T','\n'};
        uint8_t b[3]={'1','T','\n'};
        uint16_t sizeOfa = sizeof(a)/sizeof(a[0]);
	uint16_t sizeOfb = sizeof(b)/sizeof(b[0]);
	processInputData(a, sizeOfa);
	processInputData(b, sizeOfb);
        }
    }
}


/**
  * @brief  Make the device connectable and make connections
  * @param  None 
  * @retval None
  */
void Make_Connection(void)
{  
    tBleStatus ret;     //Used to check status
    
    
#if CLIENT
    
    /* Start general discovery procedure */
    if(0)
    {
      ret = aci_gap_start_general_discovery_proc(0x4000, 0x4000,PUBLIC_ADDR, 0x00);       //Scan for devices
      if (ret != 0){
            PRINTF("Error while starting general discovery.\r\n");
            Clock_Wait(100);        
            }
      //Wait for procedure to complete
      while(!flag_scan_complete)
      {
        //printf("case1\r\n");
        HCI_Process();    //Continue to process incoming data. The EVT_BLUE_GAP_PROCEDURE_COMPLETE event occurs when scanning is complete
      }
      Clock_Wait(100);
    }
    
   
        ret = aci_gap_create_connection(0x4000, 0x4000, PUBLIC_ADDR, slaves[index].bdaddr, PUBLIC_ADDR, 0x0020, 0x0020, 0, 0x0064, 0 , 0x03e8);
        if (ret != 0){
            PRINTF("Error while starting connection to server %d.\n", index);
            Clock_Wait(100);
        }
        while(!flag_connection_complete)
        {
         //printf("%d\n\r",Clock_Timeus());
            HCI_Process();    //EVT_LE_META_EVENT event triggered when connection is complete
        }
        flag_connection_complete = 0;
 
#else
    
    const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G','_','C','h','a','t'};    //Name server advertises
    
    /* disable scan response */
    hci_le_set_scan_resp_data(0,NULL);
    
    PRINTF("General Discoverable Mode ");       //Sets server in discoverable mode
    ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                   13, local_name, 0, NULL, 0, 0);

    PRINTF("%d\n\r",ret);
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
    APP_FLAG_SET(CONNECTED);    //Change flag to indicate that the device is connected
    for(int i = 0; i < numSlaves; i++)
    {
      if(slaves[i].bdaddr[0] == addr[0])
      {
        slaves[i].connection_handle = handle; //Store the connection handle
        printf("device %d connection handle: %x\r\n", i, handle);
        break;
      }
    }
    
    //Turn on Connection LED
    turnLED(3,ON);      //Turn on blue LED to indicate that device is connected
    flag_connection_complete = 1;
    printf("Connected to: ");   //Print the address of the device that was connected to
    for(int i = 5; i > 0; i--){
        PRINTF("%02X-", addr[i]);
    }
    PRINTF("%02X\n\r", addr[0]);
    
#if REQUEST_CONN_PARAM_UPDATE   //Used to request a connection parameter update
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
  //Turn off Connection LED
  //numConnected -= 1;
  turnLED(3,OFF);       //Turn off connection LED
  flag_scan_complete = 0;
  flag_connection_complete = 0;
    APP_FLAG_CLEAR(CONNECTED);  //Clear flags
    PRINTF("Disconnected\n\r");
    /* Make the device connectable again. */
    APP_FLAG_SET(SET_CONNECTABLE);
    APP_FLAG_CLEAR(NOTIFICATIONS_ENABLED);
    
    APP_FLAG_CLEAR(START_READ_TX_CHAR_HANDLE);
    APP_FLAG_CLEAR(END_READ_TX_CHAR_HANDLE);
    APP_FLAG_CLEAR(START_READ_RX_CHAR_HANDLE); 
    APP_FLAG_CLEAR(END_READ_RX_CHAR_HANDLE); 
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
#if THROUGHPUT_TEST && CLIENT   //Only used for throughput testing
    static tClockTime time, time2;
    static int packets=0;     
    
    if(attr_handle == tx_handle+1){ 
        if(packets==0){
            printf("Test start\n\r");
            time = Clock_Time();
        }
        
        for(int i = 0; i < attr_len; i++)
          printf("%c", attr_value[i]);
        
        printf("[RX: %d]", packets);//TBR
        
        packets++;
        
        if(packets == NUM_PACKETS){
            time2 = Clock_Time();
            tClockTime diff = time2-time;
            printf("\n%d packets. Elapsed time: %d ms. App throughput: %.2f kbps.\n\r", NUM_PACKETS, diff, (float)NUM_PACKETS*20*8/diff);
        }        
        
    }
#elif CLIENT    //When the client receives a notafication (message) from the server
    for(int i = 0; i < numSlaves; i++)
    {
      if(attr_handle == slaves[i].tx_handle+1){     //If the notification is from the TX attribute...
        printf("received ");
        
        if(attr_value[0] == 'B'){
          printf("BS: ");
          if(attr_value[1] == 'T'){
            char array[40];
            int index = 2;
            int j = 0;
            
            while(attr_value[index] != '\n'){
              array[j] = attr_value[index];
              j++;
              index++;
            }
              bstimei = s2i(array);
              send_times();
            
          }
            else if(attr_value[1] == 'E'){
              char array[40];
              array[0] = '1';
              char array1[40];
              array1[0] = '0';
              for(int index = 1; index <attr_len; index++){
                  array[index] = attr_value[index];
                  array1[index] = attr_value[index];
                  
              }
                processInputData(array, attr_len);
                processInputData(array1, attr_len);
          }
        }
        else if (attr_value[0] == 'H'){
          printf("HH: ");
            if(attr_value[1] == 'T'){
            char array[40];
            int index = 2;
            int j = 0;
            
            while(attr_value[index] != '\n'){
              array[j] = attr_value[index];
              j++;
              index++;
            }
              hhtimei = s2i(array);
              send_times();
            
          }
            else if(attr_value[1] == 'E'){
              char array[40];
              array[0] = '1';
              char array1[40];
              array1[0] = '0';
              for(int index = 1; index <attr_len; index++){
                  array[index] = attr_value[index];
                  array1[index] = attr_value[index];
                  
              }
                processInputData(array, attr_len);
                processInputData(array1, attr_len);            
          }
        }  
        for(int i = 0; i < attr_len; i++) //Print out the received message
          printf("%c",attr_value[i]);
        printf("\r\n");
        break;
      }
    }
#endif
}


void send_times(){
  
  if((bstimei != 0) && (hhtimei != 0)){
    
    offsetp = calculate( bstimei,  hhtimei, offsetp);
    
    bstimei = 0;
    hhtimei = 0;
    
    
    char number[6];
    sprintf(number, "%d", offsetp);
    char messageB[8];
    char messageH[8];
    messageB[0] = '0';
    messageH[0] = '1';
    messageB[1] = 'O';
    messageH[1] = 'O';
    int j = 2;
    for(int index = 0; index < 4; index++){
      messageB[j] = number[index];
      messageH[j] = number[index];
      j++;
    }
    messageB[6] = '\n';
    messageH[6] = '\n';
    messageB[7] = '\0';
    messageH[7] = '\0';
    processInputData(messageB, 8);
    processInputData(messageH, 8);
    
    
    
  } 
  
  
}

void User_Process(void)
{
    if(arrayFlags[index].set_connectable){
        Make_Connection();      //If devices need to be connected, connect them
        changeFlag(CLEAR, &arrayFlags[index].set_connectable);        //Update connection flag status
    }

#if REQUEST_CONN_PARAM_UPDATE    
    if(APP_FLAG(CONNECTED) && !APP_FLAG(L2CAP_PARAM_UPD_SENT) && Timer_Expired(&l2cap_req_timer)){
        printf("got here\r\n");
        aci_l2cap_connection_parameter_update_request(connection_handle, 8, 16, 0, 600);
        APP_FLAG_SET(L2CAP_PARAM_UPD_SENT);
    }
#endif
    
#if CLIENT      //Each loop the flags are checked to see what needs to be done
                //Things such as discovering the TX/RX characteristic handles and enabling notifications are done
    
    
    /* Start TX handle Characteristic discovery if not yet done */
    if (APP_FLAG(CONNECTED) && !arrayFlags[index].end_read_TX_char_handle)
    {
      if (!arrayFlags[index].start_read_TX_char_handle)
      {
        /* Discovery TX characteristic handle by UUID 128 bits */
        
         const uint8_t charUuid128_TX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
         
         aci_gatt_disc_charac_by_uuid(slaves[index].connection_handle, 0x0001, 0xFFFF,UUID_TYPE_128,
                                                   charUuid128_TX);
         changeFlag(SET, &arrayFlags[index].start_read_TX_char_handle);
      }
    }
    /* Start RX handle Characteristic discovery if not yet done */
    else if (APP_FLAG(CONNECTED) && !arrayFlags[index].end_read_RX_char_handle)
    {
      //printf("Haven't finished reading RX handle");
      /* Discovery RX characteristic handle by UUID 128 bits */
      if (!arrayFlags[index].start_read_RX_char_handle)
      {
        /* Discovery TX characteristic handle by UUID 128 bits */
        //printf("attempting to read RX handle\r\n");
         const uint8_t charUuid128_RX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};
         aci_gatt_disc_charac_by_uuid(slaves[index].connection_handle, 0x0001, 0xFFFF,UUID_TYPE_128,
                                                   charUuid128_RX);
         changeFlag(SET, &arrayFlags[index].start_read_RX_char_handle);
       }
    }
    
    if(APP_FLAG(CONNECTED) && arrayFlags[index].end_read_TX_char_handle && arrayFlags[index].end_read_RX_char_handle && !arrayFlags[index].notifications_enabled){
        uint8_t client_char_conf_data[] = {0x01, 0x00}; // Enable notifications
        struct timer t;
        Timer_Set(&t, CLOCK_SECOND*10);
        
        while(aci_gatt_write_charac_descriptor(slaves[index].connection_handle, slaves[index].tx_handle+2, 2, client_char_conf_data)==BLE_STATUS_NOT_ALLOWED){ //TX_HANDLE;
            // Radio is busy.
            if(Timer_Expired(&t)) break;
        }
        changeFlag(SET, &arrayFlags[index].notifications_enabled);
      }
    else if(APP_FLAG(CONNECTED) && arrayFlags[index].end_read_TX_char_handle && arrayFlags[index].end_read_RX_char_handle && arrayFlags[index].notifications_enabled){
        int numdevices = sizeof(slaves)/sizeof(sDevice);
        if(index < numdevices - 1)
          index++;        
    }
#endif
    
    
#if THROUGHPUT_TEST && SERVER   //Used for throughput testing
    
    printf("Throughput test\r\n");
    static uint8_t test_done = FALSE;
    
    if(APP_FLAG(CONNECTED) && !test_done && APP_FLAG(NOTIFICATIONS_ENABLED)){
    
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
        printf("\n%d packets. Elapsed time: %d ms. App throughput: %.2f kbps.\n\r", NUM_PACKETS, diff, (float)NUM_PACKETS*20*8/diff);
        
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
void HCI_Event_CB(void *pckt)   //This function is called when ACI events occur (such as when a connection is complete)
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
                    Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);                    
                }
                break;
            case EVT_BLUE_GATT_NOTIFICATION:
                {
                    evt_gatt_attr_notification *evt = (evt_gatt_attr_notification*)blue_evt->data;
                    GATT_Notification_CB(evt->attr_handle, evt->event_data_length - 2, evt->attr_value);
                }
                break;
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
                    
                    if (arrayFlags[index].start_read_TX_char_handle && !arrayFlags[index].end_read_TX_char_handle)
                    {
                      slaves[index].tx_handle = resp->attr_handle;
                      printf("Device %d TX Char Handle: %04X\n\r", index, slaves[index].tx_handle);
                    }
                    else if (arrayFlags[index].start_read_RX_char_handle && !arrayFlags[index].end_read_RX_char_handle)
                    {
                      slaves[index].rx_handle = resp->attr_handle;
                      printf("Device %d RX Char Handle: %04X\n\r", index, slaves[index].rx_handle);
                    }
                }
                break;  
                
                case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
                {
                  /* Wait for gatt procedure complete event trigger related to Discovery Charac by UUID */
                  //evt_gatt_procedure_complete *pr = (void*)blue_evt->data;
                  
                  if (arrayFlags[index].start_read_TX_char_handle && !arrayFlags[index].end_read_TX_char_handle)
                  {
                    changeFlag(SET, &arrayFlags[index].end_read_TX_char_handle);
                    //printf("tried to set TX flag\r\nTX flag is %d\r\n",arrayFlags[index].end_read_TX_char_handle);
                  }
                  else if (arrayFlags[index].start_read_RX_char_handle && !arrayFlags[index].end_read_RX_char_handle)
                  {
                    changeFlag(SET, &arrayFlags[index].end_read_RX_char_handle);
                  }
                }
                break;
                
                case EVT_BLUE_GAP_PROCEDURE_COMPLETE:
                {
                  flag_scan_complete = 1;
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
    {
      printf("\r\nFailed\r\n");
      Clock_Wait(10000);
    }
}
#endif

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
