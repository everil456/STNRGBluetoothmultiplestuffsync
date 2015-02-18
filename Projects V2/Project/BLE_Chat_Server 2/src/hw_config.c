/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : hw_config.c
* Author             : AMS - HEA&RF BU
* Version            : V1.0.0
* Date               : 04-Oct-2013
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
 #include "stm32l1xx_it.h"
 
#include <stdio.h>
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "platform_config.h"
#include "usb_pwr.h"
#include "gp_timer.h"
#include "hal.h"
#include "hci.h"
#include "hal_lis3dh.h"
#include "hal_stlm75.h"
#include "hal_lps25h.h"
#include <string.h>

#include "SDK_EVAL_Config.h"

#ifdef __GNUC__
# define _LLIO_STDIN ((int) stdin)
# define _LLIO_STDOUT ((int) stdout)
# define _LLIO_STDERR ((int) stderr)
# define _LLIO_ERROR  (-1)
#else
# ifdef __ICCARM__
# include <yfuns.h>
# endif
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef ENABLE_USB
uint8_t  USART_Rx_Buffer [USART_RX_DATA_SIZE]; 
#endif
uint32_t USART_Rx_ptr_in = 0;
volatile uint32_t USART_Rx_ptr_out = 0;
uint32_t USART_Rx_length  = 0;

uint8_t  USB_Tx_State = 0;


static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Variable Name  : timer_expired
* Description    : This variable is set to TRUE when the timer is expired.
*                  It is not automatically set to FALSE.
*******************************************************************************/
uint8_t user_timer_expired = FALSE;


/**
  * @brief  Configures peripherals' clock.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
  
  /* Enable the GPIOs Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC| RCC_AHBPeriph_GPIOD| RCC_AHBPeriph_GPIOE| RCC_AHBPeriph_GPIOH, ENABLE);     
    
  /* Enable SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG , ENABLE);
  
  /* Set low power configuration */
  RCC->AHBLPENR = RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN;   
  RCC->APB1LPENR = RCC_APB1ENR_PWREN|RCC_APB1ENR_TIM2EN;
  RCC->APB2LPENR = 0;  
}

/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures all GPIOs that as to be set ASAP.
*                  Called at startup.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{

}

/**
  * @brief  To initialize the I/O ports
  * @caller main
  * @param None
  * @retval None
  */
void  Init_GPIOs(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIOD->MODER   = 0xFFFFFFFF;
  GPIOE->MODER   = 0xFFFFFFFF;
  GPIOH->MODER   = 0xFFFFFFFF;

#ifndef ENABLE_USB  
   /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) except debug pins. */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4| GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_8 \
                                 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |  GPIO_Pin_12;
#else
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4| GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_8 \
                                 | GPIO_Pin_9 | GPIO_Pin_10;
#endif
  
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_8 \
                                 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |  GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4| GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_8 \
                                 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |  GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
   
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4| GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_8 \
                                 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |  GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;   
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
}


/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
  /* Enable USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
}

void LIS3DH_Interrupts_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef  EXTI_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  
  /* Configure external interrupt pin. */
  GPIO_InitStructure.GPIO_Pin =  LIS_A_INT2_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LIS_A_INT2_GPIO_PORT, &GPIO_InitStructure);
  
  SYSCFG_EXTILineConfig(LIS_A_INT2_EXTI_PORT_SOURCE, LIS_A_INT2_EXTI_PIN_SOURCE);
 
  EXTI_InitStructure.EXTI_Line = LIS_A_INT2_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;    
  EXTI_Init(&EXTI_InitStructure);
  
  EXTI_ClearFlag(LIS_A_INT2_EXTI_LINE);
  
  NVIC_InitStructure.NVIC_IRQChannel = LIS_A_INT2_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
* @brief  Initializes the I2C peripheral used to drive the STLM75
* @param  None
* @retval None
*/

void LPS25H_I2C_Init(void)
{
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(LPS25H_RCC_Periph_I2C, ENABLE); 
  RCC_APB2PeriphClockCmd(LPS25H_RCC_Port_I2C, ENABLE);
 
  /* GPIO AF configuration */
  GPIO_PinAFConfig(LPS25H_I2C_Port, LPS25H_I2C_SCL_Pin_Source, LPS25H_I2C_SCL_AF);
  GPIO_PinAFConfig(LPS25H_I2C_Port, LPS25H_I2C_SDA_Pin_Source, LPS25H_I2C_SCL_AF);
  
  /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  LPS25H_I2C_SCL_Pin | LPS25H_I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(LPS25H_I2C_Port, &GPIO_InitStructure);
  
  /* I2C configuration */
  I2C_DeInit(LPS25H_I2C);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = LPS25H_I2C_Speed;
  
    /* Apply I2C configuration after enabling it */
  I2C_Init(LPS25H_I2C, &I2C_InitStructure);  
  
  /* I2C Peripheral Enable */
  I2C_Cmd(LPS25H_I2C, ENABLE);
  
}

void STLM75_I2C_DeInit_GPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin =  STLM75_I2C_SCL_Pin | STLM75_I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(STLM75_I2C_Port, &GPIO_InitStructure);  
}

void STLM75_I2C_Init_GPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  STLM75_I2C_SCL_Pin | STLM75_I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(STLM75_I2C_Port, &GPIO_InitStructure); 
}


/**
* @brief  Initializes the I2C peripheral used to drive the STLM75
* @param  None
* @retval None
*/

void STLM75_I2C_Init(void)
{
  I2C_InitTypeDef  I2C_InitStructure;

  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(STLM75_RCC_Periph_I2C, ENABLE); 
  RCC_APB2PeriphClockCmd(STLM75_RCC_Port_I2C, ENABLE);
 
  /* GPIO AF configuration */
  GPIO_PinAFConfig(STLM75_I2C_Port, STLM75_I2C_SCL_Pin_Source, STLM75_I2C_SCL_AF);
  GPIO_PinAFConfig(STLM75_I2C_Port, STLM75_I2C_SDA_Pin_Source, STLM75_I2C_SCL_AF);
  
  STLM75_I2C_Init_GPIO();
  
  /* I2C configuration */
  I2C_DeInit(STLM75_I2C);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = STLM75_I2C_Speed;
  
    /* Apply I2C configuration after enabling it */
  I2C_Init(STLM75_I2C, &I2C_InitStructure);
  
  /* I2C Peripheral Enable */
  I2C_Cmd(STLM75_I2C, ENABLE);
  
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    STM32L15_USB_CONNECT;
  }
  else
  {
    STM32L15_USB_DISCONNECT;
  }  
}

/*******************************************************************************
* Function Name  : Init_User_Timer
* Description    : Initialize a timer for application usage.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Init_User_Timer(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
	TIM_TimeBaseInitStruct.TIM_Prescaler = USER_TIMER_PRESCALER;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseInitStruct.TIM_Period = USER_TIMER_PERIOD;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
    
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : Start_User_Timer
* Description    : Start the user timer
* Input          : None.
* Return         : None.
*******************************************************************************/
void Start_User_Timer(void)
{
    TIM3->CNT = 0;
    TIM_Cmd(TIM3, ENABLE);
}

/*******************************************************************************
* Function Name  : Stop_User_Timer
* Description    : Stop the user timer
* Input          : None.
* Return         : None.
*******************************************************************************/
void Stop_User_Timer(void)
{
    TIM_Cmd(TIM3, DISABLE);    
}

/*******************************************************************************
* Function Name  : TIM3_IRQHandler
* Description    : User Timer ISR
* Input          : None.
* Return         : None.
*******************************************************************************/
void TIM3_IRQHandler(void)
{
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    user_timer_expired = TRUE;    
    
#if ENABLE_MICRO_SLEEP
    Exit_LP_Sleep_Mode();
#endif
}

// TODO: implement with clock-independent function.
void us150Delay()
{
#if SYSCLK_FREQ == 4000000
for(volatile int i = 0; i < 35; i++)__NOP();
#elif SYSCLK_FREQ == 32000000
for(volatile int i = 0; i < 420; i++)__NOP();
#else
#error Implement delay function.
#endif    
}

/*******************************************************************************
* Function Name  : Handle_USBAsynchXfer.
* Description    : send data to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
#ifdef ENABLE_USB
void Handle_USBAsynchXfer (void)
{
  
  uint16_t USB_Tx_ptr;
  uint16_t USB_Tx_length;
  
  if(USB_Tx_State != 1)
  {
    if (USART_Rx_ptr_out == USART_RX_DATA_SIZE)
    {
      USART_Rx_ptr_out = 0;
    }
    
    if(USART_Rx_ptr_out == USART_Rx_ptr_in) 
    {
      USB_Tx_State = 0; 
      return;
    }
    
    if(USART_Rx_ptr_out > USART_Rx_ptr_in) /* rollback */
    { 
      USART_Rx_length = USART_RX_DATA_SIZE - USART_Rx_ptr_out;
    }
    else 
    {
      USART_Rx_length = USART_Rx_ptr_in - USART_Rx_ptr_out;
    }
    
    if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE)
    {
      USB_Tx_ptr = USART_Rx_ptr_out;
      USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
      
      USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;	
      USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;	
    }
    else
    {
      USB_Tx_ptr = USART_Rx_ptr_out;
      USB_Tx_length = USART_Rx_length;
      
      USART_Rx_ptr_out += USART_Rx_length;
      USART_Rx_length = 0;
    }
    USB_Tx_State = 1; 
    
#ifdef USE_STM3210C_EVAL
    USB_SIL_Write(EP1_IN, &USART_Rx_Buffer[USB_Tx_ptr], USB_Tx_length);  
#else
    UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
    SetEPTxCount(ENDP1, USB_Tx_length);
    SetEPTxValid(ENDP1); 
#endif /* USE_STM3210C_EVAL */
  }  
  
}
#endif
/*******************************************************************************
* Function Name  : USB_Send_Data.
* Description    : Insert a byte to USB buffer for transmission.
* Input          : None.
* Return         : none.
*******************************************************************************/
#ifdef ENABLE_USB
void USB_Send_Data(uint8_t byte)
{
  
  USART_Rx_Buffer[USART_Rx_ptr_in] = byte;
  
  USART_Rx_ptr_in++;
  
  /* To avoid buffer overflow */
  if(USART_Rx_ptr_in == USART_RX_DATA_SIZE)
  {
    USART_Rx_ptr_in = 0;
  }
}
#endif

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(uint32_t*)(0x1FF80050);
  Device_Serial1 = *(uint32_t*)(0x1FF80054);
  Device_Serial2 = *(uint32_t*)(0x1FF80064);

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}

#ifdef ENABLE_USB_PRINTF
#undef putchar

int putchar(int c)
{
  int i = 0xFFFF;/* Buffer full: wait till previous data is transmitted */
  while(USB_OUT_BUFFER_IS_FULL() && i-- > 0);
  if(i==0){
      const char *str = "\nOVERFLOW\n";
      while(*str != '\0')
          USB_Send_Data(*str++);
  }
  
  USB_Send_Data(c);
  return c;
}

void __io_putchar(char c)
{
  putchar(c);
}

size_t _write(int handle, const unsigned char * buffer, size_t size)
{
  size_t nChars = 0;

  if (handle != _LLIO_STDOUT && handle != _LLIO_STDERR) {
    return _LLIO_ERROR;
  }

  if (buffer == 0) {
    // This means that we should flush internal buffers.
    //spin until TX complete (TX is idle)
    while (!USB_OUT_BUFFER_IS_EMPTY()) {}
    return 0;
  }

  while(size--) {
    __io_putchar(*buffer++);
    ++nChars;
  }

  return nChars;
}


size_t _read(int handle, unsigned char * buffer, size_t size)
{
  return 0;
}

#endif

int USB_free_buffer_size()
{
  int free_space;
  
  if(USART_Rx_ptr_in == USART_Rx_ptr_out){
    free_space = USART_RX_DATA_SIZE;
  }
  else if(USART_Rx_ptr_out > USART_Rx_ptr_in) /* rollback */
  { 
    free_space = USART_Rx_ptr_out - USART_Rx_ptr_in;
  }
  else 
  {
    free_space = USART_RX_DATA_SIZE - USART_Rx_ptr_in + USART_Rx_ptr_out;
  }
  
  return free_space;
}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

