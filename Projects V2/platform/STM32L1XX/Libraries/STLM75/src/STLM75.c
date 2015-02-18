/**
  * @file    STLM75.c
  * @author  ART Team IMS-Systems Lab
  * @version V2.2
  * @date    01/11/2011
  * @brief   This file provides a set of functions needed to manage the
  *          communication between STM32 I2C master and STLM75 I2C slave.
  * @details
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
  * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
  * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */




/* Includes ------------------------------------------------------------------*/
#include "STLM75.h"
#include "HAL_STLM75.h"


/**
* @addtogroup STLM75
* @{
*/

/**
* @defgroup STLM75_API
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
* @brief  Initializes the OS/INT pin interrupt
* @param  None
* @retval None
*/

#ifdef OS_INT_ENABLE

void STLM75_INT_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable INT pad GPIO clocks */
  RCC_APB2PeriphClockCmd(STLM75_INT_RCC, ENABLE);
 
  /* Configure OS/INT pin as input floating */
  GPIO_InitStructure.GPIO_Pin = STLM75_INT_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;
  GPIO_Init(STLM75_INT_Port, &GPIO_InitStructure);
   
  /* Connect STLM75_INT_EXTI_Line to STLM75_INT_Pin */
  GPIO_EXTILineConfig(STLM75_INT_Port_Source, STLM75_INT_Pin_Source);

  /* Configure STLM75_INT_EXTI_Line to generate an interrupt on STLM75_INT_Edge edge */  
  EXTI_InitStructure.EXTI_Line = STLM75_INT_EXTI_Line;  
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = STLM75_INT_Edge;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable the STLM75_INT_EXTI_IRQCHANNEL Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = STLM75_INT_EXTI_IRQCHANNEL;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = STLM75_INT_Preemption_Priority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = STLM75_INT_Sub_Priority;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
#endif


/**
* @brief  Set interrupt configuration of STLM75
* @param  STLM75_Interrupt_Struct: pointer to a STLM75_InterruptTypeDef structure that
*         contains the interrupt configuration setting for the STLM75.
* @retval None
*/

#ifdef OS_INT_ENABLE

void STLM75_Interrupt_Config(STLM75_InterruptTypeDef *STLM75_Interrupt_Struct)
{
  uint8_t tmpreg1 = 0x00;
  
  tmpreg1 |= (uint8_t)(STLM75_Interrupt_Struct->Mode | STLM75_Interrupt_Struct->Polarity |
                  ((STLM75_Interrupt_Struct->Fault_Number)<<3));
  
  STLM75_Write(&tmpreg1, CONF_REG_ADDR, 1);
  STLM75_Set_Over_Limit(STLM75_Interrupt_Struct->Over_Limit_Value);
  STLM75_Set_Hysteresis(STLM75_Interrupt_Struct->Hysteresis_Value);
  
}
#endif

/**
* @brief  Read temperature data registers
* @param  pBuffer : pointer to the buffer that receives the temperature
*                   data read from the STLM75. It will give in pBuffer the TMSB byte
*                   and in pBuffer++ the TLSB byte. Temperature data are expressed as
*                   2's complement numbers.
* @retval None
*/


void STLM75_Read_Temperature_Regs(uint8_t* pBuffer)
{

  uint8_t temp[2]={0,0};
  STLM75_Read(&temp[0], STLM75_TEMP_REG_ADDR, 2);
  *pBuffer=temp[0];  
  pBuffer++;
  *pBuffer=temp[1];
  
}


/**
* @brief  Read temperature registers and put in a signed 16-bit variable
* @param  out : variable where put the raw data
* @retval None
*/

void STLM75_Read_Raw_Data(signed short* out)
{

  uint8_t temp[2]={0,0};
  STLM75_Read(&temp[0], STLM75_TEMP_REG_ADDR, 2);
  *out = (signed short)(((unsigned short)temp[0] << 8) + temp[1]);
  
}


/**
* @brief  Read temperature data
* @param  refvalue : Temperature data expressed as
*                    2's complement numbers and in tenth of °C
* @retval 1 if success, 0 if not.
*/

int STLM75_Read_Temperature_Signed(signed short* refvalue)
{
  uint8_t tmp = 0;
  uint8_t temp[2]={0,0};
  signed short value = 0x00;
  if(!STLM75_Read(&temp[0], STLM75_TEMP_REG_ADDR, 2)){
      return 0;
  }
  tmp = (temp[1]>>7) + (temp[0]<<1);
  value = (signed short)((((unsigned short)(temp[0]&0x80))<<8) + (unsigned short)(tmp));
  *refvalue = (signed short)(value*T_Resolution);  // expressed in decimal of °C
     
  return 1;
}

/**
* @brief  Enables or disables the lowpower mode for STLM75
* @param  refvalue : NewState: new state for the lowpower mode.
*         This parameter can be one of following parameters:
*         @arg ENABLE
*         @arg DISABLE
* @retval None
*/

void STLM75_Lowpower_Cmd(FunctionalState NewState)
{
  uint8_t tmpreg = 0x00;
  STLM75_Read(&tmpreg, STLM75_CONF_REG_ADDR, 1);
  if(NewState==ENABLE)
    tmpreg |= 0x01;
  else
    tmpreg &= 0xF7;
  STLM75_Write(&tmpreg, STLM75_CONF_REG_ADDR, 1);
}

/**
* @brief  Set temperature over-limit reference
* @param  refvalue : temperature threshold value expressed in tenth of °C
*                    with step of 5 tenth of °C
* @retval None
*/

void STLM75_Set_Over_Limit(signed short refvalue)
{
  signed short temp;
  uint8_t tempbuffer[2];
  uint8_t tmp1, tmp2;
  temp = (signed short) (refvalue/T_Resolution);
  tmp1 = (uint8_t)(temp>>8);
  tmp2 = (uint8_t)(temp);
  tempbuffer[0] =tmp1 + (tmp2>>1);
  tempbuffer[1]= tmp2<<7;
  STLM75_Write(&tempbuffer[0], STLM75_TOS_REG_ADDR, 2);
  
}

/**
* @brief  Set Hysteresis threshold
* @param  refvalue : Hysteresis threshold value expressed in tenth of °C
*                    with step of 5 tenth of °C
* @retval None
*/

void STLM75_Set_Hysteresis(signed short refvalue)
{
  signed short temp;
  uint8_t tempbuffer[2];
  uint8_t tmp1, tmp2;
  temp = (signed short) (refvalue/T_Resolution);
  tmp1 = (uint8_t)(temp>>8);
  tmp2 = (uint8_t)(temp);
  tempbuffer[0] =tmp1 + (tmp2>>1);
  tempbuffer[1]= tmp2<<7;
  STLM75_Write(&tempbuffer[0], STLM75_THYS_REG_ADDR, 2);
  
}

/**
* @brief  Set the command/Pointer register
* @param  eRegPointer : pointer value to write in the Command/Pointer
*                       register the buffer that receives the data read
*                       from the STLM75.
* @retval None
*/

void STLM75_SetPointer(uint8_t eRegPointer)
{  
  I2C_GenerateSTART(STLM75_I2C, ENABLE);
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(STLM75_I2C, I2C_EVENT_MASTER_MODE_SELECT));
  
  /* Send STLM75 address for write */
  I2C_Send7bitAddress(STLM75_I2C, STLM75_ADDRESS, I2C_Direction_Transmitter);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(STLM75_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
  /* Set Pointer Byte */
  I2C_SendData(STLM75_I2C, eRegPointer);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(STLM75_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  I2C_GenerateSTOP(STLM75_I2C, ENABLE);  
}

/**
 * @}
 */ /* end of group STLM75_API */

/**
*@}
*/ /* end of group STLM75 */



/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
