/**
  * @file    i2c.c
  * @author  AMS - HESA division
  * @version V1.0
  * @date    07/10/2013
  * @brief   This file provides a set of functions needed to manage the
  *          communication between I2C master and I2C slave.
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
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  */
      
#include <i2c.h>
#include <gp_timer.h>

/**
* @brief  Writes one or more byte to the device registers.
* @param  pBuffer : pointer to the buffer  containing the data to be 
*                    written into device.
* @param  WriteAddr : device's internal address to write to.
* @param  NumByteToWrite: Number of bytes to write
* @retval 1 if success, 0 if error.
*/

int I2C_Write(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
{
  struct timer t;
            
  Timer_Set(&t, CLOCK_SECOND/10);
  
  /* Added missing check: MSB enables address auto increment */
  if(NumByteToWrite>1)
    WriteAddr |= 0x80;
  
  /* Send START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
    if(Timer_Expired(&t))
      return 0;
  }  
  
  /* Send STLM75 address for write */
  I2C_Send7bitAddress(I2Cx, deviceAddress, I2C_Direction_Transmitter);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
    if(Timer_Expired(&t))
      return 0;
  }
  
  /* Send the STLM75's internal address to write to */
  I2C_SendData(I2Cx, WriteAddr);
  
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
    if(Timer_Expired(&t))
      return 0;
  }

  
  while(NumByteToWrite)
  {
    /* Send the byte to be written */
    I2C_SendData(I2Cx, *pBuffer);
    
    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
      if(Timer_Expired(&t))
        return 0;
    }
    
    pBuffer++;
    NumByteToWrite--;
  }
  
  /* Send STOP condition */
  I2C_GenerateSTOP(I2Cx, ENABLE); 
  
  return 1;
  
}

/**
* @brief  Read one or more byte from the device registers.
* @param  pBuffer : pointer to the buffer that receives the data read 
*                    from the device.
* @param  ReadAddr : device's internal address to read from.
* @param  NumByteToRead:number of bytes to read from the device.
* @retval 1 if success, 0 otherwise.
*/

int I2C_Read(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t* pBuffer, uint8_t ReadAddr, uint8_t NumByteToRead)
{
  struct timer t;
            
  Timer_Set(&t, CLOCK_SECOND/10);
    
  /* MSB enables address auto increment */
  if(NumByteToRead>1)
    ReadAddr |= 0x80;
  
    /* While the bus is busy */
  while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)){
    if(Timer_Expired(&t))
      return 0;
  }
  
  /* Send START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
    if(Timer_Expired(&t))
      return 0;
  }
   
  /* Send STLM75 address for write */
  I2C_Send7bitAddress(I2Cx, deviceAddress, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
    if(Timer_Expired(&t))
      return 0;
  }
  
  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(I2Cx, ENABLE); 
  
  /* Send the STLM75's internal address to write to */
  I2C_SendData(I2Cx, ReadAddr);  

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
    if(Timer_Expired(&t))
      return 0;
  }
  
  /* Send START condition a second time */  
  I2C_GenerateSTART(I2Cx, ENABLE);
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
    if(Timer_Expired(&t))
      return 0;
  }
  
  if(NumByteToRead == 1)
  {	/* Disable Acknowledgement */
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
  }
  else {
    I2C_AcknowledgeConfig(I2Cx, ENABLE); //I2C2
  }
  
  /* Send STLM75 address for read */
  I2C_Send7bitAddress(I2Cx, deviceAddress, I2C_Direction_Receiver);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
    if(Timer_Expired(&t))
      return 0;
  }
  
  /* While there is data to be read */
  while(NumByteToRead)  
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2Cx, DISABLE);
      /* Send STOP Condition */
      I2C_GenerateSTOP(I2Cx, ENABLE);
    }
    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))  
    {      
      /* Read a byte from the STLM75 */
      *pBuffer = I2C_ReceiveData(I2Cx);

      /* Point to the next location where the byte read will be saved */
      pBuffer++; 
      
      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
    
    if(Timer_Expired(&t))
      return 0;
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2Cx, ENABLE);  
  
  return 1;
}

/**
 * @}
 */ /* end of group STLM75_API */

/**
*@}
*/ /* end of group STLM75 */



/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
