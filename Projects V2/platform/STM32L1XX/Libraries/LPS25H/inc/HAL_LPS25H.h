/**
  * @file    HAL_LPS25H.h
  * @author  AMS - HESA (High End Sensors & Analog division)
  * @version V1.0
  * @date    07/05/2013
  * @brief   Hardware Abstraction Layer for LPS25H.
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



/* Define to prevent recursive inclusion*/
#ifndef __HAL_LPS25H_H
#define __HAL_LPS25H_H

#ifdef __cplusplus
 extern "C" {
#endif 
  

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "i2c.h"

/**
* @defgroup LPS25H
* @{
*/


/**
* @defgroup  LPS25H_I2C_Define
* @{
*/
     
#define LPS25H_ADDRESS         0xB8

#define LPS25H_I2C                  I2C1	
#define LPS25H_RCC_Periph_I2C       RCC_APB1Periph_I2C1
#define LPS25H_I2C_SCL_AF           GPIO_AF_I2C1
#define LPS25H_I2C_Port             GPIOB
#define LPS25H_I2C_SCL_Pin          GPIO_Pin_8
#define LPS25H_I2C_SCL_Pin_Source   GPIO_PinSource8
#define LPS25H_I2C_SDA_Pin          GPIO_Pin_9
#define LPS25H_I2C_SDA_Pin_Source   GPIO_PinSource9
#define LPS25H_RCC_Port_I2C         RCC_AHBPeriph_GPIOB

#define LPS25H_I2C_Speed            10000
     
#define LPS25H_Write(pBuffer, WriteAddr, NumByteToWrite) \
        I2C_Write(LPS25H_I2C, LPS25H_ADDRESS, pBuffer, WriteAddr, NumByteToWrite)
            
#define LPS25H_Read(pBuffer, ReadAddr, NumByteToRead) \
        I2C_Read(LPS25H_I2C, LPS25H_ADDRESS, pBuffer, ReadAddr, NumByteToRead)

/**
*@}
*/ /* end of group LPS25H_I2C_Define */ 

/**
*@}
*/ /* end of group LPS25H */


#endif /* __HAL_LPS25H_H */

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
