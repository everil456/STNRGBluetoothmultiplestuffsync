/**
  * @file    HAL_STLM75.h
  * @author  ART Team IMS-Systems Lab
  * @version V2.2
  * @date    01/11/2011
  * @brief   Hardware Abstraction Layer for STLM75.
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



/* Define to prevent recursive inclusion*/
#ifndef __HAL_STLM75_H
#define __HAL_STLM75_H

#ifdef __cplusplus
 extern "C" {
#endif 
  

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "i2c.h"

/**
* @defgroup STLM75
* @{
*/


/**
* @defgroup  STLM75_I2C_Define
* @{
*/

#define STLM75_I2C                  I2C1	
#define STLM75_RCC_Periph_I2C       RCC_APB1Periph_I2C1
#define STLM75_I2C_SCL_AF           GPIO_AF_I2C1
#define STLM75_I2C_Port             GPIOB
#define STLM75_I2C_SCL_Pin          GPIO_Pin_6
#define STLM75_I2C_SCL_Pin_Source   GPIO_PinSource6
#define STLM75_I2C_SDA_Pin          GPIO_Pin_7
#define STLM75_I2C_SDA_Pin_Source   GPIO_PinSource7
#define STLM75_RCC_Port_I2C         RCC_AHBPeriph_GPIOB

#define STLM75_I2C_Speed            10000

#define STLM75_Write(pBuffer, WriteAddr, NumByteToWrite) \
        I2C_Write(STLM75_I2C, STLM75_ADDRESS, pBuffer, WriteAddr, NumByteToWrite)
            
#define STLM75_Read(pBuffer, ReadAddr, NumByteToRead) \
        I2C_Read(STLM75_I2C, STLM75_ADDRESS, pBuffer, ReadAddr, NumByteToRead)
     
/**
*@}
*/ /* end of group STLM75_I2C_Define */ 


/**
* @defgroup STLM75_Interrupt_Pin_Define
* @{
*/

#ifdef OS_INT_ENABLE
#define STLM75_INT_Port                  GPIOB
#define STLM75_INT_RCC                   RCC_APB2Periph_GPIOB
#define STLM75_INT_Pin                   GPIO_Pin_5
#define STLM75_INT_Port_Source           GPIO_PortSourceGPIOB
#define STLM75_INT_Pin_Source            GPIO_PinSource5
#define STLM75_INT_EXTI_Line             EXTI_Line5
#define STLM75_INT_Edge                  EXTI_Trigger_Falling 
#define STLM75_INT_EXTI_IRQCHANNEL       EXTI9_5_IRQn
#define STLM75_INT_Preemption_Priority   12
#define STLM75_INT_Sub_Priority          0
#endif //OS_INT_ENABLE

/**
*@}
*/ /* end of group STLM75_Interrupt_Pin_Define */

/**
*@}
*/ /* end of group STLM75 */


#endif /* __HAL_STLM75_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
