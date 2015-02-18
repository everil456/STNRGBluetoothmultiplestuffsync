/**
  * @file    i2c.h
  * @author  AMS - HESA division
  * @version V1.0
  * @date    07/10/2013
  * @brief   Hardware Abstraction Layer for I2C.
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
#ifndef __I2C_H
#define __I2C_H

#include <stm32l1xx_i2c.h>

int I2C_Write(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite);

int I2C_Read(I2C_TypeDef* I2Cx, uint8_t deviceAddress, uint8_t* pBuffer, uint8_t ReadAddr, uint8_t NumByteToRead);


#endif /* __I2C_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
