/**
  * @file    HTS221.h
  * @author  AMS - HESA (High End Sensors & Analog division)
  * @version V1.0
  * @date    07/05/2013
  * @brief   Header for HTS221.c file
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



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HTS221_H
#define __HTS221_H

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes ------------------------------------------------------------------*/
#include "HAL_HTS221.h"

/**
* @addtogroup HTS221
* @{
*/

/* Exported types ------------------------------------------------------------*/

/** @defgroup HTS221_Register_Mapping
  * @{
  */

/** 
* \brief brief HTS221 I2C Slave Address 
*/

/* Exported constants --------------------------------------------------------*/
#define HTS221_ADDRESS         0xBE


#define HTS221_WHO_AM_I        0x0F
#define HTS221_RES_CONF        0x10
#define HTS221_INFO_L          0x1E
#define HTS221_INFO_H          0x1F
#define HTS221_CTRL_REG1       0x20
#define HTS221_CTRL_REG2       0x21
#define HTS221_STATUS_REG      0x27
#define HTS221_HUMIDITY_OUT_L  0x28
#define HTS221_HUMIDITY_OUT_H  0x29
#define HTS221_TEMP_OUT_L      0x2A
#define HTS221_TEMP_OUT_H      0x2B
#define HTS221_CALIB_0         0x30
#define HTS221_CALIB_1         0x31
#define HTS221_CALIB_2         0x32
#define HTS221_CALIB_3         0x33
#define HTS221_CALIB_4         0x34
#define HTS221_CALIB_5         0x35
#define HTS221_CALIB_6         0x36
#define HTS221_CALIB_7         0x37
#define HTS221_CALIB_8         0x38
#define HTS221_CALIB_9         0x39
#define HTS221_CALIB_A         0x3A
#define HTS221_CALIB_B         0x3B
#define HTS221_CALIB_C         0x3C
#define HTS221_CALIB_D         0x3D
#define HTS221_CALIB_E         0x3E
#define HTS221_CALIB_F         0x3F

#define HTS221_H0_RH_x2        0x30
#define HTS221_H1_RH_x2        0x31
#define HTS221_T0_degC_x8      0x32
#define HTS221_T1_degC_x8      0x33
#define HTS221_CALIB_4         0x34
#define HTS221_CALIB_5         0x35
#define HTS221_H0_T0_OUT_L     0x36
#define HTS221_H0_T0_OUT_H     0x37
#define HTS221_CALIB_8         0x38
#define HTS221_CALIB_9         0x39
#define HTS221_H1_T0_OUT_L     0x3A
#define HTS221_H1_T0_OUT_H     0x3B
#define HTS221_T0_OUT_L        0x3C
#define HTS221_T0_OUT_H        0x3D
#define HTS221_T1_OUT_L        0x3E
#define HTS221_T1_OUT_H        0x3F

/**
 * @}
 */ /* End of HTS221_Register_Mapping */


/**
  * @defgroup HTS221_Configuration_Defines
  *@{
  */


void HTS221_I2C_Init(void);
void HTS221_Power_On(void);
int HTS221_Read_Temperature(int16_t* temp);
int HTS221_Read_Humidity(uint16_t* humidity);


/**
 * @}
 */ /* End of STLM75 */

#endif /* __STLM75_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
