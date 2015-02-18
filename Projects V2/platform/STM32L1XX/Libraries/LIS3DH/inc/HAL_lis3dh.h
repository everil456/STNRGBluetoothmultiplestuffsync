/**
  * @file    HAL_LIS331DLH.h
  * @author  ART Team IMS-Systems Lab
  * @version V2.2
  * @date    01/11/2011
  * @brief   Hardware Abstraction Layer for LIS331DLH.
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
#ifndef __HAL_LIS331DLH_H
#define __HAL_LIS331DLH_H

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes */
#include "stm32l1xx.h"

/**
* @addtogroup LIS331DLH
* @{
*/
   
/**
* @addtogroup  LIS331DLH_SPI_Define
* @{
*/
#define LIS_SPI                  SPI1
#define LIS_RCC_Periph_SPI       RCC_APB2Periph_SPI1
#define LIS_SPI_Port             GPIOA
#define LIS_SPI_AF               GPIO_AF_SPI1
#define LIS_SPI_MISO_Pin         GPIO_Pin_6
#define LIS_SPI_MISO_SOURCE      GPIO_PinSource6
#define LIS_SPI_M0SI_Pin         GPIO_Pin_7
#define LIS_SPI_MOSI_SOURCE      GPIO_PinSource7
#define LIS_SPI_SCK_Pin          GPIO_Pin_5
#define LIS_SPI_SCK_SOURCE       GPIO_PinSource5
#define LIS_SPI_CS_Pin           GPIO_Pin_4
#define LIS_SPI_CS_Port          GPIOA
#define LIS_RCC_Port_SPI         RCC_AHBPeriph_GPIOA

 /**
*@}
*/ /* end of group LIS331DLH_SPI_Define */ 



/**
* @addtogroup Accelerometer_Interrupt_Pin_Define
* @{
*/
#define LIS_A_INT2_PIN           	GPIO_Pin_11
#define LIS_A_INT2_GPIO_PORT     	GPIOB
#define LIS_A_INT2_GPIO_CLK      	RCC_AHBPeriph_GPIOB
#define LIS_A_INT2_EXTI_LINE     	EXTI_Line11
#define LIS_A_INT2_EXTI_PIN_SOURCE 	EXTI_PinSource11
#define LIS_A_INT2_EXTI_PORT_SOURCE EXTI_PortSourceGPIOB
#define LIS_A_INT2_EXTI_IRQn		EXTI15_10_IRQn
#define LIS3DH_IRQHandler           EXTI15_10_IRQHandler


/**
*@}
*/ /* end of group Accelerometer_Interrupt_Pin_Define */


/**
*@}
*/ /* end of group LIS331DLH */



#endif /* __HAL_LIS331DLH_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
