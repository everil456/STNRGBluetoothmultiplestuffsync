/**
 * @file    USER_Platform_Configuration_STEVAL_IDB002V1.h
 * @author  AMS - AAS Division
 * @version V1.0.0
 * @date    March, 17 2014
 * @brief   This file contains definitions for BlueNRG Eval Platform 
 * @details Only led DL1,DL2 & Push, Joystick SEL Button are mapped
 *
 * In this module there the define values for the configuration  management 
 * of the BlueNRG user platform.
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
 * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_PLATFORM_CONFIGURATION_H
#define __USER_PLATFORM_CONFIGURATION_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* DFU is enabled on BlueNRG Eval Board (STEVAL-IDB002V1) */ 
#define VECTOR_TABLE_BASE_DFU_OFFSET (0x3000)
   
/**
* @brief BlueNRG User Platform SPI & GPIO lines: to be customized for addressing user platform.
*        Example configuration: BlueNRG Eval Board (STEVAL-IDB002V1).
 */
/* BlueNRG SPI Port */
#define SPI                           SPI2
/* BlueNRG SPI Clock */
#define SPI_CLK_APB1                  RCC_APB1Periph_SPI2
/* BlueNRG SPI SCLK define values */
#define SPI_SCLK_GPIO_PIN             GPIO_Pin_13                 /* PB.13 */
#define SPI_SCLK_GPIO_PORT            GPIOB                       /* GPIOB */
#define SPI_SCLK_GPIO_CLK             RCC_AHBPeriph_GPIOB
#define SPI_SCLK_GPIO_SOURCE          GPIO_PinSource13
#define SPI_SCLK_GPIO_AF              GPIO_AF_SPI2
/* Defines for MISO pin */
#define SPI_MISO_GPIO_PIN             GPIO_Pin_14                 /* PB.14 */
#define SPI_MISO_GPIO_PORT            GPIOB                       /* GPIOB */
#define SPI_MISO_GPIO_CLK             RCC_AHBPeriph_GPIOB
#define SPI_MISO_GPIO_SOURCE          GPIO_PinSource14
#define SPI_MISO_GPIO_AF              GPIO_AF_SPI2
/* Defines for MOSI pin */
#define SPI_MOSI_GPIO_PIN             GPIO_Pin_15                 /* PB.15 */
#define SPI_MOSI_GPIO_PORT            GPIOB                       /* GPIOB */
#define SPI_MOSI_GPIO_CLK             RCC_AHBPeriph_GPIOB
#define SPI_MOSI_GPIO_SOURCE          GPIO_PinSource15
#define SPI_MOSI_GPIO_AF              GPIO_AF_SPI2
/* BlueNRG SPI CS define values */
#define SPI_CS_GPIO_PIN               GPIO_Pin_12                 /* PB.12 */
#define SPI_CS_GPIO_PORT              GPIOB                       /* GPIOB */
#define SPI_CS_GPIO_CLK               RCC_AHBPeriph_GPIOB
/* BlueNRG SPI IRQ line define values */
#define SPI_IRQ_GPIO_PIN              GPIO_Pin_10                  /* PC.10 */
#define SPI_IRQ_EXTI_LINE             EXTI_Line10
#define SPI_IRQ_EXTI_PIN_SOURCE       EXTI_PinSource10
#define SPI_IRQ_GPIO_PORT             GPIOC                       /* GPIOC */
#define SPI_IRQ_GPIO_CLK              RCC_AHBPeriph_GPIOC
#define SPI_IRQ_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOC
#define SPI_IRQ_EXTI_IRQn             EXTI15_10_IRQn
#define SPI_IRQ_IRQHandler            EXTI15_10_IRQHandler

/* BlueNRG SW reset line define values */
#define SW_RST_GPIO_PIN               GPIO_Pin_13      /* PC.13 */
#define SW_RST_GPIO_PORT              GPIOC
#define SW_RST_GPIO_CLK               RCC_AHBPeriph_GPIOC

/**
* @brief BlueNRG Eval Board (STEVAL-IDB002V1) & SPI and GPIO lines x EEPROM
 */
#define EEPROM_SPI_PERIPH_NB                  SPI2
#define EEPROM_SPI_PERIPH_RCC_APB1            RCC_APB1Periph_SPI2
  
/* Defines for MOSI pin */
#define EEPROM_SPI_PERIPH_MOSI_PORT           GPIOB
#define EEPROM_SPI_PERIPH_MOSI_PIN            GPIO_Pin_15
#define EEPROM_SPI_PERIPH_MOSI_AF             GPIO_AF_SPI2
#define EEPROM_SPI_PERIPH_MOSI_RCC            RCC_AHBPeriph_GPIOB
#define EEPROM_SPI_PERIPH_MOSI_RCC_SOURCE     GPIO_PinSource15

/* Defines for MISO pin */
#define EEPROM_SPI_PERIPH_MISO_PORT           GPIOB
#define EEPROM_SPI_PERIPH_MISO_PIN            GPIO_Pin_14
#define EEPROM_SPI_PERIPH_MISO_AF             GPIO_AF_SPI2
#define EEPROM_SPI_PERIPH_MISO_RCC            RCC_AHBPeriph_GPIOB
#define EEPROM_SPI_PERIPH_MISO_RCC_SOURCE     GPIO_PinSource14

/* Defines for SCLK pin */
#define EEPROM_SPI_PERIPH_SCLK_PORT           GPIOB
#define EEPROM_SPI_PERIPH_SCLK_PIN            GPIO_Pin_13
#define EEPROM_SPI_PERIPH_SCLK_AF             GPIO_AF_SPI2
#define EEPROM_SPI_PERIPH_SCLK_RCC            RCC_AHBPeriph_GPIOB
#define EEPROM_SPI_PERIPH_SCLK_RCC_SOURCE     GPIO_PinSource13

/* Defines for chip select pin x BlueNRG Eval Board (STEVAL-IDB002V1) */ 
#define EEPROM_SPI_PERIPH_CS_PORT             GPIOC
#define EEPROM_SPI_PERIPH_CS_PIN              GPIO_Pin_12
#define EEPROM_SPI_PERIPH_CS_RCC              RCC_AHBPeriph_GPIOC

/**
 *  BlueNRG Eval Board (STEVAL-IDB002V1) DL1, DL2 leds
 */
#define USER_LED1_GPIO_PIN                    GPIO_Pin_0
#define USER_LED1_GPIO_PORT                   GPIOC
#define USER_LED1_GPIO_CLK                    RCC_AHBPeriph_GPIOC

#define USER_LED2_GPIO_PIN                    GPIO_Pin_1
#define USER_LED2_GPIO_PORT                   GPIOC
#define USER_LED2_GPIO_CLK                    RCC_AHBPeriph_GPIOC
   
/**
 * @brief BlueNRG Eval Board (STEVAL-IDB002V1) User Button
 *        IRQ handler:  EXTI9_5_IRQHandler()
 */
#define	USER_BUTTON1_GPIO_PIN	              GPIO_Pin_6
#define	USER_BUTTON1_GPIO_PORT	              GPIOC
#define	USER_BUTTON1_GPIO_CLK	              RCC_AHBPeriph_GPIOC
#define	USER_BUTTON1_EXTI_LINE	              EXTI_Line6
#define	USER_BUTTON1_EXTI_PORT_SOURCE         EXTI_PortSourceGPIOC
#define	USER_BUTTON1_EXTI_PIN_SOURCE          EXTI_PinSource6
#define USER_BUTTON1_EXTI_IRQn                EXTI9_5_IRQn

/**
 * @brief BlueNRG Eval Board (STEVAL-IDB002V1) Joystick Sel (Center) push-button
 *       IRQ handler:  EXTI9_5_IRQHandler()
 */
#define USER_BUTTON2_GPIO_PIN                 GPIO_Pin_9
#define USER_BUTTON2_GPIO_PORT                GPIOA
#define USER_BUTTON2_GPIO_CLK                 RCC_AHBPeriph_GPIOA
#define USER_BUTTON2_EXTI_LINE                EXTI_Line9
#define USER_BUTTON2_EXTI_PORT_SOURCE         EXTI_PortSourceGPIOA
#define USER_BUTTON2_EXTI_PIN_SOURCE          EXTI_PinSource9
#define USER_BUTTON2_EXTI_IRQn                EXTI9_5_IRQn 
   
   
#ifdef __cplusplus
}
#endif

#endif /* __USER_PLATFORM_CONFIGURATION_H */

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
