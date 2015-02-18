/**
  * @file    STLM75.c
  * @author  ART Team IMS-Systems Lab
  * @version V2.2
  * @date    01/11/2011
  * @brief   This file provides a set of functions needed to manage the
  *          communication between STM32 I2C master and HTS221 I2C slave.
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
#include "HTS221.h"
#include "HAL_HTS221.h"


/**
* @addtogroup HTS221
* @{
*/

/**
* @defgroup HTS221_API
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void HTS221_I2C_DeInit_GPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin =  HTS221_I2C_SCL_Pin | HTS221_I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(HTS221_I2C_Port, &GPIO_InitStructure);  
}

void HTS221_I2C_Init_GPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
    /* Configure I2C pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  HTS221_I2C_SCL_Pin | HTS221_I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(HTS221_I2C_Port, &GPIO_InitStructure); 
}

/**
* @brief  Initializes the I2C peripheral used to drive the STLM75
* @param  None
* @retval None
*/

void HTS221_I2C_Init(void)
{
  I2C_InitTypeDef  I2C_InitStructure;

  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(HTS221_RCC_Periph_I2C, ENABLE); 
  RCC_APB2PeriphClockCmd(HTS221_RCC_Port_I2C, ENABLE);
 
  /* GPIO AF configuration */
  GPIO_PinAFConfig(HTS221_I2C_Port, HTS221_I2C_SCL_Pin_Source, HTS221_I2C_SCL_AF);
  GPIO_PinAFConfig(HTS221_I2C_Port, HTS221_I2C_SDA_Pin_Source, HTS221_I2C_SCL_AF);
  
  /* Configure I2C pins: SCL and SDA */
  HTS221_I2C_Init_GPIO();
  
  /* I2C configuration */
  I2C_DeInit(HTS221_I2C);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = HTS221_I2C_Speed;
  
    /* Apply I2C configuration after enabling it */
  I2C_Init(HTS221_I2C, &I2C_InitStructure);  
  
  /* I2C Peripheral Enable */
  I2C_Cmd(HTS221_I2C, ENABLE);
  
}

void HTS221_Power_On(void)
{
    uint8_t reg = 0x80;
    HTS221_Write(&reg, HTS221_CTRL_REG1, 1);
}

int HTS221_Start_Measure(void)
{
    uint8_t reg = 0x01;
        
    if(!HTS221_Write(&reg, HTS221_CTRL_REG2, 1))
        return 0;
    
    reg = 0;
    
    while( (reg & 0x03) != 0x03 ){
        if(!HTS221_Read(&reg, HTS221_STATUS_REG, 1))
            return 0;        
    }
    return 1;
}

int HTS221_Read_Temperature(int16_t* temp)
{
    uint8_t T0_degC_x8, T1_degC_x8;
    int16_t T0_out, T1_out, T_out;
    float T0_degC, T1_degC, T_degC;    
    //int ret;
    
    if(!HTS221_Start_Measure())
        return 0;
    
    if(!HTS221_Read(&T0_degC_x8, HTS221_T0_degC_x8, 1))
       return 0;
    if(!HTS221_Read(&T1_degC_x8, HTS221_T1_degC_x8, 1))
       return 0;
//    ret = HTS221_Read((uint8_t*)&T0_out, HTS221_T0_OUT_L, 2);    
//    ret = HTS221_Read((uint8_t*)&T1_out, HTS221_T1_OUT_L, 2);
//    ret = HTS221_Read((uint8_t*)&T_OUT, HTS221_TEMP_OUT_L, 2);
    
    if(!HTS221_Read((uint8_t*)&T0_out, HTS221_T0_OUT_L, 1))
       return 0;
    if(!HTS221_Read((uint8_t*)&T0_out + 1, HTS221_T0_OUT_H, 1))
       return 0;
    if(!HTS221_Read((uint8_t*)&T1_out, HTS221_T1_OUT_L, 1))
       return 0;
    if(!HTS221_Read((uint8_t*)&T1_out + 1, HTS221_T1_OUT_H, 1))
       return 0;    
    if(!HTS221_Read((uint8_t*)&T_out, HTS221_TEMP_OUT_L, 1))
       return 0;
    if(!HTS221_Read((uint8_t*)&T_out + 1, HTS221_TEMP_OUT_H, 1))
       return 0;
    
    T0_degC = ((float)T0_degC_x8)/8;
    T1_degC = ((float)T1_degC_x8)/8;
    
    T_degC = ((float)(T_out - T0_out))/(T1_out - T0_out) * (T1_degC - T0_degC) + T0_degC;
    
    *temp = (int16_t)(T_degC * 10);
    
    return 1;
}

int HTS221_Read_Humidity(uint16_t* humidity)
{
    uint8_t H0_rh_x2, H1_rh_x2;
    int16_t H0_T0_out, H1_T0_out, H_T_out;
    float H0_rh, H1_rh, H_rh;    
    //int ret;
    
    if(!HTS221_Start_Measure())
        return 0;
    
    if(!HTS221_Read(&H0_rh_x2, HTS221_H0_RH_x2, 1))
        return 0;
    if(!HTS221_Read(&H1_rh_x2, HTS221_H1_RH_x2, 1))
       return 0;
//    ret = HTS221_Read((uint8_t*)&T0_out, HTS221_T0_OUT_L, 2);    
//    ret = HTS221_Read((uint8_t*)&T1_out, HTS221_T1_OUT_L, 2);
//    ret = HTS221_Read((uint8_t*)&T_OUT, HTS221_TEMP_OUT_L, 2);
    
    if(!HTS221_Read((uint8_t*)&H0_T0_out, HTS221_H0_T0_OUT_L, 1))
        return 0;
    if(!HTS221_Read((uint8_t*)&H0_T0_out + 1, HTS221_H0_T0_OUT_H, 1))
        return 0;
    if(!HTS221_Read((uint8_t*)&H1_T0_out, HTS221_H1_T0_OUT_L, 1))
        return 0;
    if(!HTS221_Read((uint8_t*)&H1_T0_out + 1, HTS221_H1_T0_OUT_H, 1))
        return 0;
    if(!HTS221_Read((uint8_t*)&H_T_out, HTS221_HUMIDITY_OUT_L, 1))
        return 0;
    if(!HTS221_Read((uint8_t*)&H_T_out + 1, HTS221_HUMIDITY_OUT_H, 1))
        return 0;
    
    H0_rh = ((float)H0_rh_x2)/2;
    H1_rh = ((float)H1_rh_x2)/2;
    
    H_rh = ((float)(H_T_out - H0_T0_out))/(H1_T0_out - H0_T0_out) * (H1_rh - H0_rh) + H0_rh;
    
    *humidity = (uint16_t)(H_rh * 10);
    
    return 1;
}


/**
 * @}
 */ /* end of group HTS221_API */

/**
*@}
*/ /* end of group HTS221 */



/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
