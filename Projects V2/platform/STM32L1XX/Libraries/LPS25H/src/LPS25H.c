/**
  * @file    LPS25H.c
  * @author  ART Team IMS-Systems Lab
  * @version V1.0.0
  * @date    15 Aprile 2013
  * @brief   This file provides a set of functions needed to manage the LPS25H slave.
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
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  */


#include "LPS25H.h"
#include "gp_timer.h"
#include <string.h>


/**
* @addtogroup LPS25H
* @{
*/

/**
* @defgroup LPS25H_API
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
* @brief  Set configuration of Pressure measurement of LPS25H
* @param  pxLPS25HInitStruct : pointer to a LPS25HInit structure that contains the configuration setting for the LPS25H.
* @retval None.
* @details
* <b>Example:</b>
* @code
*  LPS25HInit xLPS25HInitStructure;
*
*  xLPS25HInit.xOutputDataRate=ODR_P_7HZ_T_7HZ;
*  xLPS25HInit.xPresRes=LPS_PRESS_AVG_8;
*  xLPS25HInit.xTempRes=LPS_TEMP_AVG_8;
*  xLPS25HInit.xPressureAutoZero=LPS_DISABLE;
*  xLPS25HInit.fPressureRef=0;
*  xLPS25HInit.xBDU=LPS_DISABLE;
*
*  LPS25HConfig(&xLPS25HInit);
* @endcode
*/
void Lps25hConfig(LPS25HInit* pxLPS25HInitStruct)
{
  uint8_t tempReg[3];
  
  
  for(uint8_t i=0 ; i<3 ; i++)
    tempReg[i]=(uint8_t)(((uint32_t)(4096.0*pxLPS25HInitStruct->fPressureRef))>>(8*i));
      
  /* Write the reference value on register */
  LPS25H_Write(tempReg, LPS_REF_P_XL_ADDR,3);
  
  /* Resolution register setting */
  tempReg[0] = (uint8_t)pxLPS25HInitStruct->xPresRes | (uint8_t)pxLPS25HInitStruct->xTempRes;
    
  /* Write the value on register */ 
  Lps25hByteWrite(&tempReg[0], LPS_RES_CONF_ADDR);
  
  /* Read the CTRL1 and 2 register content */
  LPS25H_Read(tempReg, LPS_CTRL_REG1_ADDR,2); 
  
  /* Enter the SDN mode */
  tempReg[0] &= 0x7F;
  Lps25hByteWrite(tempReg, LPS_CTRL_REG1_ADDR);
  
  /* CTRL1 register */
  tempReg[0] &= 0x8B;
  tempReg[0] |= pxLPS25HInitStruct->xOutputDataRate;
    
  if(pxLPS25HInitStruct->xBDU)
    tempReg[0] |= 0x04;
  else
    tempReg[0] &= 0xFB;
 /*FIFO_EN e WTM_EN li mettiamo di default a 0 da vedere dove abilitarli*/   
  /* CTRL2 register */
  if(pxLPS25HInitStruct->xPressureAutoZero)
    tempReg[1] |= 0x02;
  else
    tempReg[1] &= 0xFD;

  /* Write the reference value on register */
  LPS25H_Write(tempReg, LPS_CTRL_REG1_ADDR,2);
  
  /* Exit the SDN mode */
  tempReg[0] |= 0x80;
  Lps25hByteWrite(tempReg, LPS_CTRL_REG1_ADDR);
  
}


/**
* @brief  Gets the general configuration of LPS25H.
* @param  pxLPS25HInitStruct : pointer to a LPS25HInit structure that will
*         contain the configuration setting read from the LPS25H registers.
* @retval None
*/
void Lps25hGetInfo(LPS25HInit* pxLPS25HInitStruct)
{
  uint8_t tempReg[4];
  
  LPS25H_Read(tempReg, LPS_REF_P_XL_ADDR,3);
  tempReg[3]=0;
  
  /* Get the reference value */
  pxLPS25HInitStruct->fPressureRef = (float)(*((uint32_t*)tempReg))/4096.0;

  /* Read register */ 
  Lps25hByteRead(&tempReg[0], LPS_RES_CONF_ADDR);
  
  pxLPS25HInitStruct->xPresRes=(LPSPressureResolution)(tempReg[0] & 0x03); 
  pxLPS25HInitStruct->xTempRes=(LPSTemperatureResolution)(tempReg[0] & 0x0C);
  
  /* Read the CTRL1 and 2 register content */
  LPS25H_Read(tempReg, LPS_CTRL_REG1_ADDR,2);
  
  /* get info */
  if(tempReg[0] & 0x04)
    pxLPS25HInitStruct->xBDU = LPS_ENABLE;
  else
    pxLPS25HInitStruct->xBDU = LPS_DISABLE;
   

  if(tempReg[1] & 0x02)
    pxLPS25HInitStruct->xPressureAutoZero = LPS_ENABLE;
  else
    pxLPS25HInitStruct->xPressureAutoZero = LPS_DISABLE;
    
  /* Get the ODR info */
  Lps25hByteRead(tempReg, LPS_CTRL_REG1_ADDR);
  
  pxLPS25HInitStruct->xOutputDataRate = (LPSOutputDataRate)(tempReg[0] & 0x70);
  
}

/**
* @brief  Change pressure resolution for Pressure sensor LPS25H.
* @param  xPressureResolution : new pressure resolution value. 
*         This parameter can be one of the @ref LPSPressureResolution value.
* @retval None
*/
void Lps25hSetPressureResolution(LPSPressureResolution xPressureResolution)
{
  uint8_t tempReg;
  
  /* Read the register content */
  Lps25hByteRead(&tempReg, LPS_RES_CONF_ADDR);
  
  /* CTRL1 register */
  tempReg &= 0xFC;
  tempReg |= xPressureResolution;
   
  /* Write computed byte onto register */
  Lps25hByteWrite(&tempReg, LPS_RES_CONF_ADDR);
}


/**
* @brief  Change temperature resolution for Pressure sensor LPS25H.
* @param  xTemperatureResolution : new pressure resolution value. 
*         This parameter can be one of the @ref LPSTemperatureResolution value.
* @retval None
*/
void Lps25hSetTemperatureResolution(LPSTemperatureResolution xTemperatureResolution)
{
  uint8_t tempReg;
  
  /* Read the register content */
  Lps25hByteRead(&tempReg, LPS_RES_CONF_ADDR);
  
  /* CTRL1 register */
  tempReg &= 0xF3;
  tempReg |= xTemperatureResolution;
   
  /* Write computed byte onto register */
  Lps25hByteWrite(&tempReg, LPS_RES_CONF_ADDR);
}


/**
* @brief  Change the ODR(Output data rate) for LPS25H.
* @param  xDataRate : new ODR value. 
*         This parameter can be one of the @ref LPSOutputDataRate value.
* @retval None
*/
void Lps25hSetDataRate(LPSOutputDataRate xDataRate)
{
  uint8_t tempReg;
  
  /* Read the register content */
  Lps25hByteRead(&tempReg, LPS_CTRL_REG1_ADDR);
  
  /* CTRL1 register */
  tempReg &= 0x8F;
  tempReg |= xDataRate;
  
  /*Enter Power Down mode*/
  Lps25hEnterShutdownCmd();
  
  /* Write computed byte onto register */
  Lps25hByteWrite(&tempReg, LPS_CTRL_REG1_ADDR);
  
  /*Exit from Power down mode*/
  Lps25hExitShutdownCmd();
}


/**
* @brief  Returns the output data rate.
* @param  None.
* @retval Datarate in Hz.
*         This parameter is a @ref LPSOutputDataRate.
*/
LPSOutputDataRate Lps25hGetDataRate(void)
{
  uint8_t tmpReg;
  
  /* Read the register content */
  Lps25hByteRead(&tmpReg, LPS_CTRL_REG1_ADDR);
  
  /* ..mask it */
  tmpReg &= 0x70;

  /* return the correspondent value */
  return ((LPSOutputDataRate)tmpReg);
}



/**
* @brief  Reboot memory content of LPS25H.
* @param  None
* @retval None
*/
void Lps25hRebootCmd(void)
{
  uint8_t tmpReg;
  
  /* Read the register content */
  Lps25hByteRead(&tmpReg, LPS_CTRL_REG2_ADDR);
  
  /* Set the BOOT bit */
  tmpReg |= 0x80;
  
  /* Write register */
  Lps25hByteWrite(&tmpReg, LPS_CTRL_REG2_ADDR);
  
}


/**
* @brief  Enter the shutdown mode for LPS25H.
* @param  None
* @retval None
*/
void Lps25hEnterShutdownCmd(void)
{
  uint8_t tmpReg;
  
  /* Read the register content */
  Lps25hByteRead(&tmpReg, LPS_CTRL_REG1_ADDR);
  
  /* Reset the power down bit */
  tmpReg &= 0x7F;
  
  /* Write register */
  Lps25hByteWrite(&tmpReg, LPS_CTRL_REG1_ADDR);
  
}


/**
* @brief  Exit the shutdown mode for LPS25H.
* @param  None
* @retval None
*/
void Lps25hExitShutdownCmd(void)
{
  uint8_t tmpReg;
  
  /* Read the register content */
  Lps25hByteRead(&tmpReg, LPS_CTRL_REG1_ADDR);
  
  /* Set the power down bit */
  tmpReg |= 0x80;
  
  /* Write register */
  Lps25hByteWrite(&tmpReg, LPS_CTRL_REG1_ADDR);
  
}

/**
* @brief Read LPS25H output register, and calculate the raw  pressure.
* @param None.
* @retval int32_t: pressure raw value.
*/
int Lps25hReadRawPressure(int32_t *raw_press)
{
  uint8_t buffer[3];
  uint32_t tempVal=0;
  
  /* Read the register content */
  if(!LPS25H_Read(buffer, LPS_PRESS_POUT_XL_ADDR, 3))
      return 0;
  
  /* Build the raw data */
  for(uint8_t i=0; i<3; i++)
      tempVal |= (((uint32_t)buffer[i]) << (8*i));
  
  /* convert the 2's complement 24 bit to 2's complement 32 bit */
  if(tempVal & 0x00800000)
    tempVal |= 0xFF000000;
  
  /* return the built value */
  *raw_press =((int32_t)tempVal);
  
  return 1;
    
}

/**
* @brief Read LPS25H output register, and calculate the pressure in hundredths of mbar.
* @param None.
* @retval int16_t: pressure value in mbar.
*/
int Lps25hReadPressure(int32_t *press)
{
  int32_t raw_press = 0;
 
  if(!Lps25hReadRawPressure(&raw_press))
      return 0;
  
  /* return the built value */
  *press = (raw_press*100)/4096;
  
  return 1;
    
}


/**
* @brief Read LPS25H output register, and calculate the raw temperature.
* @param None.
* @retval int16_t: temperature raw value.
*/
int Lps25hReadRawTemperature(int16_t *raw_data)
{
  uint8_t buffer[2];
  uint16_t tempVal=0;
  
  /* Read the register content */
  if(!LPS25H_Read(buffer, LPS_TEMP_OUT_L_ADDR, 2))
      return 0;
  
  /* Build the raw value */
  tempVal = (((uint16_t)buffer[1]) << 8)+(uint16_t)buffer[0];
  
  /* Return it */
  *raw_data = ((int16_t)tempVal);
  
  return 1;    
}

/**
* @brief Read LPS25H output register, and calculate the raw temperature.
* @param None.
* @retval int16_t: temperature raw value.
*/
int Lps25hReadTemperature(int16_t *temperature)
{
  int16_t raw_data;
  
  if(!Lps25hReadRawTemperature(&raw_data))
      return 0;
  
  *temperature = (int16_t)((((float)raw_data/480.0) + 42.5)*10);
  
  return 1;
    
}

/**
* @brief Read LPS25H output register, and calculate the raw temperature.
* @param None.
* @retval int16_t: temperature raw value.
*/
int Lps25hStartNReadTemperature(int16_t *temperature)
{
  struct timer t;
  LPS25HDataStatus status;
  
  Timer_Set(&t, CLOCK_SECOND/10);
  
  memset(&status,0,sizeof(LPS25HDataStatus));
  
  if(!Lps25hOneShot())
    return 0;
  
  while(!status.cTempDataAvailable){
    status = Lps25hGetDataStatus();
    if(Timer_Expired(&t))
      return 0;
  }
  if(!Lps25hReadTemperature(temperature))
    return 0;
  
  return 1;
    
}


/**
* @brief  Configures the LPS25H IRQ outout pins.
* @param  pxLPSIrqInit: pointer to the LPSIrqInit structure that defines the IRQ parameters.
* @retval None.
* @details
* <b>Example:</b>
* @code
*  LPSIrqInit xLps25hIrqInit;
*
*  xLps25hIrqInit.xIrqActiveLow = LPS_DISABLE;
*  xLps25hIrqInit.xOutType = LPS_PP;
*  xLps25hIrqInit.xInt1List = LPS_DATA;
*  xLps25hIrqInit.xInt2List = LPS_P_LOW_HIGH; //NOT USED
*  xLps25hIrqInit.fPressThr = 0;
*  xLps25hIrqInit.xIrqPressLow=LPS_ENABLE;
*  xLps25hIrqInit.xIrqPressHigh=LPS_ENABLE;

*  xLps25hIrqInit.xDataSignalType1 = LPS_DATA_READY; 
*  xLps25hIrqInit.xDataSignalType2 = LPS_EMPTY; //NOT USED
*
*  ExtiConfiguration();     // set the micro exti before init
*  Lps25hIrqInit(&xLps25hIrqInit);
* @endcode
*/
void Lps25hIrqInit(LPSIrqInit* pxLPSIrqInit)
{
  uint8_t tempReg[2];
    
  /* From the structure build the value to write on CTRL3 reg */
  tempReg[0] = (((uint8_t)(pxLPSIrqInit->xIrqActiveLow))<<7) | ((uint8_t)(pxLPSIrqInit->xOutType)) |/* ((uint8_t)(pxLPSIrqInit->xInt2List)<<3) |*/ ((uint8_t)(pxLPSIrqInit->xInt1List));

  /* Read the register content */
  Lps25hByteWrite(tempReg, LPS_CTRL_REG3_ADDR);
  
  /* Build the threshold register values */
  tempReg[0]=(uint8_t)(16.0*pxLPSIrqInit->fPressThr);
  tempReg[1]=(uint8_t)(((uint16_t)(16.0*pxLPSIrqInit->fPressThr))>>8);  
  
  /* Read the registers content */
  LPS25H_Write(tempReg, LPS_THS_P_LOW_REG_ADDR, 2);
  
  Lps25hByteRead(tempReg, LPS_INT_CFG_REG_ADDR);
  
  /* Enable or disable the high pressure interrupt */
  if(pxLPSIrqInit->xIrqPressHigh)
    tempReg[0] |= 0x01;
  else
    tempReg[0] &= 0xFE;
  
  /* Enable or disable the low pressure interrupt */
  if(pxLPSIrqInit->xIrqPressLow)
    tempReg[0] |= 0x02;
  else
    tempReg[0] &= 0xFD;
     
  /* Write the register or disable the high pressure interrupt */
  Lps25hByteWrite(tempReg, LPS_INT_CFG_REG_ADDR);
  
  /* if one has been requested then enable the differential circuit */
  Lps25hByteRead(tempReg, LPS_CTRL_REG1_ADDR);
  if(pxLPSIrqInit->xIrqPressHigh || pxLPSIrqInit->xIrqPressLow)
  {    
    tempReg[0]|=0x08; 
  }
  else
  /* else disable it */
  {
    tempReg[0]&=0xF7; 
  }
  Lps25hByteWrite(tempReg, LPS_CTRL_REG1_ADDR);
  
  /* if Data_Signal has been requested then select the type of Data */
  Lps25hByteRead(tempReg, LPS_CTRL_REG4_ADDR);
//  if((pxLPSIrqInit->xInt1List || pxLPSIrqInit->xInt2List)== LPS_DATA)
//  {    
//     tempReg[0] = (((uint8_t)(pxLPSIrqInit->xDataSignalType2))<<4)|((uint8_t)(pxLPSIrqInit->xDataSignalType1));
//  }
   if(pxLPSIrqInit->xInt1List == LPS_DATA)
  {    
     tempReg[0] = /*(((uint8_t)(pxLPSIrqInit->xDataSignalType2))<<4)|*/((uint8_t)(pxLPSIrqInit->xDataSignalType1));
  }
  else
  /* else nothing */
  {
     
  }
  Lps25hByteWrite(tempReg, LPS_CTRL_REG4_ADDR);

  
}



/**
* @brief  Sets the one-shot bit in order to start acquisition when the ONE SHOT mode has been selected by the ODR configuration.
* @param  None.
* @retval 1 if success, 0 otherwise.
*/
int Lps25hOneShot(void)
{
  uint8_t tempReg;
  
  /* Read the CTRL2 register */
  if(!Lps25hByteRead(&tempReg, LPS_CTRL_REG2_ADDR))
      return 0;
    
  /* Set the one shot bit */
  tempReg |= 0x01;
  
  /* Write the CTRL2 register */
  Lps25hByteWrite(&tempReg, LPS_CTRL_REG2_ADDR);
  
  return 1;

}


/**
* @brief  Gets Data status for LPS25H data.
* @param  None.
* @retval LPS25HDataStatus: Data status in a LPS25HDataStatus bit fields structure.
*/
LPS25HDataStatus Lps25hGetDataStatus(void)
{
  uint8_t tempReg;
  
  /* Read the status register */
  Lps25hByteRead(&tempReg, LPS_STATUS_REG_ADDR);
  
  /* cast and return it */
  return(*(LPS25HDataStatus*)&tempReg);
  
}


/** 
* @brief  Sets the pressure threshold.
* @param  fThreshold: Threshold expressed in mbar.
*         This parameter is a float value.
* @retval None.
*/
void Lps25hSetThreshold(float fThreshold)
{
  uint8_t tempReg[2];
  
  /* Build the threshold register values */
  tempReg[0]=(uint8_t)(16.0*fThreshold);
  tempReg[1]=(uint8_t)(((uint16_t)(16.0*fThreshold))>>8); 
  
  /* write the register content */
  LPS25H_Write(tempReg, LPS_THS_P_LOW_REG_ADDR, 2);
  
}


/** 
* @brief  Gets the pressure threshold.
* @param  None.
* @retval float: threshold value expressed in mbar.
*/
float Lps25hGetThreshold(void)
{
  uint8_t tempReg[2];
  
  /* read the register content */
  LPS25H_Read(tempReg, LPS_THS_P_LOW_REG_ADDR, 2);
  
  /* return the float value */
  return ((float)((((uint16_t)tempReg[1])<<8) + tempReg[0])/16);
  
}


/**
* @brief  Gets FIFO status for LPS25H data.
* @param  None.
* @retval LPS25HDFifoStatus: FIFO status in a LPS25HFifoStatus bit fields structure.
*/
LPS25HFifoStatus Lps25hGetFifoStatus(void)
{
  uint8_t tempReg;
  
  /* Read the status register */
  Lps25hByteRead(&tempReg, LPS_STATUS_FIFO_ADDR);
  
  /* cast and return it */
  return(*(LPS25HFifoStatus*)&tempReg);
  
}
void Lps25hConfig(LPS25HInit* pxLPS25HInitStruct);

/**
 * @}
 */ /* end of group LPS25H_API */

/**
*@}
*/ /* end of group LPS25H */



/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
