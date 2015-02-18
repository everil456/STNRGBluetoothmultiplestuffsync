/**
  * @file    LPS25H.c
  * @author  ART Team IMS-Systems Lab
  * @version V1.0.0
  * @date    15 Aprile 2013
 * @brief   Header for  LPS25H.c file
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



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LPS25H_H
#define __LPS25H_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "HAL_LPS25H.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @addtogroup Sensor_Libraries           Sensor Libraries
 * @{
 */
   
/**
 * @defgroup LPS25H
 * @brief This module contains all the functions to configure the LPS25H pressure sensor.
 * @details
 * Since this code is platform independent an implementation of the SPI or I2C driver must
 * be provided by the user according to the used platform.
 * Every function makes use of the <i>Lps25hBufferRead</i> and/or <i>Lps25hBufferWrite</i>
 * as low level functions to write bytes through the used digital interface.
 * In order to link and use this code the user should define and export these functions in a header
 * file called "HAL_LPS25H.h" (included by this module).
 * 
 * @{
 */

/**
 * @defgroup LPS25H_Exported_Types            LPS25H Exported Types
 * @{
 */

/**
 * @brief  LPS25H Functional state. Used to enable or disable a specific option. 
 */   
typedef enum
{
  LPS_DISABLE = 0,
  LPS_ENABLE = !LPS_DISABLE
}LPSFunctionalState; 
   
   
/**
 * @brief  Pressure Flag status. Used to set/reset the sensor flags.
 */   
typedef enum
{
  LPS_RESET = 0,
  LPS_SET = !LPS_RESET
}LPSFlagStatus;


/**
 * @brief LPS25H Output Data Rate
 */   
typedef enum
{
  ODR_P_T_ONE_SHOT        = 0x00,         /*!< Output Data Rate: P - one shot, T - one shot */
  ODR_P_1HZ_T_1HZ         = 0x10,         /*!< Output Data Rate: P - 1Hz, T - 1Hz */
  ODR_P_7HZ_T_7HZ         = 0x20,         /*!< Output Data Rate: P - 7Hz, T - 7Hz */
  ODR_P_12_5HZ_T_12_5HZ   = 0x30,         /*!< Output Data Rate: P - 12.5Hz, T - 12.5Hz */
  ODR_P_25HZ_T_25HZ       = 0x40         /*!< Output Data Rate: P - 25Hz, T - 25Hz */
}LPSOutputDataRate;


/**
 * @brief LPS25H Pressure resolution
 */   
typedef enum
{
  LPS_PRESS_AVG_8          = 0x00,         /*!< Internal average on 8 sample */
  LPS_PRESS_AVG_32         = 0x01,         /*!< Internal average on 32 sample */
  LPS_PRESS_AVG_128        = 0x02,         /*!< Internal average on 128 sample */
  LPS_PRESS_AVG_512        = 0x03         /*!< Internal average on 512 sample */
}LPSPressureResolution;


/**
 * @brief LPS25H Temperature resolution
 */   
typedef enum
{
  LPS_TEMP_AVG_8          = 0x00,         /*!< Internal average on 8 sample */
  LPS_TEMP_AVG_16         = 0x10,         /*!< Internal average on 16 sample */
  LPS_TEMP_AVG_32         = 0x20,         /*!< Internal average on 32 sample */
  LPS_TEMP_AVG_64         = 0x30         /*!< Internal average on 64 sample */
}LPSTemperatureResolution;



/**
 * @brief LPS25H Irq list
 */
typedef enum
{
  LPS_DATA = 0x00,
  LPS_P_HIGH = 0x01,
  LPS_P_LOW = 0x02,
  LPS_P_LOW_HIGH = 0x03
}LPSIrqList;


/**
 * @brief LPS25H Irq pin output configuration
 */
typedef enum
{
  LPS_PP = 0x00,
  LPS_OD = 0x40
}LPSOutputType;


/**
 * @brief LPS25H Irq configuration
 */
typedef enum
{
  LPS_EMPTY = 0x08,
  LPS_WTM = 0x04,
  LPS_OVR = 0x02,
  LPS_DATA_READY = 0x01
}LPSDataSignalType;

/**
 * @brief Pressure sensor Irq initialization structure
 */
typedef struct
{
  LPSFunctionalState xIrqActiveLow;
  LPSOutputType xOutType;
  LPSIrqList xInt1List;
  LPSIrqList xInt2List;
  float fPressThr;
  LPSFunctionalState xIrqPressLow;
  LPSFunctionalState xIrqPressHigh;
  LPSDataSignalType xDataSignalType1; 
  LPSDataSignalType xDataSignalType2;
}LPSIrqInit;


/**
 * @brief Pressure sensor Init structure definition
 */
typedef struct
{
  LPSOutputDataRate xOutputDataRate;      /*!< Output Data Rate */
  LPSPressureResolution xPresRes;         /*!< Pressure sensor resolution */
  LPSTemperatureResolution xTempRes;      /*!< Temperature sensor resolution */
  LPSFunctionalState xPressureAutoZero;   /*!< Auto zero feature enabled */
  float fPressureRef;                     /*!< Pressure sensor reference value */
  LPSFunctionalState xBDU;                /*!< Auto zero feature enabled */
}LPS25HInit;


/**
 * @brief Pressure sensor data status structure structure
 */
typedef struct
{
  LPSFlagStatus cTempDataAvailable:1;           /*!< Temperature data available bit */
  LPSFlagStatus cPressDataAvailable:1;          /*!< Pressure data available bit */
  uint8_t :2;                                   /*!< 2 bits padding */
  LPSFlagStatus cTempDataOverrun:1;             /*!< Temperature data over-run bit */
  LPSFlagStatus cPressDataOverrun:1;            /*!< Pressure data over-run bit */
  uint8_t :2;                                   /*!< 2 bits padding */
}LPS25HDataStatus;

/**
 * @brief Pressure sensor FIFO status structure structure
 */
typedef struct
{
  uint8_t FIFO_DIFF_POINT:5;                /*!< FIFO stored data level */
  LPSFlagStatus FIFO_EMPTY_FLAG:1;          /*!< FIFO Empty flag */
  LPSFlagStatus FIFO_OVRN_FLAG:1;           /*!< FIFO Overrun flag */
  LPSFlagStatus FIFO_WTM_FLAG:1;            /*!< FIFO Watermark flag */          /*!< 2 bits padding */
}LPS25HFifoStatus;

/**
 * @}
 */    
  
/**
 * @defgroup LPS25H_Exported_Constants       LPS25H Exported Constants
 * @{
 */


/**
 * @}
 */

/**
 * @defgroup LPS25H_Exported_Macros         LPS25H Exported Macros
 * @{
 */


/** @defgroup LPS25H_Communication        LPS25H Communication
 * @{
 */

#define Lps25hByteRead(pVal,cAddress)               LPS25H_Read(pVal,cAddress,1)
#define Lps25hByteWrite(pVal,cAddress)              LPS25H_Write(pVal,cAddress,1)


/**
 * @} 
 */

/**
 * @} 
 */  


/** @defgroup LPS25H_Register_Mapping       LPS25H Register Mapping
 * @{
 */

/**
 * @brief Reference pressure (LSB data)
 * \code
 * Read/write
 * Default value: 0x00
 * 7:0 REF7-ODR0: Lower part of the reference pressure that
 *      is sum to the sensor output pressure.
 * \endcode
 */
#define LPS_REF_P_XL_ADDR         0x08


/**
 * @brief Reference pressure (middle part)
 * \code
 * Read/write
 * Default value: 0x00
 * 7:0 REF15-ODR8: Middle part of the reference pressure that
 *      is sum to the sensor output pressure.
 * \endcode
 */
#define LPS_REF_P_L_ADDR          0x09


/**
 * @brief Reference pressure (MSB part)
 * \code
 * Read/write
 * Default value: 0x00
 * 7:0 REF15-ODR8: Higher part of the reference pressure that
 *      is sum to the sensor output pressure.
 * \endcode
 */
#define LPS_REF_P_H_ADDR          0x0A


/**
 * @brief Device identifier register.
 * \code
 * Read
 * Default value: 0xBD
 * 7:0 This read-only register contains the device identifier that, for LPS25H, is set to BDh.
 * \endcode
 */
#define LPS_WHO_AM_I_ADDR                 0x0F


/**
 * @brief Pressure resolution Register
 * \code
 * Read/write
 * Default value: 0x7A
 * 7:4 RFU
 * 3:2 AVGT1-AVGT0: temperature internal average.
 *      AVGT1 | AVGT0 | Nr. Internal Average
 *   ------------------------------------------------------
 *       0    |  0    |     8 
 *       0    |  1    |     32   
 *       1    |  0    |     128    
 *       1    |  1    |     512 

 *
 * 1:0 AVGP1-AVGP0: pressure internal average.
 *      AVGP1 |  AVGP0 | Nr. Internal Average
 *   ------------------------------------------------------
 *      0    |   0    |      8
 *      0    |   1    |      16   
 *      1    |   0    |      32   
 *      1    |   1    |      64
  
 *
 * \endcode
 */
#define LPS_RES_CONF_ADDR                 0x10 




/**
 * @brief Pressure sensor control register 1
 * \code
 * Read/write
 * Default value: 0x00
 * 7 PD: power down control. 0 - disable; 1 - enable
 * 6:4 ODR2, ODR1, ODR0: output data rate selection.
 *     ODR2  | ODR1  | ODR0  | Pressure output data-rate(Hz)  | Pressure output data-rate(Hz)
 *   ----------------------------------------------------------------------------------
 *      0    |  0    |  0    |         one shot               |         one shot 
 *      0    |  0    |  1    |            1                   |            1 
 *      0    |  1    |  0    |            7                   |            7     
 *      0    |  1    |  1    |            12.5                |            12.5  
 *      1    |  0    |  0    |            25                  |            25    
 *      1    |  0    |  1    |         Reserved               |         Reserved 
 *      1    |  1    |  0    |         Reserved               |         Reserved 
 *      1    |  1    |  1    |         Reserved               |         Reserved
 *
 * 3 DIFF_EN: Interrupt circuit. 0 - disable; 1 - enable
 * 2 BDU: block data update. 0 - disable; 1 - enable
 * 1 DELTA_EN: delta pressure. 0 - disable; 1 - enable
 * 1 RESET_AZ: reset AutoZero. 0 - disable; 1 - enable  ///////ALE REVIEW
 * 0 SIM: SPI Serial Interface Mode selection. 0 - SPI 4-wire; 1 - SPI 3-wire ///////ALE REVIEW
 * \endcode
 */
#define LPS_CTRL_REG1_ADDR                    0x20


/**
 * @brief Pressure sensor control register 2
 * \code
 * Read/write
 * Default value: 0x00
 * 7 BOOT:  Reboot memory content. 0: normal mode; 1: reboot memory content
 * 6 FIFO_EN: FIFO. 0: disable; 1:  enable
 * 5 WTM_EN:  FIFO Watermark level use. 0: disable; 1: enable
 * 4:3 Reserved. keep these bits at 0
 * 2 SWRESET: Software reset. 0: normal mode; 1: SW reset.
 * 1 AUTO_ZERO: Autozero enable. 0: normal mode; 1: autozero enable.
 * 0 ONE_SHOT: One shot enable. 0: waiting for start of conversion; 1: start for a new dataset
 * \endcode
 */
#define LPS_CTRL_REG2_ADDR                      0x21


/**
 * @brief Pressure sensor control register 3
 * \code
 * Read/write
 * Default value: 0x00
 * 7 INT_H_L: Interrupt. 0:active high; 1: active low.
 * 6 PP_OD: Push-Pull/OpenDrain selection on interrupt pads. 0: Push-pull; 1: open drain.
 * 5 Reserved
 * 4:3 INT2_S2, INT2_S1: INT2 output signal selection control bits. // TO DO
 * 1:0 INT1_S2, INT1_S1: data signal on INT1 pad control bits.
 *    INT1(2)_S2  | INT1(2)_S1  | INT1(2) pin
 *   ------------------------------------------------------
 *        0       |      0      |     Data signal 
 *        0       |      1      |     Pressure high (P_high) 
 *        1       |      0      |     Pressure low (P_low)     
 *        1       |      1      |     P_low OR P_high 
 

 * \endcode
 */
#define LPS_CTRL_REG3_ADDR                    0x22 



/**
 * @brief Pressure sensor control register 4
 * \code
 * Read/write
 * Default value: 0x00
 * 7 P2_EMPTY: Empty Signal on INT2 pin.
 * 6 P2_WTM: Watermark Signal on INT2 pin.
 * 5 P2_Overrun:Overrun Signal on INT2 pin.
 * 4 P2_DRDY: Data Ready Signal on INT2 pin.
 * 3 P1_EMPTY: Empty Signal on INT1 pin.
 * 2 P1_WTM: Watermark Signal on INT1 pin.
 * 1 P1_Overrunn:Overrun Signal on INT1 pin.
 * 0 P1_DRDY: Data Ready Signal on INT1 pin.
 * \endcode
 */
#define LPS_CTRL_REG4_ADDR                    0x23 


/**
 * @brief Interrupt configuration Register
 * \code
 * Read/write
 * Default value: 0x00.
 * 7:3 Reserved.
 * 2 LIR: Latch Interrupt request into INT_SOURCE register. 0 - disable; 1 - enable
 * 1 PL_E: Enable interrupt generation on differential pressure low event. 0 - disable; 1 - enable
 * 0 PH_E: Enable interrupt generation on differential pressure high event. 0 - disable; 1 - enable
 * \endcode
 */
#define LPS_INT_CFG_REG_ADDR                  0x24 


/**
 * @brief Interrupt source Register
 * \code
 * Read
 * Default value: 0x00.
 * 7:3 0.
 * 2 IA: Interrupt Active.0: no interrupt has been generated; 1: one or more interrupt events have been generated.
 * 1 PL: Differential pressure Low. 0: no interrupt has been generated; 1: Low differential pressure event has occurred.
 * 0 PH: Differential pressure High. 0: no interrupt has been generated; 1: High differential pressure event has occurred.
 * \endcode
 */
#define LPS_INT_SOURCE_REG_ADDR               0x25 


/**
 * @brief Threshold pressure (LSB)
 * \code
 * Read
 * Default value: 0x00.
 * 7:0 THS7-THS0: Low part of threshold value for pressure interrupt
 * generation. The complete threshold value is given by THS_P_H & THS_P_L and is
 * expressed as unsigned number. P_ths(mbar)=(THS_P_H & THS_P_L)[dec]/16.
 * \endcode
 */
#define LPS_THS_P_LOW_REG_ADDR                0x30 


/**
 * @brief Threshold pressure (MSB)
 * \code
 * Read
 * Default value: 0x00.
 * 7:0 THS15-THS8: High part of threshold value for pressure interrupt
 * generation. The complete threshold value is given by THS_P_H & THS_P_L and is
 * expressed as unsigned number. P_ths(mbar)=(THS_P_H & THS_P_L)[dec]/16.
 * \endcode
 */
#define LPS_THS_P_HIGH_REG_ADDR              0x31 


/**
 * @brief  Status Register
 * \code
 * Read
 * Default value: 0x00
 * 7:6 0
 * 5 P_OR: Pressure data overrun. 0: no overrun has occurred; 1: new data for pressure has overwritten the previous one.
 * 4 T_OR: Temperature data overrun. 0: no overrun has occurred; 1: a new data for temperature has overwritten the previous one.
 * 3:2 0
 * 1 P_DA: Pressure data available. 0: new data for pressure is not yet available; 1: new data for pressure is available.
 * 0 T_DA: Temperature data available. 0: new data for temperature is not yet available; 1: new data for temperature is available.
 * \endcode
 */
#define LPS_STATUS_REG_ADDR                 0x27 


/**
 * @brief  Pressure data (LSB).
 * \code
 * Read
 * Default value: 0x00.
 * POUT7 - POUT0: Pressure data LSB (2's complement).     
 * Pressure output data: Pout(mbar)=(PRESS_OUT_H & PRESS_OUT_L &
 * PRESS_OUT_XL)[dec]/4096.
 * \endcode
 */
#define LPS_PRESS_POUT_XL_ADDR              0x28 


/**
 * @brief  Pressure data (Middle part).
 * \code
 * Read
 * Default value: 0x80.
 * POUT15 - POUT8: Pressure data middle part (2's complement).    
 * Pressure output data: Pout(mbar)=(PRESS_OUT_H & PRESS_OUT_L &
 * PRESS_OUT_XL)[dec]/4096.
 * \endcode
 */
#define LPS_PRESS_OUT_L_ADDR                0x29


/**
 * @brief  Pressure data (MSB).
 * \code
 * Read
 * Default value: 0x2F.
 * POUT23 - POUT16: Pressure data MSB (2's complement).
 * Pressure output data: Pout(mbar)=(PRESS_OUT_H & PRESS_OUT_L &
 * PRESS_OUT_XL)[dec]/4096.
 * \endcode
 */
#define LPS_PRESS_OUT_H_ADDR                0x2A


/**
 * @brief  Temperature data (LSB).
 * \code
 * Read
 * Default value: 0x00.
 * TOUT7 - TOUT0: temperature data LSB. 
 * T(degC) = 42.5 + (Temp_OUTH & TEMP_OUT_L)[dec]/480.
 * \endcode
 */
#define LPS_TEMP_OUT_L_ADDR                 0x2B


/**
 * @brief  Temperature data (MSB).
 * \code
 * Read
 * Default value: 0x00.
 * TOUT15 - TOUT8: temperature data MSB. 
 * T(degC) = 42.5 + (Temp_OUTH & TEMP_OUT_L)[dec]/480.
 * \endcode
 */
#define LPS_TEMP_OUT_H_ADDR                 0x2C


/**
 * @brief FIFO control register 
 * \code
 * Read/write
 * Default value: 0x00
 * 7:5 F_MODE2, F_MODE1, F_MODE0: FIFO mode selection.
 *     FM2   | FM1   | FM0   |    FIFO MODE
 *   ---------------------------------------------------
 *      0    |  0    |  0    |      BYPASS MODE               
 *      0    |  0    |  1    | FIFO MODE. Stops collecting data when full               
 *      0    |  1    |  0    | STREAM MODE: Keep the newest measurements in the FIFO                  
 *      0    |  1    |  1    | STREAM MODE until trigger deasserted, then change to FIFO MODE              
 *      1    |  0    |  0    | BYPASS MODE until trigger deasserted, then STREAM MODE                 
 *      1    |  0    |  1    |       Reserved              
 *      1    |  1    |  0    | FIFO_MEAN MODE: Fifo is used to generate a running average filtered pressure              
 *      1    |  1    |  1    | BYPASS mode until trigger deasserted, then FIFO MODE              
 *
 * 4:0 FIFO Mean Mode Sample size
 *     WTM_POINT4 | WTM_POINT4 | WTM_POINT4 |  WTM_POINT4 | WTM_POINT4 | Sample Size
 *   ----------------------------------------------------------------------------------
 *      0         |    0       |    0       |      0      |     1      |       2  
 *      0         |    0       |    0       |      1      |     1      |       4
 *      0         |    0       |    1       |      1      |     1      |       8 
 *      0         |    1       |    1       |      1      |     1      |       16
 *      1         |    1       |    1       |      1      |     1      |       32
 * other values operation not guaranteed       
 * \endcode
 */
#define LPS_CTRL_FIFO_ADDR                    0x2E


/**
 * @brief FIFO Status register 
 * \code
 * Read/write
 * Default value: 0x00
 * 7 WTM_FIFO: Watermark status. 0:FIFO filling is lower than watermark level; 1: FIFO is equal or higher than watermark level.           
 * 6 FULL_FIFO: Overrun bit status. 0 - FIFO not full; 1 -FIFO is full.
 * 5 EMPTY_FIFO: Empty FIFO bit. 0 - FIFO not empty; 1 -FIFO is empty.
 * 4:0 DIFF_POINT4...0: FIFOsStored data level.
 * \endcode
 */
#define LPS_STATUS_FIFO_ADDR                    0x2F

/**
 * @brief Pressure offset register 
 * \code
 * Read/write
 * Default value: 0x00
 * 7:0 RPDS15...8:Pressure Offset for 1 point calibration after soldering.           
 * \endcode
 */
#define LPS_RPDS_TRIM_L_ADDR                    0x39


/**
 * @brief Pressure offset register 
 * \code
 * Read/write
 * Default value: 0x00
 * 7:0 RPDS23...16:Pressure Offset for 1 point calibration after soldering.           
 * \endcode
 */
#define LPS_RPDS_TRIM_H_ADDR                    0x3A

/**
 * @}
 */



/** @defgroup LPS25H_Exported_Functions           LPS25H Exported Functions
 * @{
 */

void Lps25hConfig(LPS25HInit* pxLPS25HInitStruct);
void Lps25hGetInfo(LPS25HInit* pxLPS25HInitStruct);
void Lps25hSetPressureResolution(LPSPressureResolution xPressureResolution);
void Lps25hSetTemperatureResolution(LPSTemperatureResolution xTemperatureResolution);
void Lps25hSetDataRate(LPSOutputDataRate xDataRate);
LPSOutputDataRate Lps25hGetDataRate(void);
void Lps25hRebootCmd(void);
void Lps25hEnterShutdownCmd(void);
void Lps25hExitShutdownCmd(void);
int Lps25hReadRawPressure(int32_t *raw_press);
int Lps25hReadPressure(int32_t *press);
int Lps25hReadRawTemperature(int16_t *raw_data);
int Lps25hReadTemperature(int16_t *temperature);
int Lps25hStartNReadTemperature(int16_t *temperature);
void Lps25hIrqInit(LPSIrqInit* pxLPSIrqInit);
int Lps25hOneShot(void);
LPS25HDataStatus Lps25hGetDataStatus(void);
void Lps25hSetThreshold(float fThreshold);
float Lps25hGetThreshold(void);
LPS25HFifoStatus Lps25hGetFifoStatus(void);


/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __LPS25H_H */

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
