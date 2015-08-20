/**************************************************************************/
/*! 
	@file     Adafruit_INA219.cpp
	@author   K.Townsend (Adafruit Industries)
	@license  BSD (see license.txt)
	
	Driver for the INA219 current sensor

	This is a library for the Adafruit INA219 breakout
	----> https://www.adafruit.com/products/???
		
	Adafruit invests time and resources providing this open source code, 
	please support Adafruit and open-source hardware by purchasing 
	products from Adafruit!

	@section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/


#include "lib_ina219.h"

extern I2C_HandleTypeDef hi2c1 ;

uint16_t ina219_powerDivider_mW;
uint16_t ina219_currentDivider_mA;
uint16_t ina219_powerDivider_mW;

/**************************************************************************/
/*! 
    @brief  Sends a single command byte over I2C
*/
/**************************************************************************/
void ina219_WriteRegister(uint8_t reg, uint16_t *value)
{
	uint8_t i2c_temp[2];
	i2c_temp[0] = *value>>8;
	i2c_temp[1] = *value;
	HAL_I2C_Mem_Write(&hi2c1, ina219_i2caddr<<1, (uint16_t)reg, 1, i2c_temp, 2, 1);
}

/**************************************************************************/
/*! 
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
void ina219_ReadRegister(uint8_t reg, uint16_t *value)
{
	uint8_t i2c_temp[2];
	HAL_I2C_Mem_Read(&hi2c1, ina219_i2caddr<<1, (uint16_t)reg, 1,i2c_temp, 2, 1);
	*value = ((uint16_t)i2c_temp[0]<<8 )|(uint16_t)i2c_temp[1];
}

/**************************************************************************/
/*! 
    @brief  Configures to INA219 to be able to measure up to 32V and 2A
            of current.  Each unit of current corresponds to 100uA, and
            each unit of power corresponds to 2mW. Counter overflow
            occurs at 3.2A.
			
    @note   These calculations assume a 0.1 ohm resistor is present
*/
/**************************************************************************/
void ina219_SetCalibration_32V_640mA(void)
{
  // By default we use a pretty huge range for the input voltage,
  // which probably isn't the most appropriate choice for system
  // that don't use a lot of power.  But all of the calculations
  // are shown below if you want to change the settings.  You will
  // also need to change any relevant register settings, such as
  // setting the VBUS_MAX to 16V instead of 32V, etc.

  // VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
  // VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
  // RSHUNT = 0.5               (Resistor value in ohms)
  
  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 640mA
  
  // 2. Determine max expected current
  // MaxExpected_I = 640mA
  
  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.0000195318              (19uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0,0001563         (156uA per bit)
  
  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.00002 (20uA per bit)
  
  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 4096 (0x1000)
  
	uint16_t ina219_calValue = 4096;
  
  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.0004 (2mW per bit)
  
  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 3.2767A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.32V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If
  
  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 3.2 * 32V
  // MaximumPower = 102.4W
  
  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 50;  // Current LSB = 20uA per bit (1000/20 = 10)
  ina219_powerDivider_mW = 2;     // Power LSB = 1mW per bit (2/1)

  // Set Calibration register to 'Cal' calculated above	
	ina219_WriteRegister(INA219_REG_CALIBRATION, &ina219_calValue);
  
  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	ina219_WriteRegister(INA219_REG_CONFIG, &config);
}


/**************************************************************************/
void ina219_SetCalibration_32V_320mA(void)
{
  uint16_t ina219_calValue = 8192;
  ina219_currentDivider_mA = 100;      // Current LSB = 40uA per bit (1000/40 = 25)
  ina219_powerDivider_mW = 2;         // Power LSB = 800?W per bit

  // Set Calibration register to 'Cal' calculated above	
	ina219_WriteRegister(INA219_REG_CALIBRATION, &ina219_calValue);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_4_160MV |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	ina219_WriteRegister(INA219_REG_CONFIG, &config);
}
void ina219_SetCalibration_16V_80mA(void)
{
	uint16_t ina219_calValue = 40960;
  ina219_currentDivider_mA = 500;  // Current LSB = 2uA per bit (1000/50 = 20)
  ina219_powerDivider_mW = 2;     // Power LSB = 1mW per bit

  // Set Calibration register to 'Cal' calculated above 
	ina219_WriteRegister(INA219_REG_CALIBRATION, &ina219_calValue);
  
  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	ina219_WriteRegister(INA219_REG_CONFIG, &config);
}
/**************************************************************************/
/*! 
    @brief  Setups the HW (defaults to 32V and 2A for calibration values)
*/
/**************************************************************************/
void ina219_begin(void) 
{
  ina219_SetCalibration_16V_80mA();
}

/**************************************************************************/
/*! 
    @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
float ina219_getBusVoltage_V(void)
{
	uint16_t tmp;
	float tmp2;
	ina219_ReadRegister(INA219_REG_BUSVOLTAGE, &tmp);
	tmp2 = (float)tmp * 0.01f;
	return tmp2;
}

/**************************************************************************/
/*! 
    @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
float ina219_getShuntVoltage_mV(void)
{
	uint16_t tmp;
	float tmp2;
	ina219_ReadRegister(INA219_REG_SHUNTVOLTAGE , &tmp);
	tmp2 = (float)tmp * 0.001f;
	return tmp2;
}

/**************************************************************************/
/*! 
    @brief  Gets the raw current value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
float ina219_getCurrent_mA(void)
{
	uint16_t tmp;
	float tmp2;
	ina219_ReadRegister(INA219_REG_CURRENT , &tmp);
	tmp2 = (float)tmp / ina219_currentDivider_mA;
	return tmp2;
}
 



