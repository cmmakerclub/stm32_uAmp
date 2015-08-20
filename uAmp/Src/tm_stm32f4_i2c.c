/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#include "tm_stm32f4_i2c.h"


uint8_t TM_I2C_Read(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg) {
	uint8_t received_data;
	TM_I2C_Start(I2Cx, address, I2C_TRANSMITTER_MODE, I2C_ACK_DISABLE);
	TM_I2C_WriteData(I2Cx, reg);
	TM_I2C_Stop(I2Cx);
	TM_I2C_Start(I2Cx, address, I2C_RECEIVER_MODE, I2C_ACK_DISABLE);
	received_data = TM_I2C_ReadNack(I2Cx);
	return received_data;
}


void TM_I2C_ReadMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
	uint8_t i;
	TM_I2C_Start(I2Cx, address, I2C_TRANSMITTER_MODE, I2C_ACK_ENABLE);
	TM_I2C_WriteData(I2Cx, reg);
	TM_I2C_Stop(I2Cx);
	TM_I2C_Start(I2Cx, address, I2C_RECEIVER_MODE, I2C_ACK_ENABLE);
	for (i = 0; i < count; i++) {
		if (i == (count - 1)) {
			/* Last byte */
			data[i] = TM_I2C_ReadNack(I2Cx);
		} else {
			data[i] = TM_I2C_ReadAck(I2Cx);
		}
	}
}

uint8_t TM_I2C_ReadNoRegister(I2C_TypeDef* I2Cx, uint8_t address) {
	uint8_t data;
	TM_I2C_Start(I2Cx, address, I2C_RECEIVER_MODE, I2C_ACK_ENABLE);
	/* Also stop condition happens */
	data = TM_I2C_ReadNack(I2Cx);
	return data;
}

void TM_I2C_ReadMultiNoRegister(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* data, uint16_t count) {
	uint8_t i;
	TM_I2C_Start(I2Cx, address, I2C_RECEIVER_MODE, I2C_ACK_ENABLE);
	for (i = 0; i < count; i++) {
		if (i == (count - 1)) {
			/* Last byte */
			data[i] = TM_I2C_ReadNack(I2Cx);
		} else {
			data[i] = TM_I2C_ReadAck(I2Cx);
		}
	}
}

void TM_I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data) {
	TM_I2C_Start(I2Cx, address, I2C_TRANSMITTER_MODE, I2C_ACK_DISABLE);
	TM_I2C_WriteData(I2Cx, reg);
	TM_I2C_WriteData(I2Cx, data);
	TM_I2C_Stop(I2Cx);
}

void TM_I2C_WriteMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
	uint8_t i;
	TM_I2C_Start(I2Cx, address, I2C_TRANSMITTER_MODE, I2C_ACK_DISABLE);
	TM_I2C_WriteData(I2Cx, reg);
	for (i = 0; i < count; i++) {
		TM_I2C_WriteData(I2Cx, data[i]);
	}
	TM_I2C_Stop(I2Cx);
}

void TM_I2C_WriteNoRegister(I2C_TypeDef* I2Cx, uint8_t address, uint8_t data) {
	TM_I2C_Start(I2Cx, address, I2C_TRANSMITTER_MODE, I2C_ACK_DISABLE);
	TM_I2C_WriteData(I2Cx, data);
	TM_I2C_Stop(I2Cx);
}

void TM_I2C_WriteMultiNoRegister(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* data, uint16_t count) {
	uint8_t i;
	TM_I2C_Start(I2Cx, address, I2C_TRANSMITTER_MODE, I2C_ACK_DISABLE);
	for (i = 0; i < count; i++) {
		TM_I2C_WriteData(I2Cx, data[i]);
	}
	TM_I2C_Stop(I2Cx);
}

/* Private functions */
