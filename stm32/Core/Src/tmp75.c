/*
 * tmp75.c
 *
 *  Created on: 2020/04/25
 *      Author: cronos
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "tmp75.h"
#include "main.h"

I2C_HandleTypeDef hi2c1;

//----------------------------------------------------------------
// Get UnixTime from Sakura.io
//----------------------------------------------------------------
HAL_StatusTypeDef TMP75_INIT(void)
{
    uint8_t Time_Retry = 0;
    uint32_t Time0 = 0, Time1 = 0;

HAL_StatusTypeDef status = HAL_OK;

uint8_t THigh[2], TLow[2], buf;

// ### Safety Limit ###
// FOR GATEDRIVE AUTO CUT
// 120.0 degC / 0.25 = 480
// 480 = 0x1E0, SetData = 0x1E0 << 7
THigh[0] = 0b11110000;	//120degC
//THigh[0] = 0b01010000;	//80degC
THigh[1] = 0b00000000;

// 00.0 degC / 0.25 = 0
//TLow[0] = 0x0;
TLow[0] = 0b01001011;	//75degC  THighを超過した後、TLowまで戻ってこないと再開しない
TLow[1] = 0x0;

buf = TMP75_Config;

//HAL_I2C_Master_Transmit(&TMP75_I2C_HANDLER, TMP75_DEVICE_ADDRESS, (uint8_t*) buffer, 2, TMP75_I2C_TIMEOUT);
status = HAL_I2C_Mem_Write(&hi2c1, TMP75_DEVICE_ADDRESS, TMP75_REG_CONFIG_RW, 1, &buf, 1, TMP75_I2C_TIMEOUT);
if (status != HAL_OK) return status;

status = HAL_I2C_Mem_Write(&hi2c1, TMP75_DEVICE_ADDRESS, TMP75_REG_T_HIGH_RW, 1, &THigh[0], 2, TMP75_I2C_TIMEOUT);
if (status != HAL_OK) return status;

status = HAL_I2C_Mem_Write(&hi2c1, TMP75_DEVICE_ADDRESS, TMP75_REG_T_LOW_RW, 1, &TLow[0], 2, TMP75_I2C_TIMEOUT);
if (status != HAL_OK) return status;
//HAL_I2C_Mem_Read(&TMP75_I2C_HANDLER, TMP75_DEVICE_ADDRESS, TMP75_REG_CONFIG_RW, 1, &ConfigReg[0], 1, TMP75_I2C_TIMEOUT);

// これを打つと変換開始
//HAL_I2C_Mem_Write(&TMP75_I2C_HANDLER, TMP75_DEVICE_ADDRESS, TMP75_REG_CONFIG_RW, 1, TMP75_Config, 1, TMP75_I2C_TIMEOUT);
//HAL_Delay(60);  // 60mSec Later
// で、読む。
//HAL_I2C_Mem_Read(&TMP75_I2C_HANDLER, TMP75_DEVICE_ADDRESS, TMP75_REG_TEMPERATURE_R, 1, &TMP75_Temp[0], 2, TMP75_I2C_TIMEOUT);
//Temp = (TMP75_Temp[0] + TMP75_Temp[1] << 8) >> 6        // 0.25degC / LSB


//HAL_I2C_Mem_Read(&TMP75_I2C_HANDLER, TMP75_DEVICE_ADDRESS, TMP75_REG_ADDRESS, I2C_MEMADD_SIZE_8BIT, data, 3, 1000);
//double temp = (data[0] | data[1] << 8 | data[2] << 16) / 4096.0;

    return HAL_OK;
}

HAL_StatusTypeDef TMP75_STARTACQ(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_I2C_Mem_Write(&hi2c1, TMP75_DEVICE_ADDRESS, TMP75_REG_CONFIG_RW, 1, TMP75_Config, 1, TMP75_I2C_TIMEOUT);

    return status;
}

uint16_t TMP75_GETTEMP(void)
{
    uint8_t TMP75_Temp[2];
    uint16_t buf;
    uint16_t Temp = 0;

    HAL_I2C_Mem_Read(&hi2c1, TMP75_DEVICE_ADDRESS, TMP75_REG_TEMPERATURE_R, 1, &TMP75_Temp[0], 2, TMP75_I2C_TIMEOUT);
//    buf = ((TMP75_Temp[1] + (TMP75_Temp[0] << 8)) >> 6);        // 0.25degC / LSB
    buf = ((TMP75_Temp[1] + (TMP75_Temp[0] << 8)) >> 4);        // 0.25degC / LSB

	if (buf & 0x8000){							// 負温度の場合
		buf = (buf & 0x7FFF) >> 2;
        buf = (0x7FFF ^ buf) + 1;
		Temp = (uint16_t)(buf * -0.25 * 100);		// 温度変換 (0.25℃/LSB)

	} else {									// 正温度の場合
		buf = buf >> 2;
		Temp = (uint16_t)(buf * 0.25 * 100);		// 温度変換 (0.25℃/LSB)
	}

//	HAL_Delay(50);
	TMP75_STARTACQ();

    TMP75_NowTemperature = Temp;
    return Temp;
}
