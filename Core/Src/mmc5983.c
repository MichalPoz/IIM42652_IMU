/*
 * mmc5983.c
 *
 *  Created on: 23 sie 2023
 *      Author: Michal
 */


/* Includes */
#include "mmc5983.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"

mmc_mag_t MMC_MagStructure;

/* */
void MMC_Init(I2C_HandleTypeDef *I2C_Handle)
{
	MMC_MagStructure.I2C_Handle = I2C_Handle;
}

void MMC_WriteRegister(uint8_t *reg, uint8_t *data_out)
{
	uint16_t data_size = sizeof(*data_out);
	HAL_I2C_Mem_Write(MMC_MagStructure.I2C_Handle, MMC_I2C_ADR, *reg, 1, data_out, data_size, 100);
}

void MMC_Status(uint8_t *status)
{
	uint8_t reg = STATUS_REG;
	uint8_t data_size = sizeof(*status);
	HAL_I2C_Mem_Read(MMC_MagStructure.I2C_Handle, MMC_I2C_ADR, reg, 1, status, data_size, 100);
}

void MMC_ProductID(uint8_t *status)
{
	uint8_t reg = PROD_ID_REG;
	uint8_t data_size = sizeof(*status);
	HAL_I2C_Mem_Read(MMC_MagStructure.I2C_Handle, MMC_I2C_ADR_RD, reg, 1, status, data_size, 100);
}

void MMC_GetTemperature(float *data_buf)
{
	// Initialize new measurement
	uint8_t reg_value = TM_T;
	HAL_I2C_Mem_Write(MMC_MagStructure.I2C_Handle, MMC_I2C_ADR_WR, INT_CTRL_0_REG, 1, &reg_value, 1, 100);

	// Check if measurement is done
	uint8_t status_val = 0;
	uint8_t condi = OTP_READ_DONE | MEAS_T_DONE;
	do
	{
		HAL_Delay(1);
		HAL_I2C_Mem_Read(MMC_MagStructure.I2C_Handle, MMC_I2C_ADR_RD, STATUS_REG, 1, &status_val, 1, 100);
	}
	while(status_val != condi);

	// Reading measurement
	uint8_t temp_buf = 0;
	uint8_t data_size = sizeof(temp_buf);
	HAL_I2C_Mem_Read(MMC_MagStructure.I2C_Handle, MMC_I2C_ADR_RD, T_OUT_REG, 1, &temp_buf, data_size, 100);
	// Converting raw temperature to float representation
	*data_buf = (-75.0) + (float)temp_buf * (200.0/255.0);

}
void MMC_SetConfig()
{

}

void MMC_DoSoftReset()
{
	//uint8_t reg_value = SW_RST;
	//HAL_I2C_Mem_Write(MMC_MagStructure.I2C_Handle, MMC_I2C_ADR_WR, INT_CTRL_1_REG, 1, &reg_value, 1, 100);
}

void MMC_SetOperation()
{

}

void MMC_ResetOperation()
{

}

void MMC_GetXYZ()
{
	// Initialize new XYZ measurement
	uint8_t regValue = TM_M;
	HAL_I2C_Mem_Write(MMC_MagStructure.I2C_Handle, MMC_I2C_ADR_WR, INT_CTRL_0_REG, 1, &regValue, 1, 100);

	// Check if measurement is done
	uint8_t statusVal = 0;
	uint8_t condi = OTP_READ_DONE | MEAS_M_DONE;
	do
	{
		HAL_Delay(1);
		HAL_I2C_Mem_Read(MMC_MagStructure.I2C_Handle, MMC_I2C_ADR_RD, STATUS_REG, 1, &statusVal, 1, 100);
	}
	while(statusVal != condi);

	// Reading measurement
	uint8_t tempBuf[7] = {0};
	uint8_t dataSize = sizeof(tempBuf);
	HAL_I2C_Mem_Read(MMC_MagStructure.I2C_Handle, MMC_I2C_ADR_RD, X_OUT_0_REG, 1, tempBuf, dataSize, 100);

	uint32_t tempX_Raw = 0;
	uint32_t tempY_Raw = 0;
	uint32_t tempZ_Raw = 0;

	tempX_Raw = tempBuf[0];										//Xout[17:10]
	tempX_Raw = (tempX_Raw << 8) | tempBuf[1];					//Xout[9:2]
	tempX_Raw = (tempX_Raw << 2) | (tempBuf[6] >> 6);			//Xout[1:0]
	tempY_Raw = tempBuf[2];										//Yout[17:10]
	tempY_Raw = (tempY_Raw << 8) | tempBuf[3];					//Yout[9:2]
	tempY_Raw = (tempY_Raw << 2) | ((tempBuf[6] >> 4) & 0x03);	//Yout[1:0]
	tempZ_Raw = tempBuf[4];										//Zout[17:10]
	tempZ_Raw = (tempZ_Raw << 8) | tempBuf[4];					//Zout[9:2]
	tempZ_Raw = (tempZ_Raw << 2) | ((tempBuf[6] >> 2) & 0x03);	//Zout[1:0]

	MMC_MagStructure.xRaw = tempX_Raw;
	MMC_MagStructure.yRaw = tempY_Raw;
	MMC_MagStructure.zRaw = tempZ_Raw;
}

void MMC_MagFieldConvert()
{
	MMC_MagStructure.xScaled = (((float)MMC_MagStructure.xRaw - 131072.0) / 131072.0) * 8;
	MMC_MagStructure.yScaled = (((float)MMC_MagStructure.yRaw - 131072.0) / 131072.0) * 8;
	MMC_MagStructure.zScaled = (((float)MMC_MagStructure.zRaw - 131072.0) / 131072.0) * 8;
}
