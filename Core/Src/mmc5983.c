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

/* */


/* TESTED FUNCTIONS */

void MMC_WriteRegister(mmc_mag_t *magStructure, uint8_t reg, uint8_t *dataToSend)
{
	uint16_t dataSize = sizeof(*dataToSend);
	HAL_I2C_Mem_Write(magStructure->I2C_Handle, MMC_I2C_ADR_WR, reg, 1, dataToSend, dataSize, 100);
	MMC_SetShadowRegisterState(magStructure, reg, dataToSend);
}

void MMC_ReadRegister(mmc_mag_t *magStructure, uint8_t reg, uint8_t *readBuffer)
{
	uint16_t dataSize = sizeof(*readBuffer);
	HAL_I2C_Mem_Read(magStructure->I2C_Handle, MMC_I2C_ADR_RD, reg, 1, readBuffer, dataSize, 100);
}

void MMC_SetShadowRegisterState(mmc_mag_t *magStructure, uint8_t reg, uint8_t *regVal)
{
	switch(reg)
	{
	case INT_CTRL_0_REG:
	{
		magStructure->registerState->internalControl0 = *regVal;
	}
	break;

	case INT_CTRL_1_REG:
	{
		magStructure->registerState->internalControl1 = *regVal;
	}
	break;

	case INT_CTRL_2_REG:
	{
		magStructure->registerState->internalControl2 = *regVal;
	}
	break;

	case INT_CTRL_3_REG:
	{
		magStructure->registerState->internalControl3 = *regVal;
	}
	break;

	default:
		break;
	}
}

void MMC_Status(mmc_mag_t *magStructure, uint8_t *status)
{
	MMC_ReadRegister(magStructure, STATUS_REG, status);
}

void MMC_ProductID(mmc_mag_t *magStructure, uint8_t *prodID)
{
	MMC_ReadRegister(magStructure, PROD_ID_REG, prodID);
}

void MMC_GetTemperature(mmc_mag_t *magStructure, float *dataBuf)
{
	// Initialize new measurement
	uint8_t regVal = TM_T;
	MMC_WriteRegister(magStructure, INT_CTRL_0_REG, &regVal);

	// Check if measurement is done
	uint8_t statusVal = 0;
	uint8_t condi = OTP_READ_DONE | MEAS_T_DONE;
	do
	{
		HAL_Delay(1);
		MMC_ReadRegister(magStructure, STATUS_REG, &statusVal);
	}
	while(statusVal != condi);

	// Clearing Shadow Memory TM_T - manually
	regVal &= ~TM_T;
	MMC_SetShadowRegisterState(magStructure, INT_CTRL_0_REG, &regVal);

	// Reading measurement
	uint8_t tempBuf = 0;
	MMC_ReadRegister(magStructure, T_OUT_REG, &tempBuf);

	// Converting raw temperature to float representation
	*dataBuf = (-75.0) + (float)tempBuf * (200.0/255.0);
}

void MMC_GetXYZ(mmc_mag_t *magStructure)
{
	// Initialize new XYZ measurement
	uint8_t regVal = TM_M;
	MMC_WriteRegister(magStructure, INT_CTRL_0_REG, &regVal);

	// Check if measurement is done
	uint8_t statusVal = 0;
	uint8_t condi = OTP_READ_DONE | MEAS_M_DONE;
	do
	{
		HAL_Delay(1);
		MMC_ReadRegister(magStructure, STATUS_REG, &statusVal);
	}
	while(statusVal != condi);

	// Clearing Shadow Memory TM_M - manually
	regVal &= ~TM_M;
	MMC_SetShadowRegisterState(magStructure, INT_CTRL_0_REG, &regVal);

	// Reading measurement
	uint8_t tempBuf[7] = {0};
	uint8_t dataSize = sizeof(tempBuf);
	HAL_I2C_Mem_Read(magStructure->I2C_Handle, MMC_I2C_ADR_RD, X_OUT_0_REG, 1, tempBuf, dataSize, 100);

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

	magStructure->xRaw = tempX_Raw;
	magStructure->yRaw = tempY_Raw;
	magStructure->zRaw = tempZ_Raw;
}

void MMC_MagFieldConvert(mmc_mag_t *magStructure)
{
	magStructure->xScaled = (((float)magStructure->xRaw - (float)magStructure->xOffset) / 131072.0) * 8;
	magStructure->yScaled = (((float)magStructure->yRaw - (float)magStructure->yOffset) / 131072.0) * 8;
	magStructure->zScaled = (((float)magStructure->zRaw - (float)magStructure->zOffset) / 131072.0) * 8;
}

/* NOT TESTED FUNCTIONS */

uint8_t MMC_CheckShadowRegisterState(mmc_mag_t *magStructure, uint8_t reg)
{
	uint8_t shadowRegister = NULL;

	switch(reg)
	{
	case INT_CTRL_0_REG:
	{
		shadowRegister = magStructure->registerState->internalControl0;
	}
	break;

	case INT_CTRL_1_REG:
	{
		shadowRegister = magStructure->registerState->internalControl1;
	}
	break;

	case INT_CTRL_2_REG:
	{

		shadowRegister = magStructure->registerState->internalControl2;
	}
	break;

	case INT_CTRL_3_REG:
	{

		shadowRegister = magStructure->registerState->internalControl3;
	}
	break;

	default:
		break;
	}

	return shadowRegister;
}


void MMC_Init(mmc_mag_t *magStructure, mmc_memory_t *magRegisterState, I2C_HandleTypeDef *I2C_Handle)
{
	magStructure->I2C_Handle = I2C_Handle;
	magStructure->registerState = magRegisterState;


	// Zainicjowanie zmiennych
	magStructure->xOffset = 131072;
	magStructure->yOffset = 131072;
	magStructure->zOffset = 131072;

	// Wybor configu itd itp
	// Sprawdzenie polaczenia - odczyt ID

}

void MMC_SetConfig()
{

}

void MMC_DoSoftReset(mmc_mag_t *magStructure)
{
	uint8_t regVal = SW_RST;
	MMC_WriteRegister(magStructure, INT_CTRL_1_REG, &regVal);
	HAL_Delay(50);
}

void MMC_EnableInterrput(mmc_mag_t *magStructure)
{
	uint8_t regVal = INT_M_DONE;
	MMC_WriteRegister(magStructure, INT_CTRL_0_REG, &regVal);
}

void MMC_DisableInterrupt(mmc_mag_t *magStructure)
{
	uint8_t regVal = MMC_CheckShadowRegisterState(magStructure, INT_CTRL_0_REG);
	if (regVal == INT_M_DONE)
	{
		regVal &= ~INT_M_DONE;
		MMC_WriteRegister(magStructure, INT_CTRL_0_REG, &regVal);
	}
	else
	{
		//ERROR HANDLING
	}
}

void MMC_EnableAutoSetReset(mmc_mag_t *magStructure)
{
	uint8_t regVal = AUTO_SR_EN;
	MMC_WriteRegister(magStructure, INT_CTRL_0_REG, &regVal);
}

void MMC_DisableAutoSetReset(mmc_mag_t *magStructure)
{
	uint8_t regVal = MMC_CheckShadowRegisterState(magStructure, INT_CTRL_0_REG);
	if (regVal == AUTO_SR_EN)
	{
		regVal &= ~AUTO_SR_EN;
		MMC_WriteRegister(magStructure, INT_CTRL_0_REG, &regVal);
	}
	else
	{
		//ERROR HANDLING
	}
}

void MMC_EnableXChannel(mmc_mag_t *magStructure)
{
	uint8_t regVal = MMC_CheckShadowRegisterState(magStructure, INT_CTRL_1_REG);
	if (regVal == X_DISABLE)
	{
		regVal &= ~X_DISABLE;
		MMC_WriteRegister(magStructure, INT_CTRL_1_REG, &regVal);
	}
	else
	{
		//ERROR HANDLING
	}
}

void MMC_DisableXChannel(mmc_mag_t *magStructure)
{
	uint8_t regVal = X_DISABLE;
	MMC_WriteRegister(magStructure, INT_CTRL_1_REG, &regVal);
}

void MMC_EnableYZChannel(mmc_mag_t *magStructure)
{
	uint8_t regVal = MMC_CheckShadowRegisterState(magStructure, INT_CTRL_1_REG);
	if (regVal == Y_Z_DISABLE)
	{
		regVal &= ~Y_Z_DISABLE;
		MMC_WriteRegister(magStructure, INT_CTRL_1_REG, &regVal);
	}
	else
	{
		//ERROR HANDLING
	}
}

void MMC_DisableYZChannel(mmc_mag_t *magStructure)
{
	uint8_t regVal = Y_Z_DISABLE;
	MMC_WriteRegister(magStructure, INT_CTRL_1_REG, &regVal);
}

void MMC_SetOperation(mmc_mag_t *magStructure)
{
	uint8_t regVal = SET_ON;
	MMC_WriteRegister(magStructure, INT_CTRL_0_REG, &regVal);

	// Clearing register state value - manually
	regVal &= ~SET_ON;
	MMC_SetShadowRegisterState(magStructure, INT_CTRL_0_REG, &regVal);
	HAL_Delay(1);
}

void MMC_ResetOperation(mmc_mag_t *magStructure)
{
	uint8_t regVal = RESET_ON;
	MMC_WriteRegister(magStructure, INT_CTRL_0_REG, &regVal);

	// Clearing register state value - manually
	regVal &= ~RESET_ON;
	MMC_SetShadowRegisterState(magStructure, INT_CTRL_0_REG, &regVal);
	HAL_Delay(1);
}

void MMC_RemoveBridgeOffset(mmc_mag_t *magStructure)
{
	uint32_t setOutX = 0;
	uint32_t setOutY = 0;
	uint32_t setOutZ = 0;
	uint32_t resetOutX = 0;
	uint32_t resetOutY = 0;
	uint32_t resetOutZ = 0;

	// Perform SET operation
	MMC_SetOperation(magStructure);

	// Perform Measurement
	MMC_GetXYZ(magStructure);
	setOutX = magStructure->xRaw;
	setOutY = magStructure->yRaw;
	setOutZ = magStructure->zRaw;

	// Perform RESET operation
	MMC_ResetOperation(magStructure);

	// Perform Measurement
	resetOutX = magStructure->xRaw;
	resetOutY = magStructure->yRaw;
	resetOutZ = magStructure->zRaw;

	// Computing Offset Value
	magStructure->xOffset = (setOutX + resetOutX) / 2;
	magStructure->yOffset = (setOutY + resetOutY) / 2;
	magStructure->zOffset = (setOutZ + resetOutZ) / 2;
}

void MMC_SetBandwidth(mmc_mag_t *magStructure, uint8_t bandwidth)
{
	switch(bandwidth)
	{
	case 800:
	{
		MMC_WriteRegister(magStructure, INT_CTRL_1_REG, MEAS_05ms_BW_800);
	}
	break;

	case 400:
	{
		MMC_WriteRegister(magStructure, INT_CTRL_1_REG, MEAS_2ms_BW_400);
	}
	break;

	case 200:
	{
		MMC_WriteRegister(magStructure, INT_CTRL_1_REG, MEAS_4ms_BW_200);
	}
	break;

	case 100:
	{
		MMC_WriteRegister(magStructure, INT_CTRL_1_REG, MEAS_8ms_BW_100);
	}
	break;

	default:
	{
		// Add default action
	}
	break;
	}

}

void MMC_SetContinuousFreq(mmc_mag_t *magStructure, uint8_t frequency)
{
	switch(frequency)
	{
	case 0:
	{

	}
	break;

	case 1:
	{

	}
	break;

	case 10:
	{

	}
	break;

	case 20:
	{

	}
	break;

	case 50:
	{

	}
	break;

	case 100:
	{

	}
	break;

	case 200:
	{

	}
	break;

	case 1000:
	{

	}
	break;
	}
}
/* TO DO
 * - Setting Periodic Set Operation
 * - Setting Continuous Mode Frequency
 * - Struct or sturct members to store config states
 * - Interrupt handling
 */
