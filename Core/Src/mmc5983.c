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


/*************************************************************
 * NOT TESTED FUNCTIONS
 *************************************************************/

void MMC_WriteRegister(mmc_mag_t *magStructure, uint8_t reg, uint8_t *dataToSend, unsigned char writeFlag)
{
	MMC_SetShadowRegisterState(magStructure, reg, dataToSend);

	if (writeFlag)
	{
		uint16_t dataSize = sizeof(*dataToSend);
		HAL_I2C_Mem_Write(magStructure->I2C_Handle, MMC_I2C_ADR_WR, reg, 1, dataToSend, dataSize, 100);
	}

}

void MMC_ReadRegister(mmc_mag_t *magStructure, uint8_t reg, uint8_t *readBuffer)
{
	uint16_t dataSize = sizeof(*readBuffer);
	HAL_I2C_Mem_Read(magStructure->I2C_Handle, MMC_I2C_ADR_RD, reg, 1, readBuffer, dataSize, 100);
}

void MMC_SetShadowRegisterState(mmc_mag_t *magStructure, uint8_t reg, uint8_t *regVal)
{
	// DODANIE FUNKCJI OBSLUGI BLEDOW - zapisanie juz zapisanej wartosci
	switch(reg)
	{
	case INT_CTRL_0_REG:
	{
		magStructure->registerState->internalControl0 = *regVal;
	} break;

	case INT_CTRL_1_REG:
	{
		magStructure->registerState->internalControl1 = *regVal;
	} break;

	case INT_CTRL_2_REG:
	{
		magStructure->registerState->internalControl2 = *regVal;
	} break;

	case INT_CTRL_3_REG:
	{
		magStructure->registerState->internalControl3 = *regVal;
	} break;

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
	//uint8_t regVal = TM_T;
	//MMC_WriteRegister(magStructure, INT_CTRL_0_REG, &regVal);
	MMC_SetBit(magStructure, INT_CTRL_0_REG, TM_T, 1);

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
	//regVal &= ~TM_T;
	//MMC_SetShadowRegisterState(magStructure, INT_CTRL_0_REG, &regVal);
	MMC_ClearBit(magStructure, INT_CTRL_0_REG, TM_T, 0);

	// Reading measurement
	uint8_t tempBuf = 0;
	MMC_ReadRegister(magStructure, T_OUT_REG, &tempBuf);

	// Converting raw temperature to float representation
	*dataBuf = (-75.0) + (float)tempBuf * (200.0/255.0);
}

void MMC_GetXYZ(mmc_mag_t *magStructure)
{
	// Initialize new XYZ measurement
	MMC_SetBit(magStructure, INT_CTRL_0_REG, TM_M, 1);
	//MMC_WriteRegister(magStructure, INT_CTRL_0_REG, &regVal);

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
	//regVal &= ~TM_M;
	//MMC_SetShadowRegisterState(magStructure, INT_CTRL_0_REG, &regVal);
	MMC_ClearBit(magStructure, INT_CTRL_0_REG, TM_M, 0);

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

void MMC_SetOperation(mmc_mag_t *magStructure)
{
	//uint8_t regVal = SET_ON;
	//MMC_WriteRegister(magStructure, INT_CTRL_0_REG, &regVal);
	MMC_SetBit(magStructure, INT_CTRL_0_REG, SET_ON, 1);

	// Clearing register state value in shadow memory only
	//regVal &= ~SET_ON;
	MMC_ClearBit(magStructure, INT_CTRL_0_REG, SET_ON, 0);
	//MMC_SetShadowRegisterState(magStructure, INT_CTRL_0_REG, &regVal);
	HAL_Delay(1);
}

void MMC_ResetOperation(mmc_mag_t *magStructure)
{
	//uint8_t regVal = RESET_ON;
	//MMC_WriteRegister(magStructure, INT_CTRL_0_REG, &regVal);
	MMC_SetBit(magStructure, INT_CTRL_0_REG, RESET_ON, 1);

	// Clearing register state value in shadow memory only
	//regVal &= ~RESET_ON;
	//MMC_SetShadowRegisterState(magStructure, INT_CTRL_0_REG, &regVal);
	MMC_ClearBit(magStructure, INT_CTRL_0_REG, SET_ON, 0);
	HAL_Delay(1);
}

void MMC_SetBit(mmc_mag_t *magStructure, uint8_t internalReg, uint8_t regVal, unsigned char writeFlag)
{
	uint8_t internalRegState = MMC_CheckShadowRegisterState(magStructure, internalReg);
	uint8_t newInternalRegState = internalRegState | regVal;
	MMC_WriteRegister(magStructure, internalReg, &newInternalRegState, writeFlag);
}

void MMC_ClearBit(mmc_mag_t *magStructure, uint8_t internalReg, uint8_t regVal, unsigned char writeFlag)
{
	uint8_t internalRegState = MMC_CheckShadowRegisterState(magStructure, internalReg);
	uint8_t newInternalRegState = internalRegState & (~regVal);
	MMC_WriteRegister(magStructure, internalReg, &newInternalRegState, writeFlag);
}

uint8_t MMC_CheckShadowRegisterState(mmc_mag_t *magStructure, uint8_t internalReg) // Checked, not tested
{
	uint8_t shadowRegister = 0x0;

	switch(internalReg)
	{
	case INT_CTRL_0_REG:
	{
		shadowRegister = magStructure->registerState->internalControl0;
	} break;

	case INT_CTRL_1_REG:
	{
		shadowRegister = magStructure->registerState->internalControl1;
	} break;

	case INT_CTRL_2_REG:
	{

		shadowRegister = magStructure->registerState->internalControl2;
	} break;

	case INT_CTRL_3_REG:
	{

		shadowRegister = magStructure->registerState->internalControl3;
	} break;

	default:
		break;
	}

	return shadowRegister;
}


void MMC_Init(mmc_mag_t *magStructure, mmc_memory_t *magRegisterState, mmc_error_t *magErrorState, mmc_config_t *magConfigState, I2C_HandleTypeDef *I2C_Handle)
{
	magStructure->I2C_Handle = I2C_Handle;
	magStructure->registerState = magRegisterState;
	magStructure->errorState = magErrorState;

	// Default Register States
	magStructure->registerState->internalControl0 = 0x0;
	magStructure->registerState->internalControl1 = 0x0;
	magStructure->registerState->internalControl2 = 0x0;
	magStructure->registerState->internalControl3 = 0x0;

	// Zainicjowanie zmiennych
	magStructure->xOffset = 131072;
	magStructure->yOffset = 131072;
	magStructure->zOffset = 131072;

	// Default Configuration
	magStructure->configState->bandwidthFreq = 100;
	magStructure->configState->continuousFreq = 0;
	magStructure->configState->setPeriod = 1;

}

void MMC_SetConfig(mmc_mag_t *magStructure)
{
	// Default configuration
	uint16_t bandwidthFrequency = 100;
	uint16_t continuousFrequency = 0;
	uint16_t setPeriod = 100;

	MMC_SetBandwidth(magStructure, bandwidthFrequency);
	MMC_SetContinuousFreq(magStructure, continuousFrequency);
	MMC_SetSetOperationPeriod(magStructure, setPeriod);
}

void MMC_DoSoftReset(mmc_mag_t *magStructure) // Checked, not tested
{
	MMC_SetBit(magStructure, INT_CTRL_1_REG, SW_RST, 1);
	HAL_Delay(50);
}

void MMC_EnableInterrput(mmc_mag_t *magStructure) // Checked, not tested
{
	MMC_SetBit(magStructure, INT_CTRL_0_REG, INT_M_DONE, 1);
}

void MMC_DisableInterrupt(mmc_mag_t *magStructure)
{
	uint8_t regVal = MMC_CheckShadowRegisterState(magStructure, INT_CTRL_0_REG);
	uint8_t compareVal = regVal & INT_M_DONE;
	if (compareVal == INT_M_DONE)
	{
		MMC_ClearBit(magStructure, INT_CTRL_0_REG, INT_M_DONE, 1);
	}
	else
	{
		//ERROR HANDLING -
	}
}

void MMC_EnableAutoSetReset(mmc_mag_t *magStructure)
{
	MMC_SetBit(magStructure, INT_CTRL_0_REG, AUTO_SR_EN, 1);
}

void MMC_DisableAutoSetReset(mmc_mag_t *magStructure)
{
	uint8_t regVal = MMC_CheckShadowRegisterState(magStructure, INT_CTRL_0_REG);
	uint8_t compareVal = regVal & AUTO_SR_EN;
	if (compareVal == AUTO_SR_EN)
	{
		MMC_ClearBit(magStructure, INT_CTRL_0_REG, AUTO_SR_EN, 1);
	}
	else
	{
		//ERROR HANDLING
	}
}

void MMC_EnableXChannel(mmc_mag_t *magStructure)
{
	uint8_t regVal = MMC_CheckShadowRegisterState(magStructure, INT_CTRL_1_REG);
	uint8_t compareVal = regVal & X_DISABLE;
	if (compareVal == X_DISABLE)
	{
		MMC_ClearBit(magStructure, INT_CTRL_1_REG, X_DISABLE, 1);
	}
	else
	{
		//ERROR HANDLING
	}
}

void MMC_DisableXChannel(mmc_mag_t *magStructure)
{
	MMC_SetBit(magStructure, INT_CTRL_1_REG, X_DISABLE, 1);
}

void MMC_EnableYZChannel(mmc_mag_t *magStructure)
{
	uint8_t regVal = MMC_CheckShadowRegisterState(magStructure, INT_CTRL_1_REG);
	uint8_t compareVal = regVal & Y_Z_DISABLE;
	if (compareVal == Y_Z_DISABLE)
	{
		MMC_ClearBit(magStructure, INT_CTRL_1_REG, Y_Z_DISABLE, 1);
	}
	else
	{
		//ERROR HANDLING
	}
}

void MMC_DisableYZChannel(mmc_mag_t *magStructure)
{
	MMC_SetBit(magStructure, INT_CTRL_1_REG, Y_Z_DISABLE, 1);
}

void MMC_SetBandwidth(mmc_mag_t *magStructure, uint16_t bandwidth)
{
	//Dzialanie funkcji jest niezalezne od innych rejestrow stad mozna spokojnie ustawic wybrana wartosc
	switch(bandwidth)
	{
	case 800:
	{
		MMC_ClearBit(magStructure, INT_CTRL_1_REG, BW_1, 0);
		MMC_ClearBit(magStructure, INT_CTRL_1_REG, BW_0, 1);
		magStructure->configState->bandwidthFreq = bandwidth;
	} break;

	case 400:
	{
		MMC_ClearBit(magStructure, INT_CTRL_1_REG, BW_1, 0);
		MMC_SetBit(magStructure, INT_CTRL_1_REG, BW_0, 1);
		magStructure->configState->bandwidthFreq = bandwidth;
	} break;

	case 200:
	{
		MMC_SetBit(magStructure, INT_CTRL_1_REG, BW_1, 0);
		MMC_ClearBit(magStructure, INT_CTRL_1_REG, BW_0, 1);
		magStructure->configState->bandwidthFreq = bandwidth;
	} break;

	case 100:
	{
		MMC_SetBit(magStructure, INT_CTRL_1_REG, BW_1, 0);
		MMC_SetBit(magStructure, INT_CTRL_1_REG, BW_0, 1);
		magStructure->configState->bandwidthFreq = bandwidth;
	} break;

	default:
	{
		// Add default action - information about wrong bandwidth - error message
	} break;

	}
}

uint16_t MMC_GetBandwidth(mmc_mag_t *magStructure)
{
	return magStructure->configState->bandwidthFreq;
}

void MMC_EnableContinuousMode(mmc_mag_t *magStructure)
{
	uint8_t stateVal = MMC_CheckShadowRegisterState(magStructure, INT_CTRL_2_REG);
	uint8_t compareVal = stateVal >> 5;
	if (compareVal != 0b00000000)
	{
		MMC_SetBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_ON, 1);
	}
	else
	{
		//ERROR HANDLING
	}
}

void MMC_DisableContinuousMode(mmc_mag_t *magStructure)
{
	uint8_t stateVal = MMC_CheckShadowRegisterState(magStructure, INT_CTRL_2_REG);
	uint8_t compareVal = stateVal & CONTINUOUS_ON;
	if (compareVal != 0b00000000)
	{
		MMC_ClearBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_ON, 1);
	}
	else
	{
		//ERROR HANDLING
	}
}

void MMC_SetContinuousFreq(mmc_mag_t *magStructure, uint16_t frequency)
{
	switch(frequency)
	{
	case 1:
	{
		// 001 State
		MMC_ClearBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_2, 0);
		MMC_ClearBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_1, 0);
		MMC_SetBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_0, 1);
		magStructure->configState->continuousFreq = frequency;
	} break;

	case 10:
	{
		// 010 State
		MMC_ClearBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_2, 0);
		MMC_SetBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_1, 0);
		MMC_ClearBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_0, 1);
		magStructure->configState->continuousFreq = frequency;
	} break;

	case 20:
	{
		// 011 State
		MMC_ClearBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_2, 0);
		MMC_SetBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_1, 0);
		MMC_SetBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_0, 1);
		magStructure->configState->continuousFreq = frequency;
	} break;

	case 50:
	{
		// 100 State
		MMC_SetBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_2, 0);
		MMC_ClearBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_1, 0);
		MMC_ClearBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_0, 1);
		magStructure->configState->continuousFreq = frequency;
	} break;

	case 100:
	{
		// 101 State
		MMC_SetBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_2, 0);
		MMC_ClearBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_1, 0);
		MMC_SetBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_0, 1);
		magStructure->configState->continuousFreq = frequency;
	} break;

	case 200:
	{
		if (MMC_GetBandwidth(magStructure) == 200)
		{
			// 110
			MMC_SetBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_2, 0);
			MMC_SetBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_1, 0);
			MMC_ClearBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_0, 1);
			magStructure->configState->continuousFreq = frequency;
		}
		else
		{
			//ERROR HANDLING - WRONG BANDWIDTH VALUE FOR THIS CONFIGURATION
		}

	} break;

	case 1000:
	{
		if (MMC_GetBandwidth(magStructure) == 1000)
		{
			// 111
			MMC_SetBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_2, 0);
			MMC_SetBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_1, 0);
			MMC_SetBit(magStructure, INT_CTRL_2_REG, CONTINUOUS_0, 1);
			magStructure->configState->continuousFreq = frequency;
		}
		else
		{
			//ERROR HANDLING - WRONG BANDWIDTH VALUE FOR THIS CONFIGURATION
		}

	} break;

	default:
	{
		break;//Zwrocenie infomracji o zlej wartosc
	}

	}
}

uint16_t MMC_GetContinuousFreq(mmc_mag_t *magStructure)
{
	return magStructure->configState->continuousFreq;
}

void MMC_EnablePeriodicSet(mmc_mag_t *magStructure)
{

	uint8_t stateVal_1 = MMC_CheckShadowRegisterState(magStructure, INT_CTRL_0_REG);
	uint8_t stateVal_2 = MMC_CheckShadowRegisterState(magStructure, INT_CTRL_2_REG);
	uint8_t compareVal = (stateVal_1 | stateVal_2);	// temporary value
	if (compareVal == 0b00101000)
	{
		MMC_SetBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_ON, 1);
	}
	else
	{
		//Periodic Set Error Identifier
	}

}

void MMC_DisablePeriodicSet(mmc_mag_t *magStructure)
{
	MMC_ClearBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_ON, 1);
}

void MMC_SetSetOperationPeriod(mmc_mag_t *magStructure, uint16_t setPeriod)
{
	// sprawdzenie czy periodic set jest ustawione
	uint8_t stateVal = MMC_CheckShadowRegisterState(magStructure, INT_CTRL_2_REG);
	uint8_t compareVal = (stateVal & PERIOD_SET_ON);
	if (compareVal == PERIOD_SET_ON)
	{
		switch(setPeriod)
		{
			case 1:
			{
				// 000
				MMC_ClearBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_2, 0);
				MMC_ClearBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_1, 0);
				MMC_ClearBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_0, 1);
				magStructure->configState->setPeriod = setPeriod;
			} break;

			case 25:
			{
				// 001
				MMC_ClearBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_2, 0);
				MMC_ClearBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_1, 0);
				MMC_SetBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_0, 1);
				magStructure->configState->setPeriod = setPeriod;
			} break;

			case 75:
			{
				// 010
				MMC_ClearBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_2, 0);
				MMC_SetBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_1, 0);
				MMC_ClearBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_0, 1);
				magStructure->configState->setPeriod = setPeriod;
			} break;

			case 100:
			{
				// 011
				MMC_ClearBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_2, 0);
				MMC_SetBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_1, 0);
				MMC_SetBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_0, 1);
				magStructure->configState->setPeriod = setPeriod;
			} break;

			case 250:
			{
				// 100
				MMC_SetBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_2, 0);
				MMC_ClearBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_1, 0);
				MMC_ClearBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_0, 1);
				magStructure->configState->setPeriod = setPeriod;
			} break;

			case 500:
			{
				// 101
				MMC_SetBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_2, 0);
				MMC_ClearBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_1, 0);
				MMC_SetBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_0, 1);
				magStructure->configState->setPeriod = setPeriod;
			} break;

			case 1000:
			{
				// 110
				MMC_SetBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_2, 0);
				MMC_SetBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_1, 0);
				MMC_ClearBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_0, 1);
				magStructure->configState->setPeriod = setPeriod;
			} break;

			case 2000:
			{
				// 111
				MMC_SetBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_2, 0);
				MMC_SetBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_1, 0);
				MMC_SetBit(magStructure, INT_CTRL_2_REG, PERIOD_SET_0, 1);
				magStructure->configState->setPeriod = setPeriod;
			} break;

			default:
			{
				//ERROR
			} break;
		}
	}
	else
	{
		//ERROR HANDLING - trzeba ustawic enable period set
	}
}

uint16_t MMC_GetSetOperationPeriod(mmc_mag_t *magStructure)
{
	return magStructure->configState->setPeriod;
}

void MMC_ApplyExtraCurrentPosToNeg(mmc_mag_t *magStructure)
{
	MMC_SetBit(magStructure, INT_CTRL_3_REG, ST_ENP, 1);
}

void MMC_RemoveExtraCurrentPosToNeg(mmc_mag_t *magStructure)
{
	MMC_ClearBit(magStructure, INT_CTRL_3_REG, ST_ENP, 1);
}

void MMC_ApplyExtraCurrentNegToPos(mmc_mag_t *magStructure)
{
	MMC_SetBit(magStructure, INT_CTRL_3_REG, ST_ENM, 1);
}

void MMC_RemoveExtraCurrentNegToPos(mmc_mag_t *magStructure)
{
	MMC_ClearBit(magStructure, INT_CTRL_3_REG, ST_ENM, 1);
}

void MMC_Enable3WireSPI(mmc_mag_t *magStructure)
{
	MMC_SetBit(magStructure, INT_CTRL_3_REG, SPI_3_WIRE, 1);
}

void MMC_Disable3WireSPI(mmc_mag_t *magStructure)
{
	MMC_ClearBit(magStructure, INT_CTRL_3_REG, SPI_3_WIRE, 1);
}
