/*
 * mmc5983.h
 *
 *  Created on: 23 sie 2023
 *      Author: Michal
 */

#ifndef INC_MMC5983_H_
#define INC_MMC5983_H_

/* Includes */
#include "mmc5983_regs.h"
#include "main.h"
#include "spi.h"

typedef struct MMC_RegisterState
{
    uint8_t internalControl0;
    uint8_t internalControl1;
    uint8_t internalControl2;
    uint8_t internalControl3;
} mmc_memory_t;

typedef struct MMC_ConfigState
{
	// Bandwidth
	uint16_t bandwidthFreq;
	// Frequency of Continuous Mode
	uint16_t continuousFreq;
	// Set operation Period
	uint16_t setPeriod;

} mmc_config_t;

typedef enum {
	NONE,
	MMC_I2C_ERROR,
	// Do dokończenia
} mmc_error_t;

typedef struct MMC_MagStructure
{
	I2C_HandleTypeDef *I2C_Handle;
	uint32_t xRaw;
	uint32_t yRaw;
	uint32_t zRaw;
	float xScaled;
	float yScaled;
	float zScaled;
	uint32_t xOffset;
	uint32_t yOffset;
	uint32_t zOffset;
	mmc_memory_t *registerState;
	mmc_error_t *errorState;
	mmc_config_t *configState;

} mmc_mag_t;


/* TESTED FUNCTIONS */
void MMC_Init(mmc_mag_t *magStructure, mmc_memory_t *magRegisterState, mmc_error_t *magErrorState, mmc_config_t *magConfigState, I2C_HandleTypeDef *I2C_Handle);

void MMC_WriteRegister(mmc_mag_t *magStructure, uint8_t reg, uint8_t *dataToSend, unsigned char writeFlag);

void MMC_ReadRegister(mmc_mag_t *magStructure, uint8_t internalReg, uint8_t *readBuffer);

void MMC_SetBit(mmc_mag_t *magStructure, uint8_t internalReg, uint8_t regVal, unsigned char writeFlag);

void MMC_ClearBit(mmc_mag_t *magStructure, uint8_t internalReg, uint8_t regVal, unsigned char writeFlag);

void MMC_Status(mmc_mag_t *magStructure, uint8_t *status);

void MMC_ProductID(mmc_mag_t *magStructure, uint8_t *prodID);

void MMC_GetTemperature(mmc_mag_t *magStructure, float *dataBuf);

/* NOT TESTED FUNCTIONS */

void MMC_SetConfig(mmc_mag_t *magStructure, uint16_t bandwidthFrequency, uint16_t continuousFrequency, uint16_t setPeriod);

void MMC_GetXYZ(mmc_mag_t *magStructure);

void MMC_MagFieldConvert(mmc_mag_t *magStructure);

void MMC_SetShadowRegisterState(mmc_mag_t *magStructure, uint8_t internalReg, uint8_t *regVal);

void MMC_SetOperation(mmc_mag_t *magStructure);

void MMC_ResetOperation(mmc_mag_t *magStructure);

void MMC_SetSetOperationPeriod(mmc_mag_t *magStructure, uint16_t setPeriod);

uint16_t MMC_GetSetOperationPeriod(mmc_mag_t *magStructure);

void MMC_RemoveBridgeOffset(mmc_mag_t *magStructure);

void MMC_SetBandwidth(mmc_mag_t *magStructure, uint16_t bandwidth);

uint16_t MMC_GetBandwidth(mmc_mag_t *magStrucuture);

void MMC_SetContinuousFreq(mmc_mag_t *magStructure, uint16_t frequency);

uint16_t MMC_GetContinuousFreq(mmc_mag_t *magStructure);

void MMC_DisableXChannel(mmc_mag_t *magStructure);

void MMC_EnableXChannel(mmc_mag_t *magStructure);

void MMC_DisableYZChannel(mmc_mag_t *magStructure);

void MMC_EnableYZChannel(mmc_mag_t *magStructure);

void MMC_ApplyExtraCurrentPosToNeg(mmc_mag_t *magStructure);

void MMC_RemoveExtraCurrentPosToNeg(mmc_mag_t *magStructure);

void MMC_ApplyExtraCurrentNegToPos(mmc_mag_t *magStructure);

void MMC_RemoveExtraCurrentNegToPos(mmc_mag_t *magStructure);

void MMC_Enable3WireSPI(mmc_mag_t *magStructure);

void MMC_Disable3WireSPI(mmc_mag_t *magStructure);

uint8_t MMC_CheckShadowRegisterState(mmc_mag_t *magStructure, uint8_t reg);

#endif /* INC_MMC5983_H_ */
