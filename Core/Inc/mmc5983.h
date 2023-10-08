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

typedef struct MMC_ErrorState
{

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

} mmc_mag_t;


/* TESTED FUNCTIONS */

/* NOT TESTED FUNCTIONS */


void MMC_Init(mmc_mag_t *magStructure, mmc_memory_t *magRegisterState, I2C_HandleTypeDef *I2C_Handle);
void MMC_WriteRegister(mmc_mag_t *magStructure, uint8_t reg, uint8_t *dataToSend);
void MMC_ReadRegister(mmc_mag_t *magStructure, uint8_t reg, uint8_t *readBuffer);
void MMC_Status(mmc_mag_t *magStructure, uint8_t *status);
void MMC_ProductID(mmc_mag_t *magStructure, uint8_t *prodID);
void MMC_GetTemperature(mmc_mag_t *magStructure, float *dataBuf);
void MMC_GetXYZ(mmc_mag_t *magStructure);
void MMC_MagFieldConvert(mmc_mag_t *magStructure);
void MMC_SetShadowRegisterState(mmc_mag_t *magStructure, uint8_t reg, uint8_t *regVal);
void MMC_RemoveBridgeOffset(mmc_mag_t *magStructure);
uint8_t MMC_CheckShadowRegisterState(mmc_mag_t *magStructure, uint8_t reg);

#endif /* INC_MMC5983_H_ */
