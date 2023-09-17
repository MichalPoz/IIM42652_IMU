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

typedef struct MMC_MagStructure
{
	I2C_HandleTypeDef *I2C_Handle;
	uint32_t xRaw;
	uint32_t yRaw;
	uint32_t zRaw;
	float xScaled;
	float yScaled;
	float zScaled;

} mmc_mag_t;

void MMC_Init(I2C_HandleTypeDef *I2C_Handle);
void MMC_WriteRegister(uint8_t *reg, uint8_t *data_out);
void MMC_Status(uint8_t *status);
void MMC_ProductID(uint8_t *status);
void MMC_GetTemperature(float *data_buf);
void MMC_GetXYZ();

#endif /* INC_MMC5983_H_ */
