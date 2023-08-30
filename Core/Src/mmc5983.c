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

mmc_mag_t mmc_mag_s;

/* */
void mmc_init(I2C_HandleTypeDef *i2c_handler)
{
	mmc_mag_s.i2c_h = i2c_handler;
}

void mmc_write_register(uint8_t *reg, uint8_t *data_out)
{
	uint16_t data_size = sizeof(*data_out);
	HAL_I2C_Mem_Write(mmc_mag_s.i2c_h, MMC_I2C_ADR, *reg, 1, data_out, data_size, 100);
}

void mmc_status(uint8_t *status)
{
	uint8_t reg = STATUS_REG;
	uint8_t data_size = sizeof(*status);
	HAL_I2C_Mem_Read(mmc_mag_s.i2c_h, MMC_I2C_ADR, reg, 1, status, data_size, 100);
}

void mmc_product_id(uint8_t *status)
{
	uint8_t reg = PROD_ID_REG;
	uint8_t data_size = sizeof(*status);
	uint8_t addr_reg = MMC_I2C_ADR | 0x01;
	HAL_I2C_Mem_Read(mmc_mag_s.i2c_h, addr_reg, reg, 1, status, data_size, 100);
}

