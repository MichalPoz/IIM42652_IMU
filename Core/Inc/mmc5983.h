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

typedef struct mmc_mag_s
{
	I2C_HandleTypeDef *i2c_h;

} mmc_mag_t;

void mmc_init(I2C_HandleTypeDef *i2c_handler);
void mmc_write_register(uint8_t *reg, uint8_t *data_out);
void mmc_status(uint8_t *status);
void mmc_product_id(uint8_t *status);

#endif /* INC_MMC5983_H_ */
