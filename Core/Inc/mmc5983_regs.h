/*
 * mmc5983_regs.h
 *
 *  Created on: 23 sie 2023
 *      Author: Michal
 */

#ifndef INC_MMC5983_REGS_H_
#define INC_MMC5983_REGS_H_

/*Device address*/

#define MMC_I2C_ADR 0b0110000 << 1

/*Registers definitions*/

#define X_OUT_0_REG     0x0
#define X_OUT_1_REG     0X01
#define Y_OUT_0_REG     0x02
#define Y_OUT_1_REG     0x03
#define Z_OUT_0_REG     0x04
#define Z_OUT_1_REG     0x05
#define XYZ_OUT_2_REG   0x06
#define T_OUT_REG       0x07
#define STATUS_REG      0x08
#define INT_CTRL_0_REG  0x09
#define INT_CTRL_1_REG  0x0a
#define INT_CTRL_2_REG  0x0b
#define INT_CTRL_3_REG  0x0c
#define PROD_ID_REG     0x2f
#define DUMMY           0x0

#endif /* INC_MMC5983_REGS_H_ */
