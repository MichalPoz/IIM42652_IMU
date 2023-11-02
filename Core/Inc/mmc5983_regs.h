/*
 * mmc5983_regs.h
 *
 *  Created on: 23 sie 2023
 *      Author: Michal
 */

#ifndef INC_MMC5983_REGS_H_
#define INC_MMC5983_REGS_H_

/* Device address */
#define MMC_I2C_ADR 	0b0110000 << 1
#define MMC_I2C_ADR_WR 	MMC_I2C_ADR
#define MMC_I2C_ADR_RD 	MMC_I2C_ADR | 0x01

/* Registers definitions */
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

/* Status register values */
#define MEAS_M_DONE 	0x01
#define MEAS_T_DONE 	0x02
#define OTP_READ_DONE	0x10

/* Internal Control 0 register values */
#define TM_M 		0x01
#define TM_T 		0x02
#define INT_M_DONE 	0x04
#define SET_ON 		0x08
#define RESET_ON 	0x10
#define AUTO_SR_EN 	0x20
#define OTP_READ	0x40

/* Internal Control 1 registers values */
#define BW_0				0b00000001
#define BW_1				0b00000010
#define X_DISABLE			0b00000100
#define Y_Z_DISABLE			0b00011000
#define SW_RST				0b10000000

/* Internal Control 2 registers values */
#define CONTINUOUS_0		0b00000001
#define CONTINUOUS_1		0b00000010
#define CONTINUOUS_2		0b00000100
#define CONTINUOUS_ON		0b00001000
#define PERIOD_SET_0		0b00010000
#define PERIOD_SET_1		0b00100000
#define PERIOD_SET_2		0b01000000
#define PERIOD_SET_ON		0b10000000

/* Internal Control 3 registers values */
#define ST_ENP				0b00000010
#define ST_ENM				0b00000100
#define SPI_3_WIRE			0b01000000

#endif /* INC_MMC5983_REGS_H_ */
