/**
 * @file iim42652.h
 * @author Michał Słomiany (m.slomiany@outlook.com) 
 * @brief
 * @version 0.1
 * @date 2023-01-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef IIM42652_H
#define IIM42652_H

/* Includes */
#include "iim42652_regs.h"
#include "main.h"
#include "spi.h"

/* Configuration options */
#define TEMP_DISABLE 	0b00100000
#define IDLE 		0b00010000
#define GYRO_STB	0b00000100
#define GYRO_LN		0b00001100
#define ACC_LP		0b00000010
#define ACC_LN		0b00000011
#define PWR_DEFAULT 0b00000000

typedef struct IIM_ConfigState
{
	SPI_HandleTypeDef *spi_h;
	uint8_t accel_fs_sel;
	uint8_t accel_odr;
	uint8_t accel_ui_filt_bw;
	uint8_t accel_ui_filt_ord;
	uint8_t accel_dec2_m2_ord;
	uint8_t gyro_fs_sel;
	uint8_t gyro_odr;
	uint8_t gyro_ui_filt_bw;
	uint8_t gyro_ui_filt_ord;
	uint8_t gyro_dec2_m2_ord;

} iim_config_t;

typedef struct IIM_ImuStructure
{
	iim_config_t *configState;
} iim_imu_t;


void IIM_IMU_Init(iim_imu_t *imuStructure, iim_config_t *imuConfigState, SPI_HandleTypeDef *spi_handler);

void IIM_IMU_Activate();

void IIM_IMU_Deactivate();

void IIM_IMU_WhoTest(iim_imu_t *imuStructure, uint8_t *dev_id);

void IIM_IMU_ReadRegister(iim_imu_t *imuStructure, uint8_t reg, uint8_t bytes, uint8_t *data_out);

void IIM_IMU_WriteRegister(iim_imu_t *imuStructure, uint8_t reg, uint8_t data_in);

void IIM_IMU_EnableTemperature(iim_imu_t *imuStructure);

void IIM_IMU_DisableTemperature(iim_imu_t *imuStructure);

void IIM_IMU_SetPowerConfig(iim_imu_t *imuStructure, uint8_t value);

void IIM_IMU_SetGyroConfig(iim_imu_t *imuStructure, uint8_t value);

void IIM_IMU_SetAccelConfig(iim_imu_t *imuStructure, uint8_t value);

void IIM_IMU_ReadTemperature(iim_imu_t *imuStructure, float *temperature);

#endif
