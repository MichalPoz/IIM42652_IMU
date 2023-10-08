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
#define TEMP_DIS 	0x00100000
#define IDLE 		0b00010000
#define GYRO_STB	0b00000100
#define GYRO_LN		0b00001100
#define ACC_LP		0b00000010
#define ACC_LN		0b00000011
#define PWR_DEFAULT 0b00000000


typedef struct iim_imu_s
{
	SPI_HandleTypeDef *spi_h;
    uint8_t gyro_fs;
    uint8_t gyro_odr;
    uint8_t acc_fs;
    uint8_t acc_odr;

} iim_imu_t;

typedef struct IIM_IMU_AccelConfig
{
	uint8_t accel_fs_sel;
	uint8_t accel_odr;
	uint8_t accel_ui_filt_bw;
	uint8_t accel_ui_filt_ord;
	uint8_t accel_dec2_m2_ord;

} IIM_IMU_AccelConfig_t;

typedef struct IIM_IMU_GyroConfig
{
	uint8_t gyro_fs_sel;
	uint8_t gyro_odr;
	uint8_t gyro_ui_filt_bw;
	uint8_t gyro_ui_filt_ord;
	uint8_t gyro_dec2_m2_ord;

} IIM_IMU_GyroConfig_t;

typedef struct IIM_IMU_DataRaw
{
	int16_t x;
	int16_t y;
	int16_t z;

} IIM_IMU_DataRaw_t;

typedef struct IIM_IMU_DataScaled
{

};

void iim_who_test(uint8_t *dev_id);
void iim_activate_imu();
void iim_deactivate_imu();
void iim_imu_init(SPI_HandleTypeDef *spi_handler);
void iim_read_register(uint8_t *reg, uint8_t *data_out);
void iim_write_register(uint8_t *reg, uint8_t *data_in);
void iim_set_power_config(uint8_t *value);
void iim_set_gyro_config_0(uint8_t *value);
void iim_set_accel_config_0(uint8_t *value);
void iim_read_temperature(float *temperature);
void iim_read_acc_data();
void iim_read_gyro_data();

#endif
