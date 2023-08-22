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
#define TEMP_DIS 	0x20
#define IDLE 		0x10
#define GYRO_STB	0x04
#define GYRO_LN		0x0c
#define ACC_LP		0x02
#define ACC_LN		0x03
#define PWR_DEFAULT 0x00


typedef struct iim_imu_s
{
	SPI_HandleTypeDef *spi_h;
    uint8_t gyro_fs;
    uint8_t gyro_odr;
    uint8_t acc_fs;
    uint8_t acc_odr;

} iim_imu_t;

typedef struct iim_imu_accel_cfg
{
	uint8_t accel_fs_sel;
	uint8_t accel_odr;
	uint8_t accel_ui_filt_bw;
	uint8_t accel_ui_filt_ord;
	uint8_t accel_dec2_m2_ord;

} iim_imu_accel_cfg_t;

typedef struct iim_imu_gyro_cfg
{
	uint8_t gyro_fs_sel;
	uint8_t gyro_odr;
	uint8_t gyro_ui_filt_bw;
	uint8_t gyro_ui_filt_ord;
	uint8_t gyro_dec2_m2_ord;

} iim_imu_gyro_cfg_t;

void who_test(uint8_t *dev_id);
void activate_imu();
void deactivate_imu();
void iim_imu_init(SPI_HandleTypeDef *spi_handler);
void read_register(uint8_t *reg, uint8_t *data_out);
void write_register(uint8_t *reg, uint8_t *data_in);
void set_power_config(uint8_t *value);
void set_gyro_config_0(uint8_t *value);
void set_accel_config_0(uint8_t *value);
void read_temperature(float *temperature);
void read_acc_data();
void read_gyro_data();

#endif
