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



void who_test();
void activate_imu();
void deactivate_imu();
void iim_imu_init(SPI_HandleTypeDef *spi_handler);
void soft_reset();
void read_register();
void write_register();

#endif
