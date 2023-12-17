/**
 * @brief
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

typedef enum {
	LP,	// only for accel
	LN,	// for accel and gyro
	STB // only for gyroscope
} iim_imu_sensor_mode_t;

typedef struct iim_config_s
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
	iim_imu_sensor_mode_t *sensor_mode;
} iim_imu_config_t;

typedef struct iim_imu_data_raw_s
{
	int16_t x_axis;
	int16_t y_axis;
	int16_t z_axis;
} iim_data_raw_t;

typedef struct iim_imu_data_s
{
	float x_axis;
	float y_axis;
	float z_axis;
} iim_data_t;

typedef struct iim_imu_s
{
	iim_imu_config_t *config_state;
	iim_imu_sensor_mode_t *accel_mode;
	iim_imu_sensor_mode_t *gyro_mode;
	iim_data_raw_t *accel_raw;
	iim_data_raw_t *gyro_raw;
	iim_data_t *accel_data;
	iim_data_t *gyro_data;
} iim_imu_t;


void iim_imu_init(iim_imu_t *imu_s, SPI_HandleTypeDef *spi_handler);

void iim_imu_activate();

void iim_imu_deactivate();

void iim_imu_who_test(iim_imu_t *imu_s, uint8_t *dev_id);

void iim_imu_read_register(iim_imu_t *imu_s, uint8_t reg, uint8_t bytes, uint8_t *data_out);

void iim_imu_write_register(iim_imu_t *imu_s, uint8_t reg, uint8_t data_in);

void iim_imu_enable_temperature(iim_imu_t *imu_s);

void iim_imu_disable_temperature(iim_imu_t *imu_s);

void iim_imu_enable_accelerometer(iim_imu_t *imu_s);

void iim_imu_disable_accelerometer(iim_imu_t *imu_s);

void iim_imu_enable_gyroscope(iim_imu_t *imu_s);

void iim_imu_disable_gyroscope(iim_imu_t *imu_s);

void iim_imu_set_power_config(iim_imu_t *imu_s, uint8_t value);

void iim_imu_set_gyro_config(iim_imu_t *imu_s, uint8_t value);

void iim_imu_set_accel_config(iim_imu_t *imu_s, uint8_t value);

void iim_imu_read_temperature(iim_imu_t *imu_s, float *temperature);

void iim_imu_read_acceleration(iim_imu_t *imu_s);

void iim_imu_read_gyro(iim_imu_t *imu_s);

void iim_imu_convert_acceleration(iim_imu_t *imu_s);

void iim_imu_convert_gyro(iim_imu_t *imu_s);

#endif
