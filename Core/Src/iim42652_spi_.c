/**
 * @file iim42652_spi_.c
 * @author Michał Poźniak (p.michal90.8@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-07-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef IIM42652_C
#define IIM42652_C

/* Includes */
#include "iim42652.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"

static iim_imu_config_t imu_config_s;
static iim_imu_sensor_mode_t accel_sensor_mode;
static iim_imu_sensor_mode_t gyro_sensor_mode;
static iim_data_raw_t accel_raw;
static iim_data_raw_t gyro_raw;
static iim_data_t accel_data;
static iim_data_t gyro_data;


void iim_imu_init(iim_imu_t *imu_s, SPI_HandleTypeDef *spi_handler)
{
	// Creating secondary structures - need to check
	//iim_imu_config_t imu_config_s;
	//iim_imu_sensor_mode_t accel_sensor_mode;
	//iim_imu_sensor_mode_t gyro_sensor_mode;
	//iim_data_raw_t accel_raw;
	//iim_data_raw_t gyro_raw;
	//iim_data_t	accel_data;
	//iim_data_t gyro_data;

	// Assigning addresses of SPI handler and structures
	imu_s->config_state = &imu_config_s;
	imu_s->config_state->spi_h = spi_handler;
	imu_s->accel_mode = &accel_sensor_mode;
	imu_s->gyro_mode = &gyro_sensor_mode;
	imu_s->accel_raw = &accel_raw;
	imu_s->gyro_raw = &gyro_raw;
	imu_s->accel_data = &accel_data;
	imu_s->gyro_data = &gyro_data;

	// Defining sensor modes
	accel_sensor_mode = LN;
	gyro_sensor_mode = LN;

	// Defining configurations of sensors

	// Connection test - checking SPI connection establishment

	// Turning on sensors and setting configuration
	iim_imu_enable_accelerometer(imu_s);
	iim_imu_enable_gyroscope(imu_s);
}

void iim_imu_activate()
{
	HAL_GPIO_WritePin(GPIOC, CHIP_SELECT_Pin, GPIO_PIN_RESET);
}

void iim_imu_deactivate()
{
	HAL_GPIO_WritePin(GPIOC, CHIP_SELECT_Pin, GPIO_PIN_SET);
}

void iim_imu_who_test(iim_imu_t *imu_s, uint8_t *dev_id)
{
	iim_imu_read_register(imu_s, WHO_AM_I, 1, dev_id);
}

void iim_imu_read_register(iim_imu_t *imu_s, uint8_t reg, uint8_t bytes, uint8_t *data_out)
{
	uint8_t addr = 0x80 | reg;
	uint8_t tx_buff[2] = {addr, 0};
	uint8_t rx_buff[2] = {0};
	iim_imu_activate();
	HAL_SPI_TransmitReceive(imu_s->config_state->spi_h, tx_buff, rx_buff, 2, 100);
	iim_imu_deactivate();
	*data_out = rx_buff[1];
}

void iim_imu_write_register(iim_imu_t *imu_s, uint8_t reg, uint8_t data_in)
{
	uint8_t addr = 0x00 | reg;
	uint8_t data_out[2] = {addr, data_in};
	iim_imu_activate();
	HAL_SPI_Transmit(imu_s->config_state->spi_h, data_out, 2, 1000);
	iim_imu_deactivate();
}

void iim_imu_enable_temperature(iim_imu_t *imu_s)
{
	uint8_t pwr_state;
	iim_imu_read_register(imu_s, PWR_MGMT0, 1, &pwr_state);
	if ( BITVALUE(pwr_state, 5) )
	{
		// _temperature measurement will only work when gyro or acc are powered on
		pwr_state &= (~TEMP_DISABLE);
		iim_imu_write_register(imu_s, PWR_MGMT0, pwr_state);
	}
}

void iim_imu_disable_temperature(iim_imu_t *imu_s)
{
	uint8_t pwr_state;
	iim_imu_read_register(imu_s, PWR_MGMT0, 1, &pwr_state);
	if ( !BITVALUE(pwr_state, 5) )
	{
		pwr_state |= TEMP_DISABLE;
		iim_imu_write_register(imu_s, PWR_MGMT0, pwr_state);
	}
}

void iim_imu_enable_accelerometer(iim_imu_t *imu_s)
{
	uint8_t pwr_state;
	iim_imu_read_register(imu_s, PWR_MGMT0, 1, &pwr_state);
	if ( !BITVALUE(pwr_state, 0) && !BITVALUE(pwr_state, 1) )
	{
		switch(*(imu_s->accel_mode))
		{
			case LP:	//LP
			{
				pwr_state |= ACC_LP;
				iim_imu_write_register(imu_s, PWR_MGMT0, pwr_state);
			} break;

			case LN:	//LN
			{
				pwr_state |= ACC_LN;
				iim_imu_write_register(imu_s, PWR_MGMT0, pwr_state);
			} break;

			default:
				break;
		}
	}
}

void iim_imu_disable_accelerometer(iim_imu_t *imu_s)
{
	uint8_t pwr_state;
	iim_imu_read_register(imu_s, PWR_MGMT0, 1, &pwr_state);
	if ( BITVALUE(pwr_state, 1) )
	{
		pwr_state &= (~ACC_LN);	//ACC_LN is equal to 00000011, to turn off 1:0 bit have to be 0s
	}
}

void iim_imu_enable_gyroscope(iim_imu_t *imu_s)
{
	uint8_t pwr_state;
	iim_imu_read_register(imu_s, PWR_MGMT0, 1, &pwr_state);
	if ( !BITVALUE(pwr_state, 3) && !BITVALUE(pwr_state, 2) )
	{
		switch(*(imu_s->gyro_mode))
		{

			case LN:	// LN
			{
				pwr_state |= GYRO_LN;
				iim_imu_write_register(imu_s, PWR_MGMT0, pwr_state);
			} break;

			case STB:	// STB
			{
				pwr_state |= GYRO_STB;
				iim_imu_write_register(imu_s, PWR_MGMT0, pwr_state);
			} break;

			default:
				break;
		}
	}
}

void iim_imu_disable_gyroscope(iim_imu_t *imu_s)
{
	uint8_t pwr_state;
	iim_imu_read_register(imu_s, PWR_MGMT0, 1, &pwr_state);
	if ( BITVALUE(pwr_state, 2) )
	{
		pwr_state &= (~GYRO_LN);	//GYRO_LN is equal to 00001100, to turn off 3:2 bits have to be 0s
	}
}

void iim_imu_read_temperature(iim_imu_t *imu_s, float *temperature)
{
	uint8_t rx_buff[2] = {0};
	int16_t tmp;

	// read _temperature register values - 2 bytes
	iim_imu_read_register(imu_s, TEMP_DATA1_UI, 1, rx_buff);
	iim_imu_read_register(imu_s, TEMP_DATA0_UI, 1, &rx_buff[1]);

	tmp = rx_buff[0];
	tmp <<= 8;
	tmp |= rx_buff[1];

	*temperature = ((float)tmp / 132.28) + 25;

}

void iim_imu_set_power_config(iim_imu_t *imu_s, uint8_t value)
{
	iim_imu_write_register(imu_s, PWR_MGMT0, value);
}

void iim_imu_set_gyro_config(iim_imu_t *imu_s, uint8_t value)
{
	iim_imu_write_register(imu_s, GYRO_CONFIG0, value);
}

void iim_imu_set_accel_config(iim_imu_t *imu_s, uint8_t value)
{
	iim_imu_write_register(imu_s, ACCEL_CONFIG0, value);
}

void iim_imu_read_acceleration(iim_imu_t *imu_s)
{
	uint8_t rx_buff[6] = {0};
	uint16_t tmp1;
	uint16_t tmp2;
	uint16_t tmp3;

	// read _temperature register values - 2 bytes
	iim_imu_read_register(imu_s, ACCEL_DATA_X1_UI, 1, rx_buff);
	iim_imu_read_register(imu_s, ACCEL_DATA_X0_UI, 1, &rx_buff[1]);
	iim_imu_read_register(imu_s, ACCEL_DATA_Y1_UI, 1, &rx_buff[2]);
	iim_imu_read_register(imu_s, ACCEL_DATA_Y0_UI, 1, &rx_buff[3]);
	iim_imu_read_register(imu_s, ACCEL_DATA_Z1_UI, 1, &rx_buff[4]);
	iim_imu_read_register(imu_s, ACCEL_DATA_Z0_UI, 1, &rx_buff[5]);

	// X - axis
	tmp1 = rx_buff[0];
	tmp1 <<= 8;
	tmp1 |= rx_buff[1];

	// Y - axis
	tmp2 = rx_buff[2];
	tmp2 <<= 8;
	tmp2 |= rx_buff[3];

	// Z - axis
	tmp3 = rx_buff[4];
	tmp3 <<= 8;
	tmp3 |= rx_buff[5];

	imu_s->accel_raw->x_axis = (int16_t)tmp1;
	imu_s->accel_raw->y_axis = (int16_t)tmp2;
	imu_s->accel_raw->z_axis = (int16_t)tmp3;

}

void iim_imu_convert_acceleration(iim_imu_t *imu_s)
{
	imu_s->accel_data->x_axis = ((float)imu_s->accel_raw->x_axis * 16.0) / 32768.0;
	imu_s->accel_data->y_axis = ((float)imu_s->accel_raw->y_axis * 16.0) / 32768.0;
	imu_s->accel_data->z_axis = ((float)imu_s->accel_raw->z_axis * 16.0) / 32768.0;
}

void iim_imu_read_gyro(iim_imu_t *imu_s)
{
	uint8_t rx_buff[6] = {0};
	uint16_t tmp1;
	uint16_t tmp2;
	uint16_t tmp3;

	// read _temperature register values - 2 bytes
	iim_imu_read_register(imu_s, GYRO_DATA_X1_UI, 1, rx_buff);
	iim_imu_read_register(imu_s, GYRO_DATA_X0_UI, 1, &rx_buff[1]);
	iim_imu_read_register(imu_s, GYRO_DATA_Y1_UI, 1, &rx_buff[2]);
	iim_imu_read_register(imu_s, GYRO_DATA_Y0_UI, 1, &rx_buff[3]);
	iim_imu_read_register(imu_s, GYRO_DATA_Z1_UI, 1, &rx_buff[4]);
	iim_imu_read_register(imu_s, GYRO_DATA_Z0_UI, 1, &rx_buff[5]);

	// X - axis
	tmp1 = rx_buff[0];
	tmp1 <<= 8;
	tmp1 |= rx_buff[1];

	// Y - axis
	tmp2 = rx_buff[2];
	tmp2 <<= 8;
	tmp2 |= rx_buff[3];

	// Z - axis
	tmp3 = rx_buff[4];
	tmp3 <<= 8;
	tmp3 |= rx_buff[5];

	imu_s->gyro_raw->x_axis = (int16_t)tmp1;
	imu_s->gyro_raw->y_axis = (int16_t)tmp2;
	imu_s->gyro_raw->z_axis = (int16_t)tmp3;

}

void iim_imu_convert_gyro(iim_imu_t *imu_s)
{
	imu_s->gyro_data->x_axis = ((float)imu_s->gyro_raw->x_axis * 2000.0) / 32768.0;
	imu_s->gyro_data->y_axis = ((float)imu_s->gyro_raw->y_axis * 2000.0) / 32768.0;
	imu_s->gyro_data->z_axis = ((float)imu_s->gyro_raw->z_axis * 2000.0) / 32768.0;
}

#endif
