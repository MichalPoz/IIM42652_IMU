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

#define BITVALUE(X, N) ( ( (X) >> (N) ) & 0x1 )

void iim_imu_init(iim_imu_t *imu_s, iim_imu_config_t *imu_config_s, SPI_HandleTypeDef *spi_handler)
{
	imu_s->config_state = imu_config_s;

	imu_s->config_state->spi_h = spi_handler;
	HAL_GPIO_WritePin(GPIOC, CHIP_SELECT_Pin, GPIO_PIN_SET);

	uint8_t reg_value = 0b00000001;
	iim_imu_write_register(imu_s, DEVICE_CONFIG, reg_value);
	HAL_Delay(1000);

	uint8_t test_val = 0b00000010;
	uint8_t condi = BITVALUE(test_val, 1);
	HAL_Delay(100);
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

void iim_imu_enable_accelerometer(iim_imu_t *imu_s, char accMode[])
{
	uint8_t pwr_state;
	iim_imu_read_register(imu_s, PWR_MGMT0, 1, &pwr_state);
	if ( !BITVALUE(pwr_state, 0) && !BITVALUE(pwr_state, 1) )
	{
		switch(accMode[])
		{
			case 'LP':
			{
				pwr_state |= ACC_LP;
				iim_imu_write_register(imu_s, PWR_MGMT0, pwr_state);
			} break;

			case 'LN':
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

void iim_imu_enable_gyroscope(iim_imu_t *imu_s, char gyroMode)
{
	uint8_t pwr_state;
	iim_imu_read_register(imu_s, PWR_MGMT0, 1, &pwr_state);
	if ( !BITVALUE(pwr_state, 3) && !BITVALUE(pwr_state, 2) )
	{
		switch(gyroMode)
		{
			case 'STB':
			{
				pwr_state |= GYRO_STB;
				iim_imu_write_register(imu_s, PWR_MGMT0, pwr_state);
			} break;

			case 'LN':
			{
				pwr_state |= GYRO_LN;
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

/*
void IIM_readAccData()
{
	uint8_t rx_buff[6];
	uint16_t tmp;

	uint8_t addr = 0x80 | ACCEL_DATA_X1_UI;
	iim_imu_Activate();
	HAL_SPI_Transmit(imu_s->configState->spi_h, &addr, 1, 1000);
	HAL_SPI_Receive(imu_s->configState->spi_h, rx_buff, 6, 1000);

    tmp = rx_buff[0];
    tmp <<= 8;
    tmp |= rx_buff[1];

    accelRaw.x = (int16_t)tmp;

    tmp = rx_buff[2];
    tmp <<= 8;
    tmp |= rx_buff[3];

    accelRaw.y = (int16_t)tmp;

    tmp = rx_buff[4];
    tmp <<= 8;
    tmp |= rx_buff[5];

    accelRaw.z = (int16_t)tmp;
}
*/

/*
void iim_convert_accel(iim_scaled_data *output, iim_raw_data input)
{
    output->x = ((float)input.x * 16.0) / 32768.0;
    output->y = ((float)input.y * 16.0) / 32768.0;
    output->z = ((float)input.z * 16.0) / 32768.0;
}

void iim_convert_gyro(iim_scaled_data *output, iim_raw_data input)
{
    output->x = ((float)input.x * 2000.0) / 32768.0;
    output->y = ((float)input.y * 2000.0) / 32768.0;
    output->z = ((float)input.z * 2000.0) / 32768.0;
}
*/
//void config_setup()


/*
void IIM_init_SPI(SPI_HandleTypeDef *spi_handler)
{
    status.spi_h = spi_handler;
    status.gyro_fs = set_GYRO_FS_SEL_2000_dps;
    status.gyro_odr = set_GYRO_ODR_1kHz;
    status.acc_fs = set_ACCEL_FS_SEL_16g;
    status.acc_odr = set_ACCEL_ODR_1kHz;
    HAL_GPIO_WritePin(GPIOC, Chip_select_Pin, GPIO_PIN_set);	// CS pin should be default high
}


void IIM_readAccel_SPI(iim_raw_data *data)
{
    uint8_t tmp[6];
    uint16_t temp;
    uint8_t reg_adr = 0x80 | ACCEL_DATA_X1_UI;
    HAL_GPIO_WritePin(GPIOC, Chip_select_Pin, GPIO_PIN_REset);
    HAL_SPI_Transmit(status.spi_h, &reg_adr, 1, 100);
    HAL_SPI_Receive(status.spi_h, tmp, 6, 100);
    HAL_GPIO_WritePin(GPIOC, Chip_select_Pin, GPIO_PIN_set);

    temp = (tmp[0] << 8) | tmp[1];
    data->x = (int16_t)temp;

    temp = (tmp[2] << 8) | tmp[3];
    data->y = (int16_t)temp;

    temp = (tmp[4] << 8) | tmp[5];
    data->z = (int16_t)temp;
}

void IIM_readGyro_SPI(iim_raw_data *data)
{
    uint8_t tmp[6];
    uint16_t temp;
    uint8_t reg_adr = 0x80 | GYRO_DATA_X1_UI;
    HAL_GPIO_WritePin(GPIOC, Chip_select_Pin, GPIO_PIN_REset);
    HAL_SPI_Transmit(status.spi_h, &reg_adr, 1, 100);
    HAL_SPI_Receive(status.spi_h, tmp, 6, 100);
    HAL_GPIO_WritePin(GPIOC, Chip_select_Pin, GPIO_PIN_set);

    temp = (tmp[0] << 8 | tmp[1]);
    data->x = (int16_t)temp;

    temp = (tmp[2] << 8 | tmp[3]);
    data->y = (int16_t)temp;

    temp = (tmp[4] << 8 | tmp[5]);
    data->z = (int16_t)temp;
}

void IIM_convertAccel_SPI(iim_scaled_data *output, iim_raw_data input)
{
    output->x = ((float)input.x * 16.0) / 32768.0;
    output->y = ((float)input.y * 16.0) / 32768.0;
    output->z = ((float)input.z * 16.0) / 32768.0;
}

void IIM_convertGyro_SPI(iim_scaled_data *output, iim_raw_data input)
{
    output->x = ((float)input.x * 2000.0) / 32768.0;
    output->y = ((float)input.y * 2000.0) / 32768.0;
    output->z = ((float)input.z * 2000.0) / 32768.0;
}

void IIM_configAccel_SPI(uint8_t fs, uint8_t odr)
{
    status.acc_fs = fs;
    status.acc_odr = odr;
    uint8_t tmp;
    tmp = fs << 5;
    tmp |= odr;
    uint8_t reg_adr = 0x00 | GYRO_DATA_X1_UI;
    //HAL_I2C_Mem_Write(status.i2c_h, IIM_ADR, ACCEL_CONFIG0, 1, &tmp, 1, 10);
    HAL_GPIO_WritePin(GPIOC, Chip_select_Pin, GPIO_PIN_REset);
    HAL_SPI_Transmit(status.spi_h, pData, Size, Timeout)
    HAL_SPI_Transmit(status.spi_h, &ACCEL_CONFIG0, 1, 10);
    HAL_GPIO_WritePin(GPIOC, Chip_select_Pin, GPIO_PIN_set);
}

void IIM_configGyro_SPI(uint8_t fs, uint8_t odr)
{
    status.gyro_fs = fs;
    status.gyro_odr = odr;
    uint8_t tmp;
    tmp = fs << 5;
    tmp |= odr;
    //HAL_I2C_Mem_Write(status.i2c_h, IIM_ADR, GYRO_CONFIG0, 1, &tmp, 1, 10);
}
*/
#endif
