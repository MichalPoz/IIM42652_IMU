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

iim_imu_t iim_imu_s;


void iim_imu_init(SPI_HandleTypeDef *spi_handler)
{
	iim_imu_s.spi_h = spi_handler;
	iim_imu_s.gyro_fs = SET_GYRO_FS_SEL_2000_dps;
	iim_imu_s.gyro_odr = SET_GYRO_ODR_1kHz;
	iim_imu_s.acc_fs = SET_ACCEL_FS_SEL_16g;
	iim_imu_s.acc_odr = SET_ACCEL_ODR_1kHz;
	HAL_GPIO_WritePin(GPIOC, CHIP_SELECT_Pin, GPIO_PIN_SET);

}

void activate_imu()
{
	HAL_GPIO_WritePin(GPIOC, CHIP_SELECT_Pin, GPIO_PIN_RESET);
}

void deactivate_imu()
{
	HAL_GPIO_WritePin(GPIOC, CHIP_SELECT_Pin, GPIO_PIN_SET);
}

void who_test(uint8_t *dev_id)
{
	uint8_t reg = WHO_AM_I;
	read_register(&reg, dev_id);
}

void read_register(uint8_t *reg, uint8_t *data_out)
{
	uint8_t addr = 0x80 | *reg;
	activate_imu();
	HAL_SPI_Transmit(iim_imu_s.spi_h, &addr, 1, 100);
	while(HAL_SPI_GetState(iim_imu_s.spi_h) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(iim_imu_s.spi_h, data_out, 1, 100);
	deactivate_imu();
	HAL_Delay(10);
}

void write_register(uint8_t *reg, uint8_t *data_in)
{
	uint8_t addr = 0x00 | *reg;
	activate_imu();
	HAL_SPI_Transmit(iim_imu_s.spi_h, &addr, 1, 100);
	while(HAL_SPI_GetState(iim_imu_s.spi_h) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(iim_imu_s.spi_h, data_in, 1, 100);
	deactivate_imu();
	HAL_Delay(100);
}

void set_power_config(uint8_t *value)
{
	uint8_t reg = PWR_MGMT0;
	write_register(&reg, value);
}

void set_gyro_config_0(uint8_t *value)
{
	uint8_t reg = GYRO_CONFIG0;
	write_register(&reg, value);
}

void set_accel_config_0(uint8_t *value)
{
	uint8_t reg = ACCEL_CONFIG0;
	write_register(&reg, value);
}

void read_temperature()
{

}

void read_acc_data()
{

}

void read_gyro_data()
{

}
//void config_setup()


/*
void IIM_init_SPI(SPI_HandleTypeDef *spi_handler)
{
    status.spi_h = spi_handler;
    status.gyro_fs = SET_GYRO_FS_SEL_2000_dps;
    status.gyro_odr = SET_GYRO_ODR_1kHz;
    status.acc_fs = SET_ACCEL_FS_SEL_16g;
    status.acc_odr = SET_ACCEL_ODR_1kHz;
    HAL_GPIO_WritePin(GPIOC, Chip_select_Pin, GPIO_PIN_SET);	// CS pin should be default high
}

void IIM_readTemperature_SPI(float *temperature)
{
    uint8_t tmp[2];
    uint8_t reg_adr = 0x80 | TEMP_DATA1_UI;
    activate_imu();
    HAL_SPI_TransmitReceive(status.spi_h, &reg_adr, tmp, 2, 100);
    deactivate_imu();
    int16_t temp_int = (tmp[0] << 8) | tmp[1];
    *temperature = (float)temp_int;
    *temperature = ((*temperature) / 132.48) + 25.0;
}

void IIM_readAccel_SPI(iim_raw_data *data)
{
    uint8_t tmp[6];
    uint16_t temp;
    uint8_t reg_adr = 0x80 | ACCEL_DATA_X1_UI;
    HAL_GPIO_WritePin(GPIOC, Chip_select_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(status.spi_h, &reg_adr, 1, 100);
    HAL_SPI_Receive(status.spi_h, tmp, 6, 100);
    HAL_GPIO_WritePin(GPIOC, Chip_select_Pin, GPIO_PIN_SET);

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
    HAL_GPIO_WritePin(GPIOC, Chip_select_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(status.spi_h, &reg_adr, 1, 100);
    HAL_SPI_Receive(status.spi_h, tmp, 6, 100);
    HAL_GPIO_WritePin(GPIOC, Chip_select_Pin, GPIO_PIN_SET);

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
    HAL_GPIO_WritePin(GPIOC, Chip_select_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(status.spi_h, pData, Size, Timeout)
    HAL_SPI_Transmit(status.spi_h, &ACCEL_CONFIG0, 1, 10);
    HAL_GPIO_WritePin(GPIOC, Chip_select_Pin, GPIO_PIN_SET);
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
