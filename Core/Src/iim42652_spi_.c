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

void IIM_IMU_Init(iim_imu_t *imuStructure, iim_config_t *imuConfigState, SPI_HandleTypeDef *spi_handler)
{
	imuStructure->configState = imuConfigState;

	imuStructure->configState->spi_h = spi_handler;
	HAL_GPIO_WritePin(GPIOC, CHIP_SELECT_Pin, GPIO_PIN_SET);

	uint8_t regValue = 0b00000001;
	IIM_IMU_WriteRegister(imuStructure, DEVICE_CONFIG, regValue);
	HAL_Delay(1000);

	uint8_t testVal = 0b00000010;
	uint8_t condi = BITVALUE(testVal, 1);
	HAL_Delay(100);
}

void IIM_IMU_Activate()
{
	HAL_GPIO_WritePin(GPIOC, CHIP_SELECT_Pin, GPIO_PIN_RESET);
}

void IIM_IMU_Deactivate()
{
	HAL_GPIO_WritePin(GPIOC, CHIP_SELECT_Pin, GPIO_PIN_SET);
}

void IIM_IMU_WhoTest(iim_imu_t *imuStructure, uint8_t *dev_id)
{
	IIM_IMU_ReadRegister(imuStructure, WHO_AM_I, 1, dev_id);
}

void IIM_IMU_ReadRegister(iim_imu_t *imuStructure, uint8_t reg, uint8_t bytes, uint8_t *data_out)
{
	uint8_t addr = 0x80 | reg;
	uint8_t txBuff[2] = {addr, 0};
	uint8_t rxBuff[2] = {0};
	IIM_IMU_Activate();
	//HAL_SPI_Transmit(imuStructure->configState->spi_h, &addr, 1, 1000);
	//HAL_SPI_Receive(imuStructure->configState->spi_h, rxBuff, 2, 1000);
	HAL_SPI_TransmitReceive(imuStructure->configState->spi_h, txBuff, rxBuff, 2, 100);
	IIM_IMU_Deactivate();
	*data_out = rxBuff[1];
	HAL_Delay(10);
}

void IIM_IMU_WriteRegister(iim_imu_t *imuStructure, uint8_t reg, uint8_t data_in)
{
	uint8_t addr = 0x00 | reg;
	uint8_t data_out[2] = {addr, data_in};
	IIM_IMU_Activate();
	HAL_SPI_Transmit(imuStructure->configState->spi_h, data_out, 2, 1000);
	IIM_IMU_Deactivate();
}

void IIM_IMU_EnableTemperature(iim_imu_t *imuStructure)
{
	uint8_t pwrState;
	IIM_IMU_ReadRegister(imuStructure, PWR_MGMT0, 1, &pwrState);
	if ( BITVALUE(pwrState, 5) )
	{
		// Temperature measurement will only work when gyro or acc are powered on
		//pwrState &= (~TEMP_DISABLE);
		pwrState = 0b00011100;
		IIM_IMU_WriteRegister(imuStructure, PWR_MGMT0, pwrState);
	}
}

void IIM_IMU_DisableTemperature(iim_imu_t *imuStructure)
{
	uint8_t pwrState;
	IIM_IMU_ReadRegister(imuStructure, PWR_MGMT0, 1, &pwrState);
	if ( !BITVALUE(pwrState, 5) )
	{
		pwrState |= TEMP_DISABLE;
		IIM_IMU_WriteRegister(imuStructure, PWR_MGMT0, pwrState);
	}
}

void IIM_IMU_EnableAccelerometer()
{

}

void IIM_IMU_EnableGyroscope()
{

}

void IIM_IMU_DisableAccelerometer()
{

}

void IIM_IMU_DisableGyroscope()
{

}

void IIM_IMU_ReadTemperature(iim_imu_t *imuStructure, float *temperature)
{
	uint8_t rxBuff[2] = {0};
	int16_t tmp;

	// Read TEMPerature register values - 2 bytes
	IIM_IMU_ReadRegister(imuStructure, TEMP_DATA1_UI, 1, rxBuff);
	IIM_IMU_ReadRegister(imuStructure, TEMP_DATA0_UI, 1, &rxBuff[1]);

	tmp = rxBuff[0];
	tmp <<= 8;
	tmp |= rxBuff[1];

	*temperature = ((float)tmp / 132.28) + 25;

}

void IIM_IMU_SetPowerConfig(iim_imu_t *imuStructure, uint8_t value)
{
	IIM_IMU_WriteRegister(imuStructure, PWR_MGMT0, value);
}

void IIM_IMU_SetGyroConfig(iim_imu_t *imuStructure, uint8_t value)
{
	IIM_IMU_WriteRegister(imuStructure, GYRO_CONFIG0, value);
}

void IIM_IMU_SetAccelConfig(iim_imu_t *imuStructure, uint8_t value)
{
	IIM_IMU_WriteRegister(imuStructure, ACCEL_CONFIG0, value);
}

/*
void IIM_ReadAccData()
{
	uint8_t rx_buff[6];
	uint16_t tmp;

	uint8_t addr = 0x80 | ACCEL_DATA_X1_UI;
	IIM_IMU_Activate();
	HAL_SPI_Transmit(imuStructure->configState->spi_h, &addr, 1, 1000);
	HAL_SPI_Receive(imuStructure->configState->spi_h, rx_buff, 6, 1000);

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
    status.gyro_fs = SET_GYRO_FS_SEL_2000_dps;
    status.gyro_odr = SET_GYRO_ODR_1kHz;
    status.acc_fs = SET_ACCEL_FS_SEL_16g;
    status.acc_odr = SET_ACCEL_ODR_1kHz;
    HAL_GPIO_WritePin(GPIOC, Chip_select_Pin, GPIO_PIN_SET);	// CS pin should be default high
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
