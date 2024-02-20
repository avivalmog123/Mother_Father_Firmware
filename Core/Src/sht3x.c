/*
 * sht3x.c
 *
 *  Created on: 16/03/2023
 *      Author: Michael Grenader
 */


#include <assert.h>
#include <sht3x.h>

/**
 * Registers addresses.
 */
typedef enum
{
	SHT3X_COMMAND_MEASURE_HIGHREP_STRETCH = 0x2C06,
	SHT3X_COMMAND_CLEAR_STATUS = 0x3041,
	SHT3X_COMMAND_BREAK = 0x3093,
	SHT3X_COMMAND_SOFT_RESET = 0x30A2,
	SHT3X_COMMAND_HEATER_ENABLE = 0x306d,
	SHT3X_COMMAND_HEATER_DISABLE = 0x3066,
	SHT3X_COMMAND_READ_STATUS = 0xF32D,
	SHT3X_COMMAND_FETCH_DATA = 0xE000,
	SHT3X_COMMAND_MEASURE_HIGHREP_10HZ = 0x2737,
	SHT3X_COMMAND_MEASURE_LOWREP_10HZ = 0x272A
} sht3x_command_t;

void SHT3X_Set_Handle(sht3x_handle_t* SHT3X, I2C_HandleTypeDef* hi2cx)
{
	SHT3X->i2c_handle = hi2cx;
}

void SHT3X_Set_Address(sht3x_handle_t* SHT3X, SHT3X_ADDR_t Address)
{
	SHT3X->device_address = Address;
}

static uint8_t calculate_crc(const uint8_t *data, size_t length)
{
	uint8_t crc = 0xff;
	for (size_t i = 0; i < length; i++) {
		crc ^= data[i];
		for (size_t j = 0; j < 8; j++) {
			if ((crc & 0x80u) != 0) {
				crc = (uint8_t)((uint8_t)(crc << 1u) ^ 0x31u);
			} else {
				crc <<= 1u;
			}
		}
	}
	return crc;
}


static bool sht3x_send_command(sht3x_handle_t *handle, sht3x_command_t command)
{
	uint8_t command_buffer[2] = {(command & 0xff00u) >> 8u, command & 0xffu};

	if (HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address << 1u, command_buffer, sizeof(command_buffer),
	                            SHT3X_I2C_TIMEOUT) != HAL_OK) {
		return false;
	}

	return true;
}

static uint16_t uint8_to_uint16(uint8_t msb, uint8_t lsb)
{
	return (uint16_t)((uint16_t)msb << 8u) | lsb;
}

bool sht3x_init(sht3x_handle_t *handle)
{
	assert(handle->i2c_handle->Init.NoStretchMode == I2C_NOSTRETCH_DISABLED);
	// TODO: Assert i2c frequency is not too high

	int test;


	uint8_t status_reg_and_checksum[3];
	if (HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address << 1u, test, 1, SHT3X_I2C_TIMEOUT) != HAL_OK) {
		return false;
	}

	if (HAL_I2C_Mem_Read(handle->i2c_handle, handle->device_address << 1u, SHT3X_COMMAND_READ_STATUS, 2, (uint8_t*)&status_reg_and_checksum,
					  sizeof(status_reg_and_checksum), SHT3X_I2C_TIMEOUT) != HAL_OK) {
		return false;
	}

	uint8_t calculated_crc = calculate_crc(status_reg_and_checksum, 2);

	if (calculated_crc != status_reg_and_checksum[2]) {
		return false;
	}

	return true;
}

bool sht3x_read_temperature_and_humidity(sht3x_handle_t *handle, float *temperature, float *humidity)
{

	sht3x_send_command(handle, SHT3X_COMMAND_MEASURE_HIGHREP_STRETCH);

	// After sending a command to the sensor, a minimal waiting time of 1ms is needed before another command can be received by the sensor
	HAL_Delay(1);

	uint8_t buffer[6];
	if (HAL_I2C_Master_Receive(handle->i2c_handle, handle->device_address << 1u, buffer, sizeof(buffer), SHT3X_I2C_TIMEOUT) != HAL_OK) {
		return false;
	}

	uint8_t temperature_crc = calculate_crc(buffer, 2);
	uint8_t humidity_crc = calculate_crc(buffer + 3, 2);
	if (temperature_crc != buffer[2] || humidity_crc != buffer[5]) {
		return false;
	}

	int16_t temperature_raw = (int16_t)uint8_to_uint16(buffer[0], buffer[1]);
	uint16_t humidity_raw = uint8_to_uint16(buffer[3], buffer[4]);

	*temperature = -45.0f + 175.0f * (float)temperature_raw / 65535.0f;  // °C
	*humidity = 100.0f * (float)humidity_raw / 65535.0f;  // %RH

	return true;
}

bool sht3x_set_header_enable(sht3x_handle_t *handle, bool enable)
{
	if (enable) {
		return sht3x_send_command(handle, SHT3X_COMMAND_HEATER_ENABLE);
	} else {
		return sht3x_send_command(handle, SHT3X_COMMAND_HEATER_DISABLE);
	}
}
