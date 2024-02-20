/*
 * sht3x.h
 *
 *  Created on: 16/03/2023
 *      Author: Michael Grenader
 */


#pragma once

#include <stdbool.h>

#include "stm32f4xx_hal.h"



#ifndef SHT3X_I2C_TIMEOUT
//#define SHT3X_I2C_TIMEOUT 30
#define SHT3X_I2C_TIMEOUT 100
#endif

typedef enum {
	SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW = 0x44,  /* I2C Address A (ADDR connected to logic low)  */
	SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_HIGH = 0x45  /* I2C Address B (ADDR connected to logic high)  */
} SHT3X_ADDR_t;



/**
 * Structure defining a handle describing a SHT3x device.
 */
typedef struct {

	/**
	 * The handle to the I2C bus for the device.
	 */
	I2C_HandleTypeDef *i2c_handle;

	/**
	 * The I2C device address.
	 * @see{PCA9865_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW} and @see{SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_HIGH}
	 */
	uint16_t device_address;

} sht3x_handle_t;

extern sht3x_handle_t sht3x;

void SHT3X_Set_Handle(sht3x_handle_t* SHT3X, I2C_HandleTypeDef* hi2cx);
void SHT3X_Set_Address(sht3x_handle_t* SHT3X, SHT3X_ADDR_t Address);

/**
 * Checks if an SHT3x is reachable using the given handle.
 * @param handle Handle to the SHT3x device.
 * @return True on success, false otherwise.
 */
bool sht3x_init(sht3x_handle_t *handle);

/**
 * Takes a single temperature and humidity measurement.
 * @param handle Handle to the SHT3x device.
 * @param temperature Pointer to the storage location for the sampled temperature.
 * @param humidity Pointer to the storage location for the sampled humidity.
 * @return True on success, false otherwise.
 */
bool sht3x_read_temperature_and_humidity(sht3x_handle_t *handle, float *temperature, float *humidity);

/**
 * Turns the SHT3x's internal heater on or of.
 * @param handle Handle to the SHT3x device.
 * @param enable True to enable to heater, false to disable it.
 * @return True on success, false otherwise.
 */
bool sht3x_set_header_enable(sht3x_handle_t *handle, bool enable);
