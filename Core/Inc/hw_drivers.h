/*
 * hw_drivers.h
 *
 *  Created on: Jan 12, 2023
 *      Author: Michael Grenader
 */

#include <Pc_Comm_App.h>
#include "stm32f405xx.h"
#include "main.h"
#include <stdlib.h>

void hwGpioSet(GPIO_TypeDef *GPIOx, uint16_t gpioPin, uint8_t pinState);
