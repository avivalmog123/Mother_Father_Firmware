/*
 * utils.h
 *
 *  Created on: Feb 5, 2023
 *      Author: Michael Grenader
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_


#include "hw_drivers.h"

uint32_t crc32Init();
uint32_t crc32ByteCalc(uint32_t crc32Base, uint8_t NewDataByte);
uint32_t crc32BuffCalc(uint8_t *Message, uint32_t Offset, uint32_t nBytes);


#endif /* INC_UTILS_H_ */
