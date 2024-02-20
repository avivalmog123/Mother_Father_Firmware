/*
 * hal_hsb.h
 *
 *  Created on: Jan 12, 2023
 *      Author: Michael Grenader
 */

#ifndef INC_HAL_USB_H_
#define INC_HAL_USB_H_

#include "usartdll.h"

uint8_t usbBufferTx(uint8_t* BufferPtr, uint16_t BufferLength);
void usbBufferProcessing();
void usbCircularBufferInsert(uint8_t* Ptr, int Length);

#endif /* INC_HAL_USB_H_ */
