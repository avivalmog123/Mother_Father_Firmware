/*
 * hal_usb.c
 *
 *  Created on: Jan 12, 2023
 *      Author: Michael Grenader
 */

#include "hal_usb.h"
#include "usbd_cdc_if.h"

/*   	Definitions 					  */
#define USB_RX_CIRCULAR_BUFFER_LENGTH	5000
#define USB_TX_RETRIES	3
void usbFsmCallBack();
void usbFsmTimerCallback();

uint16_t usbRxCircularHeadIndex = 0;
uint16_t usbRxCircularTailIndex = 0;
uint8_t usbRxCicularBuffer[USB_RX_CIRCULAR_BUFFER_LENGTH];

uint8_t usbBufferTx(uint8_t *BufferPtr, uint16_t BufferLength) {
	uint8_t i = 0;
	uint8_t result = 0;
	for (i = 0; i < USB_TX_RETRIES; i++) {
		result = CDC_Transmit_HS(BufferPtr, BufferLength);
		if (result == USBD_OK) {
			break;
		}
	}
	return (result);
}

void usbBufferProcessing() {
	uint16_t i;

	uint16_t MyLocalHeadIndex = usbRxCircularHeadIndex;
	if (usbRxCircularTailIndex < MyLocalHeadIndex) {
		for (i = 0; i < MyLocalHeadIndex - usbRxCircularTailIndex; i++)
			usartDllRxFsm(usbRxCicularBuffer[usbRxCircularTailIndex + i]);
	}
	if (usbRxCircularTailIndex > MyLocalHeadIndex) {
		for (i = 0; i < USB_RX_CIRCULAR_BUFFER_LENGTH - usbRxCircularTailIndex;
				i++)
			usartDllRxFsm(usbRxCicularBuffer[usbRxCircularTailIndex + i]);
		for (i = 0; i < MyLocalHeadIndex; i++)
			usartDllRxFsm(usbRxCicularBuffer[i]);
	}
	usbRxCircularTailIndex = MyLocalHeadIndex;
}

void usbCircularBufferInsert(uint8_t *Ptr, int Length) {

	uint16_t i;
	for (i = 0; i < Length; i++) {
		usbRxCicularBuffer[usbRxCircularHeadIndex] = *(Ptr + i);
		usbRxCircularHeadIndex++;
		if (usbRxCircularHeadIndex >= USB_RX_CIRCULAR_BUFFER_LENGTH)
			usbRxCircularHeadIndex = 0;
	}

}
