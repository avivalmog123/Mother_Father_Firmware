/*
 * usartdll.c
 *
 *  Created on: Jan 15, 2023
 *      Author: Michael Grenader
 */

#include "usartdll.h"

typedef enum {
	UART_RX_DLL_IDLE,
	UART_RX_DLL_SYNC_LSB_WAIT,
	UART_RX_DLL_ADDRESS_MSB_WAIT,
	UART_RX_DLL_ADDRESS_LSB_WAIT,
	UART_RX_DLL_OPCODE_WAIT,
	UART_RX_DLL_PAYLOAD_WAIT,
	UART_RX_DLL_CRC32_3_WAIT,
	UART_RX_DLL_CRC32_2_WAIT,
	UART_RX_DLL_CRC32_1_WAIT,
	UART_RX_DLL_CRC32_0_WAIT
} uartDllState_T;

#define UARTDLL_SYNC_WORD_MSB       'V'
#define UARTDLL_SYNC_WORD_LSB       'E'
#define UART_RX_BUFFER_LENGTH                   256
uint8_t UartRxBuffer[UART_RX_BUFFER_LENGTH];

uartDllState_T uartRxDllState = UART_RX_DLL_IDLE;
uint8_t uartBufferIndex;
uint16_t uartDllMessageLengthTemp;
uint16_t uartDllMessageLength;
uint16_t MessageLength;
uint8_t uartCommand;
uint32_t uartReceivedCrc32;
uint32_t uartAccumulated;

uint16_t errorCounter = 0;

/********************************************************************
 * void usartDllRxFsm( uint8_t Char)
 * FSM for checking message is in the correct structure
 * After finishing checking the whole message structure, go to Communication Processing
 *********************************************************************/
void usartDllRxFsm(uint8_t Char) {
	switch (uartRxDllState) {
	case UART_RX_DLL_IDLE:
		// Check if first char is the sync byte
		if (Char == UARTDLL_SYNC_WORD_MSB) {
			uartBufferIndex = 0;
			UartRxBuffer[uartBufferIndex] = Char;
			uartBufferIndex++;
			uartRxDllState = UART_RX_DLL_SYNC_LSB_WAIT;
		}
		break;
	case UART_RX_DLL_SYNC_LSB_WAIT:
		// Check if second char is the sync byte
		if (Char == UARTDLL_SYNC_WORD_LSB) {
			UartRxBuffer[uartBufferIndex] = Char;
			uartBufferIndex++;
			uartRxDllState = UART_RX_DLL_ADDRESS_MSB_WAIT;
		} else {
			uartRxDllState = UART_RX_DLL_IDLE;
			errorCounter++;
		}
		break;
	case UART_RX_DLL_ADDRESS_MSB_WAIT:
		// 3 char is MSB of data length
		UartRxBuffer[uartBufferIndex] = Char;
		uartBufferIndex++;
		uartDllMessageLengthTemp = (((uint16_t) Char) << 8);
		uartRxDllState = UART_RX_DLL_ADDRESS_LSB_WAIT;
		break;
	case UART_RX_DLL_ADDRESS_LSB_WAIT:
		// 4 char is LSB of data length
		UartRxBuffer[uartBufferIndex] = Char;
		uartDllMessageLengthTemp += Char;
		uartBufferIndex++;

		uartDllMessageLength = uartDllMessageLengthTemp;
		// Check if the message length is invalid, then abort and go to idle
		// Add one to the message dll length when checking the max capacity for the opcode
		if ((uartDllMessageLength + 1) >= UART_RX_BUFFER_LENGTH) {
			uartRxDllState = UART_RX_DLL_IDLE;
			errorCounter++;
		}
		uartAccumulated = crc32Init();
		uartRxDllState = UART_RX_DLL_OPCODE_WAIT;
		break;
	case UART_RX_DLL_OPCODE_WAIT:
		// Check 5 char is Opcode byte
		UartRxBuffer[uartBufferIndex] = Char;
		uartBufferIndex++;
		uartAccumulated = crc32ByteCalc(uartAccumulated, Char);
		uartRxDllState = UART_RX_DLL_PAYLOAD_WAIT;
		if (uartDllMessageLengthTemp)
			uartRxDllState = UART_RX_DLL_PAYLOAD_WAIT;
		else {
			uartRxDllState = UART_RX_DLL_CRC32_3_WAIT;
			uartReceivedCrc32 = 0;
		}
		break;
	case UART_RX_DLL_PAYLOAD_WAIT:
		// Data
		UartRxBuffer[uartBufferIndex] = Char;
		uartBufferIndex++;
		uartAccumulated = crc32ByteCalc(uartAccumulated, Char);
		if (!--uartDllMessageLengthTemp) {
			uartRxDllState = UART_RX_DLL_CRC32_3_WAIT;
			uartReceivedCrc32 = 0;
		}
		break;
	// Cases for CRC32 bytes
	case UART_RX_DLL_CRC32_3_WAIT:
		uartReceivedCrc32 |= (Char << (24));
		uartRxDllState = UART_RX_DLL_CRC32_2_WAIT;
		break;
	case UART_RX_DLL_CRC32_2_WAIT:
		uartReceivedCrc32 |= (Char << (16));
		uartRxDllState = UART_RX_DLL_CRC32_1_WAIT;
		break;
	case UART_RX_DLL_CRC32_1_WAIT:
		uartReceivedCrc32 |= (Char << (8));
		uartRxDllState = UART_RX_DLL_CRC32_0_WAIT;
		break;
	case UART_RX_DLL_CRC32_0_WAIT:
		uartReceivedCrc32 |= (Char);
		if (~uartAccumulated == uartReceivedCrc32) {
			Pc_Comm_App_Processing(UartRxBuffer, MessageLength);
			uartRxDllState = UART_RX_DLL_IDLE;
		} else
			errorCounter++;
		break;
	}
}
