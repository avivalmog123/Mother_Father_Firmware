/*
 * Pc_Comm_App.h
 *
 *  Created on: Jan 12, 2023
 *      Author: Michael Grenader
 */

#ifndef INC_PC_COMM_APP_H_
#define INC_PC_COMM_APP_H_

#include "hw_drivers.h"
#include "utils.h"
#include "hal_usb.h"
#include "stm32f4xx_hal_def.h"
//#include "stm32f407xx.h"


#define PC_COM_APP_RET_BUFFER_SIZE 200

// DLL Header
typedef struct __packed{
	uint8_t Sync0; // "V"
	uint8_t Sync1; // "E"
	uint16_t PayloadLength; // Payloadload length in Bytes
} DllHeader_t;

// DLL End Of Message
typedef struct  __packed {
	uint32_t CRC32; // MSByte first CRC32 calculated one byte after the Message Length polynomial 0x04C11DB7
} DllEndOfMessage_t;

typedef enum {
	SET_SELECTOR_EN = 0x00,
	GET_SHT3X_TEMP_MEAS = 0x01,
	GET_HUMID_MEAS = 0x02,
	SET_HUMID_NRST = 0x03,
	TEST = 0x04,
	PCAP04_WRITE_MEMORY = 0x05,
	PCAP04_READ_MEMORY = 0x06,
	PCAP04_WRITE_CONFIG = 0x07,
	PCAP04_READ_CONFIG = 0x08,
	PCAP04_READ_RESULTS_RAM = 0x09,
	PCAP04_POWER_ON_RESET = 0x0a,
	PCAP04_INITIALIZE = 0x0b,
	PCAP04_CDC_START_CONVERSION = 0x0c,
	PCAP04_RDC_START_CONVERSION = 0x0d,
	PCAP04_NV_STORE = 0x0e,
	PCAP04_NV_RECALL = 0x0f,
	PCAP04_NV_ERASE = 0x10,
	SET_BURN_MODE_START = 0x11,
	GET_LPS22_TEMP_MEAS = 0x12,
	GET_PRESSURE_MEAS = 0x13

} Pc_Comm_App_Opcode;

typedef struct  __packed{

	uint16_t EXIN_AN0;  // PA1_ADC1_IN1
	uint16_t EXIN_AN1;  // PA2_ADC1_IN2
	uint8_t EX_CS;		// PA9
//	uint16_t SL_SD;		// PA10
	uint8_t EXIN_GP4;	// PA11
	uint8_t EX_EN;		// PA12
	uint8_t SIG3;		// PB0
	uint8_t set_HUMID_NRST;// PB9
	uint8_t SIG1;  		// PB13
	uint8_t EXIN_GP5;	// PC3
	uint8_t EXT_CLOCK;	// PC9
	uint8_t EXT_SCK;	// PC10
	uint8_t EXT_MISO;	// PC11
	uint8_t EXT_MOSI;	// PC12
	uint8_t SIG2;		// PD1
	uint8_t EXIN_GP9;	// PD7
	uint8_t EXIN_GP8;	// PD8
	uint8_t EXIN_GP7;	// PD10
	uint8_t EXIN_GP6;	// PD11
	uint8_t POWER_FAULT;// PD15
	uint8_t EXIN_GP0;	// PE0
	uint8_t EXIN_GP1;	// PE1
	uint8_t EXIN_GP2;	// PE2
	uint8_t EXIN_GP3;	// PE3
	uint8_t set_DPS_RST;// PE8
	uint8_t set_OE;		// PE9
	uint8_t set_SR_CLR;	// PE10
//	uint16_t SL_SS;		// PE14
//	uint16_t SL_SP;		// PE15

} Pc_Comm_App_Data;


typedef struct  __packed{
	uint8_t sync0;
	uint8_t sync1;
	uint8_t dllMsb;
	uint8_t dllLsb;
	uint8_t opcode;
	uint8_t data[];
} PcCommAppLayerStruct_T;


typedef enum
{
	PCAP04_UNIT_0,
	PCAP04_UNIT_1,
	PCAP04_UNIT_2,
	PCAP04_UNIT_3,
	PCAP04_UNIT_4,
	PCAP04_UNIT_5,
	PCAP04_UNIT_6,
	PCAP04_UNIT_7,
	PCAP04_UNIT_8,
	PCAP04_UNIT_9,
	PCAP04_UNIT_10,
	PCAP04_UNIT_11,
	PCAP04_UNIT_12,
	PCAP04_UNIT_13,
	PCAP04_UNIT_14,
	PCAP04_UNIT_15,
	PCAP04_UNIT_16,
	PCAP04_UNIT_17,
	PCAP04_UNIT_18,
	PCAP04_UNIT_19,
	PCAP04_UNIT_20,
	PCAP04_UNIT_21,
	PCAP04_UNIT_22,
	PCAP04_UNIT_23,
	PCAP04_UNIT_24,
	PCAP04_UNIT_25,
	PCAP04_UNIT_26,
	PCAP04_UNIT_27,
	PCAP04_UNIT_28,
	PCAP04_UNIT_29,
	PCAP04_UNIT_30,
	PCAP04_UNIT_31,
	PCAP04_UNIT_32,
	PCAP04_UNIT_33,
	PCAP04_UNIT_34,
	PCAP04_UNIT_35,
	PCAP04_UNIT_36,
	PCAP04_UNIT_37,
	PCAP04_UNIT_38,
	PCAP04_UNIT_39,
	PCAP04_UNIT_40,
	PCAP04_UNIT_41,
	PCAP04_UNIT_42,
	PCAP04_UNIT_43,
	PCAP04_UNIT_44,
	PCAP04_UNIT_45,
	PCAP04_UNIT_46,
	PCAP04_UNIT_47,
	PCAP04_UNIT_48,
	PCAP04_UNIT_49,
	PCAP04_UNIT_50,
	PCAP04_UNIT_51,
	PCAP04_UNIT_52,
	PCAP04_UNIT_53,
	PCAP04_UNIT_54,
	PCAP04_UNIT_55,
	PCAP04_UNIT_56,
	PCAP04_UNIT_57,
	PCAP04_UNIT_58,
	PCAP04_UNIT_59,
	PCAP04_UNIT_60,
	PCAP04_UNIT_61,
	PCAP04_UNIT_62,
	PCAP04_UNIT_63,
	PCAP04_UNITS
} PCAP04_UNIT;


void Pc_Comm_App_Processing(uint8_t *ptr, uint16_t BufferLength);
void PcCommAppLayerBuildResponseMsg(uint8_t *payloadPtr, uint8_t payloadLength,
		uint8_t freeMemoryFlag);

void PcCommAppLayerSendCommResponse(uint8_t result);
void PcCommAppLayerSendCommString(char* responseString);


extern Pc_Comm_App_Data PcCommAppLayerData;

#endif /* INC_PC_COMM_APP_H_ */
