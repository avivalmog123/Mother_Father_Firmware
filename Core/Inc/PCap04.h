/*
 * PCap04.h
 *
 *  Created on: 9 Jan 2023
 *      Author: Michael Grenader
 */


#ifndef INC_PCAP04_H_
#define INC_PCAP04_H_




/*-------------------------------------------------------------------------------------------*/
/*------------------------- DEFINITIONS AND ENUMARTIONS -------------------------------------*/
/*-------------------------------------------------------------------------------------------*/

#define CFG_REGISTER(register_number)	register_number
#define MEM_address(mem_addr_number)	mem_addr_number
#define DUMMY_BYTE						0x00
#define SPI_TIMEOUT						500
#define EXPECTED_READ_TEST_BYTE			0x11
#define MAXIMUM_RESULT_BYTES			32
#define WRITE_MEMORY_OPCODE(address)	( WRITE_MEMORY | ( (address >> 8 ) & 0x03 ) )
#define READ_MEMORY_OPCODE(address)		( READ_MEMORY  | ( (address >> 8 ) & 0x03 ) )

typedef enum
{
	WRITE_MEMORY 			= 0xA0,
	READ_MEMORY				= 0x20,
	WRITE_CONFIGURATION		= 0xA3,  // A3C0
	READ_CONFIGURATION		= 0x23,  // 23C0
	READ_RESULT_RAM			= 0x40,
	POWER_ON_RESET			= 0x88,
	INITIALIZE				= 0x8A,
	CDC_START_CONVERSION	= 0x8C,
	RDC_START_CONVERSION	= 0x8E,
	DSP_TRIG				= 0x8D,
	NV_STORE				= 0x96,
	NV_RECALL				= 0x99,
	NV_ERASE				= 0x9C,
	TEST_READ				= 0x7E
} PCAP04_OPCODE;






/*-------------------- function prototypes ------------------------------------*/
void pcap04_initialization(int pcap04Unit);
void pcap04_read_results(int pcap04Unit);
void pcap04_get_results();
void pcap04_set_configuration_registers(uint8_t *configuration_registers_string, int pcap04Unit);
void pcap04_set_units_existance(char *units_existance);
void pcap04_set_stabilization_time_after_config(int stabilization_time_after_config);
void pcap04_write_command(PCAP04_OPCODE opcode, int pcap04Unit);
void pcap04_set_configuration_registers(uint8_t *configuration_registers_string, int pcap04Unit);


#endif /* INC_PCAP04_H_ */
