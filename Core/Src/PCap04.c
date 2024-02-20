/*
 * PCap04.c
 *
 *  Created on: 9 Jan 2023
 *      Author: Michael Grenader & Aviv Almog
 *      Adapting Ofer Frielich pcap04 driver
 */

/*---------------------------------------INCLUDES -------------------------------------------*/
#include "main.h"
#include "stm32f4xx.h"
#include "PCap04.h"
#include "Pc_Comm_App.h"
#include "selector.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- VARIABLES ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
unsigned int pcap04_results[8];
uint8_t  pcap04_status[3];
double capacitance[6];
int unparsedResults[8];
unsigned char Pcap04_standard_configuration[] = {
  	   	0x1D, 0x00, 0x58, 0x10,
		0x10, 0x00, 0x0F, 0x20,
		0x00, 0xD0, 0x07, 0x00,
		0x00, 0x08, 0xFF, 0x03,
		0x00, 0x24, 0x00, 0x00,
		0x00, 0x01, 0x50, 0x30,
		0x73, 0x04, 0x50, 0x08,
		0x5A, 0x00, 0x82, 0x08,
		0x08, 0x00, 0x47, 0x40,
		0x00, 0x00, 0x00, 0x71,
		0x00, 0x00, 0x08, 0x00,
		0x00, 0x00, 0x00, 0x01,
		0x00, 0x00, 0x00, 0x00
};



unsigned char Pcap04_motherfather_configuration[] = {
		0x1D , 0x00 , 0x58 , 0x10,
		0x10 , 0x00 , 0x3F , 0x20,
		0x00 , 0xD0 , 0x07 , 0x00,
		0x00 , 0x08 , 0xFF , 0x03,
		0x00 , 0x24 , 0x00 , 0x00,
		0x00 , 0x01 , 0x50 , 0x30,
		0x73 , 0x04 , 0x50 , 0x08,
		0x5A , 0x00 , 0x82 , 0x08,
		0x08 , 0x00 , 0x47 , 0x40,
		0x00 , 0x00 , 0x00 , 0x71,
		0x00 , 0x00 , 0x08 , 0x00,
		0x00 , 0x00 , 0x00 , 0x01,
		0x00 , 0x00 , 0x00 , 0x00
};

/******************************************************************************
PCAP04 Firmware hex
* standard assembly
******************************************/

unsigned char Pcap04_standard_hex[] = {
0x24, 0x05, 0xA0, 0x01, 0x20, 0x55, 0x42, 0x5C, 0x48, 0xB1, 0x07, 0x92, 0x02, 0x20, 0x13, 0x02,
0x20, 0x93, 0x02, 0xB2, 0x02, 0x78, 0x20, 0x54, 0xB3, 0x06, 0x91, 0x00, 0x7F, 0x20, 0x86, 0x20,
0x54, 0xB6, 0x03, 0x72, 0x62, 0x20, 0x54, 0xB7, 0x00, 0x00, 0x42, 0x5C, 0xA1, 0x00, 0x49, 0xB0,
0x00, 0x49, 0x40, 0xAB, 0x5D, 0x92, 0x1C, 0x90, 0x02, 0x7F, 0x20, 0x86, 0x66, 0x67, 0x76, 0x77,
0x66, 0x7A, 0xCF, 0xCD, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0x7A, 0xDC, 0xE7, 0x41, 0x32, 0xAA,
0x01, 0x99, 0xFD, 0x7B, 0x01, 0x7A, 0xCF, 0xEB, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0x7A, 0xC1,
0xE7, 0x41, 0x32, 0x6A, 0xDE, 0x44, 0x7A, 0xCF, 0xEA, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0x6A,
0xDF, 0x44, 0x7A, 0xC4, 0xE7, 0x41, 0x32, 0xAB, 0x05, 0x7A, 0xC1, 0xE1, 0x43, 0xE0, 0x3A, 0x7A,
0xC0, 0xE1, 0x43, 0xE0, 0x3A, 0x02, 0x7A, 0xCF, 0xE6, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0x7A,
0xEF, 0x44, 0x02, 0x20, 0x9D, 0x84, 0x01, 0x21, 0x2E, 0x21, 0x74, 0x20, 0x37, 0xC8, 0x7A, 0xE7,
0x43, 0x49, 0x11, 0x6A, 0xD4, 0x44, 0x7A, 0xC1, 0xD8, 0xE6, 0x43, 0xE9, 0x44, 0x1C, 0x43, 0x13,
0xAB, 0x63, 0x6A, 0xDE, 0x41, 0xAB, 0x0B, 0x46, 0x46, 0x46, 0x7A, 0xDF, 0xFF, 0xFF, 0xFF, 0xFF,
0xE3, 0x41, 0x32, 0x1C, 0x44, 0xE9, 0x13, 0x6A, 0xD4, 0x13, 0x41, 0xAA, 0xDF, 0x7A, 0xC5, 0xE1,
0x43, 0x49, 0xE0, 0x34, 0x7A, 0xCF, 0xE3, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0xDB, 0xC0, 0x27,
0xE5, 0x6A, 0xDF, 0x43, 0x7A, 0xC8, 0xE7, 0x41, 0x30, 0xAB, 0x03, 0x86, 0x01, 0x92, 0x37, 0x7A,
0xC6, 0xE7, 0x41, 0x7A, 0xFA, 0xE7, 0x43, 0xEA, 0x44, 0x7A, 0xC1, 0xE1, 0xE6, 0x43, 0xE9, 0x44,
0x25, 0xE0, 0x7A, 0xC6, 0xE7, 0x41, 0x7A, 0xFA, 0xE7, 0x43, 0xEA, 0x44, 0x7A, 0xC0, 0xE7, 0x43,
0xE9, 0x44, 0x25, 0xE0, 0x92, 0x10, 0x7A, 0xE1, 0x44, 0xE2, 0x44, 0xE3, 0x44, 0xE4, 0x44, 0xE5,
0x44, 0xE6, 0x44, 0xE7, 0x44, 0xE8, 0x44, 0xC1, 0xD8, 0x24, 0x3E, 0x92, 0xFF, 0x02, 0x7A, 0xCF,
0xD7, 0xE6, 0x43, 0xF1, 0x44, 0x7A, 0xD0, 0xE7, 0x43, 0x2A, 0x2A, 0x32, 0xAB, 0x03, 0x42, 0x5C,
0x92, 0x03, 0x7A, 0xC0, 0xE1, 0x43, 0xD9, 0x27, 0x90, 0x6A, 0xDF, 0x43, 0x7A, 0xC8, 0xE7, 0x41,
0x32, 0xAB, 0x03, 0x86, 0x01, 0x92, 0x11, 0x7A, 0xC2, 0x43, 0x7A, 0xE7, 0x44, 0x6A, 0xC6, 0x44,
0x7A, 0xC3, 0x43, 0x7A, 0xE8, 0x44, 0x6A, 0xC7, 0x44, 0xC1, 0xD4, 0x24, 0x57, 0x7A, 0xC8, 0xE1,
0x43, 0xE0, 0x3A, 0x02, 0x7A, 0xCF, 0xE7, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0x7A, 0xC7, 0xE1,
0x41, 0x6A, 0xD4, 0x45, 0x5A, 0x25, 0x36, 0x46, 0x46, 0x46, 0x46, 0x7A, 0xE9, 0x44, 0x7A, 0xC0,
0xE7, 0x43, 0x55, 0x7A, 0xEA, 0x45, 0x7A, 0xE9, 0x51, 0x1C, 0x43, 0x6A, 0xCA, 0x44, 0x1D, 0x43,
0x6A, 0xCB, 0x44, 0x7A, 0xC1, 0xCA, 0xE6, 0x43, 0xE9, 0x44, 0x7A, 0xC1, 0xE1, 0x43, 0x7A, 0xCC,
0xE0, 0xE6, 0x41, 0x2C, 0x42, 0x7A, 0xC5, 0xE1, 0x43, 0x49, 0xE0, 0x34, 0x7A, 0xC1, 0xCC, 0xE6,
0x43, 0xE9, 0x44, 0x7A, 0xC1, 0xE1, 0x43, 0x2C, 0x70, 0x7A, 0xCC, 0x43, 0x7A, 0xCF, 0x44, 0x7A,
0xCD, 0x43, 0x7A, 0xCE, 0x44, 0x6A, 0xCA, 0x43, 0xC1, 0xCA, 0x7A, 0xE6, 0x41, 0xE9, 0x45, 0x2B,
0xAE, 0xEE, 0x44, 0x7A, 0xC1, 0xCA, 0xE6, 0x43, 0xE9, 0x44, 0x7A, 0xC1, 0xE1, 0x43, 0x7A, 0xCC,
0xEC, 0xE6, 0x41, 0x2C, 0x42, 0x7A, 0xC5, 0xE1, 0x43, 0x49, 0xE0, 0x34, 0x7A, 0xC1, 0xCC, 0xE6,
0x43, 0xE9, 0x44, 0x7A, 0xC1, 0xE1, 0x43, 0x2C, 0x70, 0x7A, 0xCC, 0x43, 0x7A, 0xCF, 0x44, 0x7A,
0xCD, 0x43, 0x7A, 0xCE, 0x44, 0x6A, 0xCB, 0x43, 0xC1, 0xCA, 0x7A, 0xE6, 0x41, 0xE9, 0x45, 0x2B,
0xAE, 0xED, 0x44, 0x02
};



/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------- FUNCTIONS ------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void pcap04_write_command(PCAP04_OPCODE opcode, int pcap04Unit)
{
	setAddressLow(pcap04Unit);

	HAL_SPI_Transmit( &hspi1 , &opcode , 1 , SPI_TIMEOUT );
	setAllSelectors();
}

void pcap04_power_on_reset(int pcap04Unit)
{
	pcap04_write_command(POWER_ON_RESET, pcap04Unit);
}

void pcap04_initialize(int pcap04Unit)
{
	pcap04_write_command(INITIALIZE,  pcap04Unit);
}

void pcap04_cdc_start_conversion(int pcap04Unit)
{
	pcap04_write_command(CDC_START_CONVERSION, pcap04Unit);
}

void pcap04_rdc_start_conversion(int pcap04Unit)
{
	pcap04_write_command(RDC_START_CONVERSION, pcap04Unit );
}

void pcap04_dsp_trig(int pcap04Unit)
{
	pcap04_write_command(DSP_TRIG ,  pcap04Unit);
}

void pcap04_nv_store(int pcap04Unit)
{
	pcap04_write_command(NV_STORE, pcap04Unit );
}

void pcap04_nv_recall(int pcap04Unit)
{
	pcap04_write_command(NV_RECALL, pcap04Unit );
}

void pcap04_nv_erase(int pcap04Unit)
{
	pcap04_write_command(NV_ERASE,pcap04Unit );
}

bool __attribute__((optimize("O0"))) pcap04_read_test(int pcap04Unit)
{
	setAllSelectors();
	HAL_Delay(1);
	setAddressLow(pcap04Unit);
	uint8_t data_rx[2] , data_tx[2] = { TEST_READ , DUMMY_BYTE };

	HAL_SPI_TransmitReceive( &hspi1 , data_tx , data_rx , 2 , SPI_TIMEOUT );

	setAllSelectors();

	return ( ( data_rx[1] == EXPECTED_READ_TEST_BYTE ) ? true : false );
}

void pcap04_write_memory(uint16_t address , uint8_t data, int pcap04Unit )
{
	setAddressLow(pcap04Unit);
	uint8_t data_tx[3] = { WRITE_MEMORY_OPCODE(address) , address & 0xFF , data };
	HAL_SPI_Transmit( &hspi1 , data_tx , 3 , SPI_TIMEOUT );
	setAllSelectors();

}


uint8_t pcap04_read_memory(uint16_t address, int pcap04Unit )

{
	setAddressLow(pcap04Unit);

	uint8_t data_rx[3] , data_tx[3] = { READ_MEMORY_OPCODE(address) , address & 0xFF , DUMMY_BYTE };

	HAL_SPI_TransmitReceive( &hspi1 , data_tx , data_rx , 3 , SPI_TIMEOUT );

	setAllSelectors();

	return data_rx[2];
}

void pcap04_write_configuration( uint16_t address , uint8_t data, int pcap04Unit )
{
	setAddressLow(pcap04Unit);

	uint8_t data_tx[3] = { WRITE_CONFIGURATION , address | 0xC0 , data };

	HAL_SPI_Transmit( &hspi1 , data_tx , 3 , SPI_TIMEOUT );

	setAllSelectors();

}

uint8_t pcap04_read_configuration(uint16_t address, int pcap04Unit )
{
	setAddressLow(pcap04Unit);
	uint8_t data_rx[3] , data_tx[3] = { READ_CONFIGURATION , address | 0xC0 , DUMMY_BYTE };

	HAL_SPI_TransmitReceive( &hspi1 , data_tx , data_rx , 3 , SPI_TIMEOUT );

	setAllSelectors();
	return data_rx[2];
}

uint32_t pcap04_read_result_register(uint8_t result_register, int pcap04Unit )
{
	setAddressLow(pcap04Unit);

	uint8_t data_rx[MAXIMUM_RESULT_BYTES + 1] , data_tx[MAXIMUM_RESULT_BYTES + 1] = { READ_RESULT_RAM | ( result_register << 2 ) };

	HAL_SPI_TransmitReceive( &hspi1 , data_tx , data_rx , 5 , SPI_TIMEOUT );

	setAllSelectors();

	return *( (uint32_t *)&data_rx[1] );
}


void __attribute__((optimize("O0"))) pcap04_read_all_result_registers(unsigned int *result , uint8_t *status,  int pcap04Unit  )
{

	setAddressLow(pcap04Unit);
	uint8_t data_rx[MAXIMUM_RESULT_BYTES + 4] , data_tx[MAXIMUM_RESULT_BYTES + 4] = { READ_RESULT_RAM ,  0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };

	HAL_SPI_TransmitReceive( &hspi1 , data_tx , data_rx , MAXIMUM_RESULT_BYTES + 4 , SPI_TIMEOUT );
	for( uint8_t status_register_index = 0 ; status_register_index < 8 ; status_register_index++ )
		status[status_register_index] = data_rx[ status_register_index + MAXIMUM_RESULT_BYTES + 1 ];
	for( uint8_t result_register_index = 0 ; result_register_index < 8 ; result_register_index++ )
		result[result_register_index] = ( ( data_rx[ 4 * result_register_index + 1] << 0 ) | ( data_rx[ 4 * result_register_index + 2] << 8 ) | ( data_rx[ 4 * result_register_index + 3] << 16 ) | ( data_rx[ 4 * result_register_index + 4] << 24 ) );
	setAllSelectors();
}

void pcap04_initialization(int pcap04Unit)
{
	// Test SPI communication interface by sending Test Opcode
	for( uint8_t test_index = 0 ; test_index < 3 ; test_index++ )
	{
		if( pcap04_read_test(pcap04Unit) == true )
			break;
	}
	pcap04_initialize(pcap04Unit);
	HAL_Delay(1);
	pcap04_power_on_reset(pcap04Unit);
	HAL_Delay(1);



	setAddressLow(pcap04Unit);
	uint8_t memoryWriteTx[2] = {WRITE_MEMORY, 0x00};
	HAL_SPI_Transmit( &hspi1 , memoryWriteTx , 2 , SPI_TIMEOUT );
	for( uint16_t memory_register_index = 0 ; memory_register_index < sizeof(Pcap04_standard_hex) ; memory_register_index++ ) {
		HAL_SPI_Transmit( &hspi1 , &Pcap04_standard_hex[memory_register_index] , 1 , SPI_TIMEOUT );

	}

	setAllSelectors();

	HAL_Delay(1);
	setAddressLow(pcap04Unit);

	uint8_t configWriteTx[2] = {WRITE_CONFIGURATION, 0xC0};
	HAL_SPI_Transmit( &hspi1 , configWriteTx , 2 , SPI_TIMEOUT );
	for( uint8_t configuration_register_index = 0 ; configuration_register_index < sizeof(Pcap04_motherfather_configuration) ; configuration_register_index++ )
	{
		HAL_SPI_Transmit( &hspi1 , &Pcap04_motherfather_configuration[configuration_register_index] , 1 , SPI_TIMEOUT );
	}

	setAllSelectors();


	HAL_Delay(1);
	pcap04_cdc_start_conversion(pcap04Unit);
	HAL_Delay(1);
}


void pcap04_read_results(int pcap04Unit)
{
		pcap04_read_all_result_registers(pcap04_results , pcap04_status, pcap04Unit ); // first reading has transfer effect
		pcap04_read_all_result_registers(pcap04_results , pcap04_status, pcap04Unit ); // so reading again
}

void pcap04_read_results_debug(int pcap04Unit)
{
	uint8_t pcap_string[120];

	for(int count = 1; count < 10; count++)
	{
		pcap04_read_all_result_registers(pcap04_results , pcap04_status, pcap04Unit );
		HAL_Delay(250);
		for( uint8_t register_index = 0 ; register_index < 8 ; register_index++ )
		{
			sprintf( (char *)pcap_string , "0x%08X    " , (int)pcap04_results[register_index] );
			PcCommAppLayerSendCommString(pcap_string);
		}
		HAL_Delay(100);
		///////////////////////////////////////////////////////////////////////
		for( uint8_t register_index = 0 ; register_index < 8 ; register_index++ )
		{
			if( register_index < 6 )
				printf( (char *)pcap_string , "%.7f    " , 319.999995 * pcap04_results[register_index] / 4294967295.0 );
			else
				printf( (char *)pcap_string , "%.7f    " , 31.9999995 * pcap04_results[register_index] / 4294967295.0 );
			PcCommAppLayerSendCommString(pcap_string);
		}
		HAL_Delay(100);
	}

}

void pcap04_get_results()
{
	char pcap04_results_string[200];
	sprintf( pcap04_results_string , "%u %u %u %u %u %u %u %u\n" , pcap04_results[0] , pcap04_results[1] , pcap04_results[2] , pcap04_results[3] , pcap04_results[4] , pcap04_results[5] , pcap04_results[6] , pcap04_results[7] );
	PcCommAppLayerSendCommString(pcap04_results_string);

}

uint8_t pcap04_convert_to_byte(uint8_t *ascii_number)
{
	uint8_t high_byte = ascii_number[0];
	uint8_t  low_byte = ascii_number[1];
	if( high_byte <= '9' && high_byte >= '0' )
		high_byte = high_byte - '0';
	else
		high_byte = high_byte - 'A' + 10;
	if( low_byte <= '9' && low_byte >= '0' )
		low_byte = low_byte - '0';
	else
		low_byte = low_byte - 'A' + 10;
	return ( high_byte * 16 + low_byte );
}

void pcap04_set_configuration_registers(uint8_t *configuration_registers_string, int pcap04Unit)
{
	for( uint8_t configuration_register_index = 0 ; configuration_register_index < sizeof(Pcap04_motherfather_configuration) - 4 ; configuration_register_index++ )
	{
		uint8_t register_value = pcap04_convert_to_byte( &configuration_registers_string[ 2 * configuration_register_index ] );
		pcap04_write_configuration(CFG_REGISTER(configuration_register_index) , register_value, pcap04Unit );
		HAL_Delay(1);
	}

}


void parseResults(int *unparsedResult, double *capacitance) {
	for(int i = 0; i < 6; i++) {
	static int result[2];
    result[0] = (unparsedResult[i] >> 27) & 0x1F; // First 5 bits
    result[1] = unparsedResult[i] & 0x7FFFFFF;     // Last 27 bits
    double decimalPart  = (double)result[1] / (2<<26);
    capacitance[i] = 5.6*(result[0] + decimalPart);
	}
}


