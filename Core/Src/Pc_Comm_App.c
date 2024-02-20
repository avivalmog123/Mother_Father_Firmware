/*
 * Pc_Comm_App.c
 *
 *  Created on: Jan 12, 2023
 *      Author: Michael Grenader & Aviv Almog
 */


#include <Pc_Comm_App.h>
#include <string.h>
#include "sht3x.h"
#include "PCap04.h"
#include "selector.h"
#include "LPS22HB.h"


uint8_t PcCommAppLayerPayloadBuffer[PC_COM_APP_RET_BUFFER_SIZE] = { 0 };
uint8_t PcCommAppLayerMessageBuffer[PC_COM_APP_RET_BUFFER_SIZE] = { 0 };



// uint8_t pccpmmAppLayerRegistersArray[VMIC_NUMBER_OF_REGISTERS] = { 0 };
extern sht3x_handle_t sht3x;
extern LPS22HB_t lps22hb;
Pc_Comm_App_Data PcCommAppLayerData;
uint16_t PcCommAppLayerDataStructSize = sizeof(Pc_Comm_App_Data);

uint32_t requestedUID = 0;
uint16_t requestedTestCfg = 0;
uint16_t Test = 0;
HAL_StatusTypeDef returnStatus;
char sendChar = 'T';
char sendTest[] = "hello world";
extern int pcap04_results[8];
double capacitance[6];
char capacitancesString[6][100];



/******************************************************************************
 * @brief  void Pc_Comm_App_Processing( uint8_t * ptr, uint16_t BufferLength)
 * Update Function Later
 * @param
 * @retval
 ******************************************************************************/
void Pc_Comm_App_Processing(uint8_t *ptr, uint16_t BufferLength) {
	float sht3_temperature, lps22_temperature, humidity, pressure;
	uint16_t address;
	uint8_t data;
	uint8_t pcap04Unit;
	PcCommAppLayerStruct_T *message = (PcCommAppLayerStruct_T*) (ptr);
	// Data length variable
	uint8_t dataLength = (message->dllMsb * 256) + (message->dllLsb);
	char temp_string[100], humid_string[100], pressure_string[100];

	// uint16_t read;
	// Switch case of mother father functions, will add more detail later
	switch ((Pc_Comm_App_Opcode) message->opcode) {
	case SET_SELECTOR_EN:
		pcap04Unit = message->data[0];
		setAddressLow(pcap04Unit);

		break;
	// You can get temp measurement from sht3 or from lps22 sensor
	case GET_SHT3X_TEMP_MEAS:

		sht3x_read_temperature_and_humidity(&sht3x, &sht3_temperature, &humidity);
		// Example Temperature = 25.0123
		char *tmpSign = (sht3_temperature < 0) ? "-" : "";
		float tmpVal = (sht3_temperature < 0) ? -sht3_temperature : sht3_temperature;

		int tmpInt1 = tmpVal;                  // Get the integer (25).
		float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
		int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

		// Print as parts, note that you need 0-padding for fractional bit.

		//sprintf (temp_string, "temp_read = %s%d.%04d ", tmpSign, tmpInt1, tmpInt2);
		//sprintf( (char *)temp_string , "0x%08X    " , temperature );

		gcvt(sht3_temperature, 6, temp_string);
		PcCommAppLayerSendCommString(temp_string);
		break;

	case GET_HUMID_MEAS:
		sht3x_read_temperature_and_humidity(&sht3x, &sht3_temperature, &humidity);
		//sprintf( (char *)humid_string , "0x%08X    " , humidity );
		gcvt(humidity, 6, humid_string);
		PcCommAppLayerSendCommString(humid_string);
		break;

	case SET_HUMID_NRST:


		break;

	case TEST:
		returnStatus = HAL_OK;
		// Send status result
		PcCommAppLayerSendCommString(sendTest);
	//	PcCommAppLayerSendCommResponse(sendTest);
		break;
	case PCAP04_WRITE_MEMORY:

		address = message->data[0];
		data = message->data[1];
		pcap04Unit = message->data[2];
		pcap04_write_memory(address, data, pcap04Unit);

		break;

	case PCAP04_READ_MEMORY:
		address = message->data[0];
		pcap04Unit = message->data[1];
		pcap04_read_memory(address,pcap04Unit);

		break;

	case PCAP04_WRITE_CONFIG:
		address = message->data[0];
		data = message->data[1];
		pcap04Unit = message->data[2];
		pcap04_write_configuration(address, data, pcap04Unit);

		break;
	case PCAP04_READ_CONFIG:
		address = message->data[0];
		pcap04Unit = message->data[1];
		pcap04_read_configuration(address, pcap04Unit);

		break;

	case PCAP04_READ_RESULTS_RAM:
		pcap04Unit = message->data[0];
		int capacitanceToSend = message->data[1];

		pcap04_initialization(pcap04Unit);


		pcap04_read_results(pcap04Unit);

		 parseResults(pcap04_results, capacitance);


		 for(int i=0; i < 6; i++){

			 gcvt(capacitance[i], 15, capacitancesString[i]);

		 }
		PcCommAppLayerSendCommString(capacitancesString[capacitanceToSend]);

		break;

	case PCAP04_POWER_ON_RESET:
		pcap04Unit = message->data[0];
		pcap04_power_on_reset(pcap04Unit);

		break;
	case PCAP04_INITIALIZE:
		pcap04Unit = message->data[0];
		pcap04_initialization(pcap04Unit);

		break;

	case PCAP04_CDC_START_CONVERSION:
		pcap04Unit = message->data[0];
		pcap04_cdc_start_conversion(pcap04Unit);

		break;

	case PCAP04_RDC_START_CONVERSION:
		pcap04Unit = message->data[0];
		pcap04_rdc_start_conversion(pcap04Unit);

		break;
	case PCAP04_NV_STORE:
		pcap04Unit = message->data[0];
		pcap04_nv_store(pcap04Unit);

		break;
	case PCAP04_NV_RECALL:
		pcap04Unit = message->data[0];
		pcap04_nv_recall(pcap04Unit);

		break;

	case PCAP04_NV_ERASE:
		pcap04Unit = message->data[0];
		pcap04_nv_erase(pcap04Unit);

		break;

	case GET_LPS22_TEMP_MEAS:
		LPS22HB_Get_Temperature(&lps22hb);
		break;
	case GET_PRESSURE_MEAS:
	LPS22HB_Update_Data(&lps22hb);
	//	lps22_temperature = lps22hb -> temperature;
	pressure = lps22hb.pressure;
	gcvt(pressure, 6, pressure_string);

	// Do the conversion to send it as string
	PcCommAppLayerSendCommString(pressure_string);
		break;
	}
}
/******************************************************************************
 * @brief  void PcCommAppLayerBuildResponseMsg(uint8_t *payloadPtr, uint8_t payloadLength,
		uint8_t freeMemoryFlag)
 * @param
 * @retval
 ******************************************************************************/

void PcCommAppLayerBuildResponseMsg(uint8_t *payloadPtr, uint8_t payloadLength,
		uint8_t freeMemoryFlag) {
	// Build message
	uint16_t messageLength = sizeof(DllHeader_t) + payloadLength
			+ sizeof(DllEndOfMessage_t);
	// DLL header
	DllHeader_t dllHeader;
	dllHeader.Sync0 = 'V';
	dllHeader.Sync1 = 'E';
	dllHeader.PayloadLength = payloadLength;
	//Dll End Of Message
	DllEndOfMessage_t dllEOM;
	dllEOM.CRC32 = crc32BuffCalc(payloadPtr, 0, payloadLength);

	// Copy dll header to message buffer
	memcpy(PcCommAppLayerMessageBuffer, &dllHeader, sizeof(dllHeader));
	// Copy payload to message buffer
	memcpy(PcCommAppLayerMessageBuffer + sizeof(dllHeader), payloadPtr,
			payloadLength);
	// Copy dll end of message
	memcpy(PcCommAppLayerMessageBuffer + sizeof(dllHeader) + payloadLength,
			&dllEOM, sizeof(dllEOM));
	// Send Message to PC
	usbBufferTx(PcCommAppLayerMessageBuffer, messageLength);
}


/******************************************************************************
 * @brief  void PcCommAppLayerSendCommResponse(uint8_t result)
 * Send response to PC of 1 byte
 * @param
 * @retval
 ******************************************************************************/
void PcCommAppLayerSendCommResponse(uint8_t result){
	uint8_t returnData[1] = { result };
	PcCommAppLayerBuildResponseMsg((uint8_t*) &returnData, 1, 0);
}
/******************************************************************************
 * @brief  void PcCommAppLayerSendCommString(char* responseString)
 * Send response of string to PC
 * @param
 * @retval
 ******************************************************************************/
void PcCommAppLayerSendCommString(char* responseString){
	uint8_t* returnData = (uint8_t*)responseString;
	uint8_t payloadLength = strlen(responseString);
	PcCommAppLayerBuildResponseMsg(returnData, payloadLength, 0);

}

void PcCommAppLayerSendCommDouble(double result){
	double returnData[1] = { result };
	PcCommAppLayerBuildResponseMsg((double*) &returnData, 1, 0);
}
