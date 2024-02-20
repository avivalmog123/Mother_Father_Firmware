#include "main.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "selector.h"

const int SLOTS = 64;



void setAddressLow(int address){
	int addressBackCall = SLOTS - address;
	for (int i = 0; i < SLOTS; i++)
	{
		if (i == addressBackCall)
		{
			HAL_GPIO_WritePin(GPIOE, SL_SP_Pin, GPIO_PIN_SET);
			delay_ms(ADDRESS_CHOOSE_TICK);
			HAL_GPIO_WritePin(GPIOE, SL_SP_Pin, GPIO_PIN_RESET);
			delay_ms(ADDRESS_CHOOSE_TICK);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, SL_SD_Pin, GPIO_PIN_SET);
			//delay_ms(ADDRESS_CHOOSE_TICK);
			HAL_GPIO_WritePin(GPIOE, SL_SP_Pin, GPIO_PIN_SET);  // CLK
			delay_ms(ADDRESS_CHOOSE_TICK);
			HAL_GPIO_WritePin(GPIOA, SL_SD_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, SL_SP_Pin, GPIO_PIN_RESET);
			delay_ms(ADDRESS_CHOOSE_TICK);
		}
	}
	HAL_GPIO_WritePin(GPIOE, SL_SS_Pin, GPIO_PIN_SET);   // Latch
	delay_ms(ADDRESS_CHOOSE_TICK);
	HAL_GPIO_WritePin(GPIOE, SL_SS_Pin, GPIO_PIN_RESET);
	delay_ms(ADDRESS_CHOOSE_TICK);

}



void setAllSelectors() {
	for (int i = 0; i < SLOTS; i++)
	{
		HAL_GPIO_WritePin(GPIOA, SL_SD_Pin, GPIO_PIN_SET);
		//delay_ms(ADDRESS_CHOOSE_TICK);
		HAL_GPIO_WritePin(GPIOE, SL_SP_Pin, GPIO_PIN_SET);  // CLK
		delay_ms(ADDRESS_CHOOSE_TICK);
		HAL_GPIO_WritePin(GPIOA, SL_SD_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, SL_SP_Pin, GPIO_PIN_RESET);
		delay_ms(ADDRESS_CHOOSE_TICK);

	}
	HAL_GPIO_WritePin(GPIOE, SL_SS_Pin, GPIO_PIN_SET);   // Latch
	delay_ms(ADDRESS_CHOOSE_TICK);
	HAL_GPIO_WritePin(GPIOE, SL_SS_Pin, GPIO_PIN_RESET);
	delay_ms(ADDRESS_CHOOSE_TICK);
}

void delay_ms(volatile int time_ms)
{
	int j;
    for(j = 0; j < time_ms*400; j++)
        {}  /* excute NOP for 1ms */
}
