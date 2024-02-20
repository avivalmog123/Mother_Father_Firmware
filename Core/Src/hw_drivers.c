/*
 * hw_drivers.c
 *
 *  Created on: Jan 12, 2023
 *      Author: Michael Grenader
 */
#include "hw_drivers.h"
#include "main.h"

/******************************************************************************
 *** @brief  void hwdriversGpioSet(GPIO_TypeDef* GPIOx, uint16_t gpioPin, uint8_t pinState)
 *   @param none
 *   @retval void
 ******************************************************************************/
void hwGpioSet(GPIO_TypeDef *GPIOx, uint16_t gpioPin, uint8_t pinState) {

	if (GPIOx == GPIOA) {
		switch (gpioPin) {
//		case EXIN_AN0_Pin:
//			PcCommAppLayerData.EXIN_AN0 = pinState;
//			break;
//		case EXIN_AN1_Pin:
//			pccpmmAppLayerData.EXIN_AN1 = pinState;
//			break;
//		case EX_CS_Pin:
//			pccpmmAppLayerData.EX_CS = pinState;
//			break;
//		case SL_SD_Pin:
//			PcCommAppLayerData.SL_SD = pinState;
//			break;
//		case EXIN_GP4_Pin:
//			pccpmmAppLayerData.EXIN_GP4 = pinState;
//			break;
//		case EX_EN_Pin:
//			pccpmmAppLayerData.EX_EN = pinState;
//			break;
		default:
			break;
		}

	} else if (GPIOx == GPIOB) {
		switch (gpioPin) {
//		case SIG3_Pin:
//			PcCommAppLayerData.SIG3 = pinState;
//			break;
//		case set_HUMID_NRST_Pin:
//			pccpmmAppLayerData.set_HUMID_NRST = pinState;
//			break;
//		case SIG1_Pin:
//			PcCommAppLayerData.SIG1 = pinState;
//			break;
		default:
			break;
		}
	} else if (GPIOx == GPIOC) {
		switch (gpioPin) {
//		case EXIN_GP5_Pin:
//			pccpmmAppLayerData.EXIN_GP5 = pinState;
//			break;
//		case EXT_CLOCK_Pin:
//			PcCommAppLayerData.EXT_CLOCK = pinState;
//			break;
//		case EXT_SCK_Pin:
//			pccpmmAppLayerData.EXT_SCK = pinState;
//			break;
//		case EXT_MISO_Pin:
//			pccpmmAppLayerData.EXT_MISO = pinState;
//			break;
//		case EXT_MOSI_Pin:
//			pccpmmAppLayerData.EXT_MOSI = pinState;
//			break;
		default:
			break;
		}
	} else if (GPIOx == GPIOD) {
		switch (gpioPin) {
//		case SIG2_Pin:
//			PcCommAppLayerData.SIG2 = pinState;
//			break;
//		case EXIN_GP9_Pin:
//			PcCommAppLayerData.EXIN_GP9 = pinState;
//			break;
//		case EXIN_GP8_Pin:
//			PcCommAppLayerData.EXIN_GP8 = pinState;
//			break;
//		case EXIN_GP7_Pin:
//			PcCommAppLayerData.EXIN_GP7 = pinState;
//			break;
//		case EXIN_GP6_Pin:
//			PcCommAppLayerData.EXIN_GP6 = pinState;
//			break;
//		case POWER_FAULT_Pin:
//			pccpmmAppLayerData.POWER_FAULT = pinState;
//			break;
		default:
			break;
		}
	} else if (GPIOx == GPIOE) {
		switch (gpioPin) {
//		case EXIN_GP0_Pin:
//			PcCommAppLayerData.EXIN_GP0 = pinState;
//			break;
//		case EXIN_GP1_Pin:
//			pccpmmAppLayerData.EXIN_GP1 = pinState;
//			break;
//		case EXIN_GP2_Pin:
//			PcCommAppLayerData.EXIN_GP2 = pinState;
//			break;
//		case EXIN_GP3_Pin:
//			pccpmmAppLayerData.EXIN_GP3 = pinState;
//			break;
//		case set_DPS_RST_Pin:
//			PcCommAppLayerData.set_DPS_RST = pinState;
//			break;
//		case set_OE_Pin:
//			PcCommAppLayerData.set_OE = pinState;
//			break;
//		case set_SR_CLR_Pin:
//			PcCommAppLayerData.set_SR_CLR = pinState;
//			break;
//		case SL_SS_Pin:
//			PcCommAppLayerData.SL_SS = pinState;
//			break;
//		case SL_SP_Pin:
//			PcCommAppLayerData.SL_SP = pinState;
//			break;
		default:
			break;
		}
	}

	HAL_GPIO_WritePin(GPIOx, gpioPin, pinState);
}
