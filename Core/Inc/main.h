/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void sensors_init();
/* USER CODE BEGIN Private defines */
#define I2C_DEFAULT_TIMEOUT 1000

#define I2C_LEDS_ADDR	0x21
#define I2C_LEDS_DIR_REG 0x00
#define I2C_LEDS_DATA_REG 0x14
#define I2C_LINESENSOR_DIR_REG 0x01
#define I2C_LINESENSOR_DATA_REG 0x13
/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EXIN_GP2_Pin GPIO_PIN_2
#define EXIN_GP2_GPIO_Port GPIOE
#define EXIN_GP3_Pin GPIO_PIN_3
#define EXIN_GP3_GPIO_Port GPIOE
#define EXIN_GP5_Pin GPIO_PIN_3
#define EXIN_GP5_GPIO_Port GPIOC
#define SIG3_Pin GPIO_PIN_0
#define SIG3_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define DPS_RDY_Pin GPIO_PIN_8
#define DPS_RDY_GPIO_Port GPIOE
#define OE_Pin GPIO_PIN_9
#define OE_GPIO_Port GPIOE
#define SR_CLR_Pin GPIO_PIN_10
#define SR_CLR_GPIO_Port GPIOE
#define DPS_RST_Pin GPIO_PIN_13
#define DPS_RST_GPIO_Port GPIOE
#define SL_SS_Pin GPIO_PIN_14
#define SL_SS_GPIO_Port GPIOE
#define SL_SP_Pin GPIO_PIN_15
#define SL_SP_GPIO_Port GPIOE
#define EXIN_GP7_Pin GPIO_PIN_10
#define EXIN_GP7_GPIO_Port GPIOD
#define EXIN_GP6_Pin GPIO_PIN_11
#define EXIN_GP6_GPIO_Port GPIOD
#define POWER_FAULT_Pin GPIO_PIN_15
#define POWER_FAULT_GPIO_Port GPIOD
#define EXT_CS_Pin GPIO_PIN_9
#define EXT_CS_GPIO_Port GPIOA
#define SL_SD_Pin GPIO_PIN_10
#define SL_SD_GPIO_Port GPIOA
#define EXT_EN_Pin GPIO_PIN_12
#define EXT_EN_GPIO_Port GPIOA
#define SIG2_Pin GPIO_PIN_1
#define SIG2_GPIO_Port GPIOD
#define EXIN_GP9_Pin GPIO_PIN_7
#define EXIN_GP9_GPIO_Port GPIOD
#define EXIN_GP1_Pin GPIO_PIN_1
#define EXIN_GP1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define I2C_DEFAULT_TIMEOUT 1000

#define I2C_LEDS_ADDR	0x21
#define I2C_LEDS_DIR_REG 0x00
#define I2C_LEDS_DATA_REG 0x14
#define I2C_LINESENSOR_DIR_REG 0x01
#define I2C_LINESENSOR_DATA_REG 0x13
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
