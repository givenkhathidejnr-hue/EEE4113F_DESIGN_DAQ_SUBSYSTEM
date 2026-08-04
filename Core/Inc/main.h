/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define TEST_PIN_Pin GPIO_PIN_0
#define TEST_PIN_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define SD_EN_Pin GPIO_PIN_6
#define SD_EN_GPIO_Port GPIOA
#define SD_FAULT_Pin GPIO_PIN_7
#define SD_FAULT_GPIO_Port GPIOA
#define SD_FAULT_EXTI_IRQn EXTI9_5_IRQn
#define SPI2_CS_Pin GPIO_PIN_1
#define SPI2_CS_GPIO_Port GPIOB
#define INA_SDA_Pin GPIO_PIN_11
#define INA_SDA_GPIO_Port GPIOB
#define INA_SCL_Pin GPIO_PIN_13
#define INA_SCL_GPIO_Port GPIOB
#define EXC_EN_Pin GPIO_PIN_7
#define EXC_EN_GPIO_Port GPIOC
#define DET_EN_Pin GPIO_PIN_8
#define DET_EN_GPIO_Port GPIOA
#define DET_FAULT_Pin GPIO_PIN_9
#define DET_FAULT_GPIO_Port GPIOA
#define DET_FAULT_EXTI_IRQn EXTI9_5_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define TEMP_SMBA_Pin GPIO_PIN_5
#define TEMP_SMBA_GPIO_Port GPIOB
#define EXC_FAULT_Pin GPIO_PIN_6
#define EXC_FAULT_GPIO_Port GPIOB
#define EXC_FAULT_EXTI_IRQn EXTI9_5_IRQn
#define TEMP_SCL_Pin GPIO_PIN_8
#define TEMP_SCL_GPIO_Port GPIOB
#define TEMP_SDA_Pin GPIO_PIN_9
#define TEMP_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SD_CS_Pin			SPI2_CS_Pin
#define SD_CS_GPIO_Port		SPI2_CS_GPIO_Port

extern SPI_HandleTypeDef 	hspi2;
#define SD_SPI_HANDLE		hspi2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
