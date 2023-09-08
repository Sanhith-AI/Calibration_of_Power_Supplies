/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32h7xx_hal.h"

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
#define uC_OK_Pin GPIO_PIN_11
#define uC_OK_GPIO_Port GPIOF
#define uC_BUSY_Pin GPIO_PIN_12
#define uC_BUSY_GPIO_Port GPIOF
#define uC_ERR_Pin GPIO_PIN_13
#define uC_ERR_GPIO_Port GPIOF
#define MOD1_STRB_Pin GPIO_PIN_8
#define MOD1_STRB_GPIO_Port GPIOH
#define MOD2_STRB_Pin GPIO_PIN_9
#define MOD2_STRB_GPIO_Port GPIOH
#define MOD3_STRB_Pin GPIO_PIN_10
#define MOD3_STRB_GPIO_Port GPIOH
#define MOD4_STRB_Pin GPIO_PIN_11
#define MOD4_STRB_GPIO_Port GPIOH
#define MOD5_STRB_Pin GPIO_PIN_12
#define MOD5_STRB_GPIO_Port GPIOH
#define MOD1_DEN_Pin GPIO_PIN_2
#define MOD1_DEN_GPIO_Port GPIOG
#define MOD2_DEN_Pin GPIO_PIN_3
#define MOD2_DEN_GPIO_Port GPIOG
#define MOD3_DEN_Pin GPIO_PIN_4
#define MOD3_DEN_GPIO_Port GPIOG
#define MOD4_DEN_Pin GPIO_PIN_5
#define MOD4_DEN_GPIO_Port GPIOG
#define MOD5_DEN_Pin GPIO_PIN_6
#define MOD5_DEN_GPIO_Port GPIOG
#define ADC_nRST_Pin GPIO_PIN_15
#define ADC_nRST_GPIO_Port GPIOA
#define MOD1_nCS_Pin GPIO_PIN_0
#define MOD1_nCS_GPIO_Port GPIOD
#define MOD2_nCS_Pin GPIO_PIN_1
#define MOD2_nCS_GPIO_Port GPIOD
#define MOD3_nCS_Pin GPIO_PIN_2
#define MOD3_nCS_GPIO_Port GPIOD
#define MOD4_nCS_Pin GPIO_PIN_3
#define MOD4_nCS_GPIO_Port GPIOD
#define MOD5_nCS_Pin GPIO_PIN_4
#define MOD5_nCS_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
