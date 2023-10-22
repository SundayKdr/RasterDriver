/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define NOTUSED_1_IN_Pin GPIO_PIN_1
#define NOTUSED_1_IN_GPIO_Port GPIOF
#define CONFIG_3_Pin GPIO_PIN_0
#define CONFIG_3_GPIO_Port GPIOA
#define CONFIG_2_Pin GPIO_PIN_1
#define CONFIG_2_GPIO_Port GPIOA
#define CONFIG_1_Pin GPIO_PIN_2
#define CONFIG_1_GPIO_Port GPIOA
#define INDICATION_0_OUT_Pin GPIO_PIN_3
#define INDICATION_0_OUT_GPIO_Port GPIOA
#define INDICATION_1_OUT_Pin GPIO_PIN_4
#define INDICATION_1_OUT_GPIO_Port GPIOA
#define IN_MOTION_OUT_Pin GPIO_PIN_5
#define IN_MOTION_OUT_GPIO_Port GPIOA
#define EXP_REQ_IN_Pin GPIO_PIN_6
#define EXP_REQ_IN_GPIO_Port GPIOA
#define GRID_BUTTON_Pin GPIO_PIN_7
#define GRID_BUTTON_GPIO_Port GPIOA
#define GRID_BUTTON_EXTI_IRQn EXTI9_5_IRQn
#define NOTUSED_1_OUT_Pin GPIO_PIN_0
#define NOTUSED_1_OUT_GPIO_Port GPIOB
#define GRID_INFIELD_DETECT_Pin GPIO_PIN_8
#define GRID_INFIELD_DETECT_GPIO_Port GPIOA
#define GRID_HOME_DETECT_Pin GPIO_PIN_9
#define GRID_HOME_DETECT_GPIO_Port GPIOA
#define NOTUSED_0_IN_Pin GPIO_PIN_10
#define NOTUSED_0_IN_GPIO_Port GPIOA
#define NOTUSED_PUSHBUTTON_Pin GPIO_PIN_15
#define NOTUSED_PUSHBUTTON_GPIO_Port GPIOA
#define NOTUSED_PUSHBUTTON_EXTI_IRQn EXTI15_10_IRQn
#define NOTUSED_0_OUT_Pin GPIO_PIN_3
#define NOTUSED_0_OUT_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_4
#define RESET_GPIO_Port GPIOB
#define ENABLE_Pin GPIO_PIN_5
#define ENABLE_GPIO_Port GPIOB
#define CURRENT_WIND_Pin GPIO_PIN_6
#define CURRENT_WIND_GPIO_Port GPIOB
#define STEP_Pin GPIO_PIN_7
#define STEP_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_8
#define DIR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
