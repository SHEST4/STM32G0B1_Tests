/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define PWR_GSM_Pin GPIO_PIN_11
#define PWR_GSM_GPIO_Port GPIOC
#define PWR_KEY_Pin GPIO_PIN_12
#define PWR_KEY_GPIO_Port GPIOC
#define USB_Test_Pin GPIO_PIN_1
#define USB_Test_GPIO_Port GPIOC
#define USB_Test_EXTI_IRQn EXTI0_1_IRQn
#define ADC_PWR_EN_Pin GPIO_PIN_5
#define ADC_PWR_EN_GPIO_Port GPIOC
#define ADC_ACC_Pin GPIO_PIN_0
#define ADC_ACC_GPIO_Port GPIOB
#define ADC_ACC_EXTI_IRQn EXTI0_1_IRQn
#define LED_G_Pin GPIO_PIN_2
#define LED_G_GPIO_Port GPIOB
#define ADC_PWR_Pin GPIO_PIN_10
#define ADC_PWR_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_12
#define LED_B_GPIO_Port GPIOB
#define ACCEL_SDA_Pin GPIO_PIN_14
#define ACCEL_SDA_GPIO_Port GPIOB
#define INT2_Pin GPIO_PIN_15
#define INT2_GPIO_Port GPIOB
#define INT2_EXTI_IRQn EXTI4_15_IRQn
#define INT1_Pin GPIO_PIN_8
#define INT1_GPIO_Port GPIOA
#define INT1_EXTI_IRQn EXTI4_15_IRQn
#define ACCEL_SCL_Pin GPIO_PIN_9
#define ACCEL_SCL_GPIO_Port GPIOA
#define ACCEL_PULL_UP_Pin GPIO_PIN_9
#define ACCEL_PULL_UP_GPIO_Port GPIOD
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VRTC_Pin GPIO_PIN_9
#define VRTC_GPIO_Port GPIOC
#define VCC_GSM_DIS_Pin GPIO_PIN_4
#define VCC_GSM_DIS_GPIO_Port GPIOD
#define PPS_Pin GPIO_PIN_3
#define PPS_GPIO_Port GPIOB
#define PWR_GPS_EN_Pin GPIO_PIN_5
#define PWR_GPS_EN_GPIO_Port GPIOB
#define CPU_TX_GPS_RX_Pin GPIO_PIN_6
#define CPU_TX_GPS_RX_GPIO_Port GPIOB
#define CPU_RX_GPS_TX_Pin GPIO_PIN_7
#define CPU_RX_GPS_TX_GPIO_Port GPIOB
#define CPU_RX_GSM_TX_Pin GPIO_PIN_9
#define CPU_RX_GSM_TX_GPIO_Port GPIOB
#define CPU_TX_GSM_RX_Pin GPIO_PIN_10
#define CPU_TX_GSM_RX_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
