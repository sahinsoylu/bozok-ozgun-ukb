/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f2xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "pressure/pressure.h"
#include "gps/gps.h"
#include "types.h"
#include "icm/imu.h"
#include "FreeRTOS.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart2;
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
#define UART_RX_DMA_BUFFER_SIZE 64 // En büyük paket (34) için yeterli, fazlasıyla büyük bir buffer
extern uint8_t huart4_rx_dma_buffer[UART_RX_DMA_BUFFER_SIZE]; // Bu, değişkenin BİLDİRİMİ!
extern uint16_t received_data_len;
extern float data_test;
extern enum UKB_MODE ukb_mode;
extern SemaphoreHandle_t sutDataReadySemaphore;
extern volatile FlightData_t g_flightData;
extern SystemControl_t g_systemControl;
void reset_state(void);

// Test LED'leri için GPIO tanımlamaları
// Bunları kendi pinlerinize göre ayarlayın (örneğin PB1 ve PB2)
#define LED_5BYTE_GPIO_Port GPIOB
#define LED_5BYTE_Pin GPIO_PIN_1

#define LED_34BYTE_GPIO_Port GPIOB
#define LED_34BYTE_Pin GPIO_PIN_2

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M0_Pin GPIO_PIN_15
#define M0_GPIO_Port GPIOA
#define M1_Pin GPIO_PIN_3
#define M1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
