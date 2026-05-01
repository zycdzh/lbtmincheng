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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef unsigned char uint8; 

typedef signed char int8; 

typedef unsigned short uint16; 

typedef signed short int16; 

typedef unsigned int uint32; 

typedef signed int int32;

typedef float fp32; 

typedef double fp64; 	
	
extern uint32_t Time_Base; 	
#include "bsp_uart.h"	
#include "led.h"
#include "tim.h"
#include "can_function.h" 
#include "beep.h"
#include "chassis.h"
#include "friction.h"
#include "pid_init.h"
#include "referee_usart_task.h"
#include "cmsis_os.h"
#include "bsp_rc.h"
#include "gyroscope.h"
#include "gimbal.h"
#include "struct_typedef.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "referee.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "printf.h"
#include "detect_task.h"
#include "servo.h"

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
#define R_LED_Pin GPIO_PIN_11
#define R_LED_GPIO_Port GPIOE
#define G_LED_Pin GPIO_PIN_14
#define G_LED_GPIO_Port GPIOF

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
