/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern void chassis_task(void *argument);
extern void gimbal_task(void *argument);
extern void friction_task(void *argument);
extern void referee_usart_task(void *argument);
extern void detect_task(void *argument);
extern void servo_task(void *argument);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
TaskHandle_t   chassis_Handle;
TaskHandle_t   gimbal_Handle;
TaskHandle_t   friction_Handle;
TaskHandle_t   referee_usart_Handle;
TaskHandle_t   detect_task_Handle;
TaskHandle_t   servo_task_Handle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* 创建互斥锁，... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* 创建信号量，... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* 启动定时器，... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 创建队列，... */
  /* USER CODE END RTOS_QUEUES */

  /* 创建默认任务 */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 在此处添加线程，... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* 添加事件，... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  默认任务入口，在此创建所有应用任务后删除自身
  * @param  argument: 未使用
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* 创建底盘任务 - 电机控制，优先级高于普通 */
  xTaskCreate(
      chassis_task,           // 任务函数
      "chassis",              // 任务名称
      512,                    // 堆栈大小（单位：word）
      NULL,                   // 参数
      osPriorityNormal1,      // 优先级
      &chassis_Handle);       // 任务句柄

  /* 创建云台任务 - 俯仰和偏航控制，最高优先级 */
  xTaskCreate(
      gimbal_task,            // 任务函数
      "gimbal_task",          // 任务名称
      512,                    // 堆栈大小
      NULL,                   // 参数
      osPriorityNormal3,      // 优先级 - 最高，实时控制
      &gimbal_Handle);        // 任务句柄

  /* 创建摩擦轮任务 - 发射控制 */
  xTaskCreate(
      friction_task,          // 任务函数
      "friction_task",       // 任务名称
      256,                    // 堆栈大小
      NULL,                   // 参数
      osPriorityNormal2,      // 优先级 - 高，时序要求
      &friction_Handle);     // 任务句柄

  /* 创建裁判系统通信任务 */
  xTaskCreate(
      referee_usart_task,    // 任务函数
      "referee_usart_task",  // 任务名称
      256,                    // 堆栈大小
      NULL,                   // 参数
      osPriorityNormal1,      // 优先级
      &referee_usart_Handle); // 任务句柄

  /* 创建错误检测任务 */
  xTaskCreate(
      detect_task,           // 任务函数
      "detect_task",         // 任务名称
      256,                   // 堆栈大小
      NULL,                  // 参数
      osPriorityNormal,      // 优先级 - 最低，仅监控
      &detect_task_Handle);  // 任务句柄

  /* 创建舵机任务 */
  xTaskCreate(
      servo_task,            // 任务函数
      "servo_task",          // 任务名称
      256,                   // 堆栈大小
      NULL,                  // 参数
      osPriorityNormal1,      // 优先级
      &servo_task_Handle);   // 任务句柄

  /* 创建完所有任务后删除自身 */
  vTaskDelete(NULL);

  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */