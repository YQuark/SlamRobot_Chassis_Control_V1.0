/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis_tasks.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t chassisControlTaskHandle;
osThreadId_t encoderUpdateTaskHandle;
osThreadId_t adcMonitorTaskHandle;
osThreadId_t ledTaskHandle;
osThreadId_t upperUartTaskHandle;

const osThreadAttr_t chassisControlTask_attributes = {
  .name = "chassisControl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t encoderUpdateTask_attributes = {
  .name = "encoderUpdate",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t adcMonitorTask_attributes = {
  .name = "adcMonitor",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

const osThreadAttr_t ledTask_attributes = {
  .name = "led",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t upperUartTask_attributes = {
  .name = "upperUart",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE END Variables */
/* Definitions for systemTask */
osThreadId_t systemTaskHandle;
const osThreadAttr_t systemTask_attributes = {
  .name = "systemTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  ChassisTasks_InitHardware();

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of systemTask */
  systemTaskHandle = osThreadNew(StartDefaultTask, NULL, &systemTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  chassisControlTaskHandle = osThreadNew(Task_ChassisControl, NULL, &chassisControlTask_attributes);
  encoderUpdateTaskHandle = osThreadNew(Task_EncoderUpdate, NULL, &encoderUpdateTask_attributes);
  adcMonitorTaskHandle = osThreadNew(Task_AdcMonitor, NULL, &adcMonitorTask_attributes);
  upperUartTaskHandle = osThreadNew(Task_UpperUart, NULL, &upperUartTask_attributes);
  ledTaskHandle = osThreadNew(Task_Led, NULL, &ledTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the systemTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
