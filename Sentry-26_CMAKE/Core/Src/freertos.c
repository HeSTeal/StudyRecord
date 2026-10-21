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
#include "controller.h"
#include "robot.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "dma.h"
#include "SMC.h"
#include "robot_def.h"
#include "DMmotor.h"
#include "DJImotor.h"
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
#ifdef CHASSIS_BOARD
// extern Chassis_Upload_Data_s chassis_feedback_data __attribute__((aligned(4)));
// extern float chassis_vx, chassis_vy,chassis_vw;  
#endif
#ifdef GIMBAL_BOARD
extern DMMotorInstance   *yaw_motor, *big_pitch_motor, *small_pitch_motor;
extern DJIMotorInstance *friction_l, *friction_r, *loader;
#endif
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for INSTask */
osThreadId_t INSTaskHandle;
const osThreadAttr_t INSTask_attributes = {
  .name = "INSTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RobotTask */
osThreadId_t RobotTaskHandle;
const osThreadAttr_t RobotTask_attributes = {
  .name = "RobotTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for VOFATask */
osThreadId_t VOFATaskHandle;
const osThreadAttr_t VOFATask_attributes = {
  .name = "VOFATask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RefreeTask */
osThreadId_t RefreeTaskHandle;
const osThreadAttr_t RefreeTask_attributes = {
  .name = "RefreeTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartINSTask(void *argument);
void StartMotorTask(void *argument);
void StartRobotTask(void *argument);
void StartVOFATask(void *argument);
void StartRefreeTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of INSTask */
  INSTaskHandle = osThreadNew(StartINSTask, NULL, &INSTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of RobotTask */
  RobotTaskHandle = osThreadNew(StartRobotTask, NULL, &RobotTask_attributes);

  /* creation of VOFATask */
  VOFATaskHandle = osThreadNew(StartVOFATask, NULL, &VOFATask_attributes);

  /* creation of RefreeTask */
  RefreeTaskHandle = osThreadNew(StartRefreeTask, NULL, &RefreeTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartINSTask */
/**
* @brief Function implementing the INSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartINSTask */
__weak void StartINSTask(void *argument)
{
  /* USER CODE BEGIN StartINSTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartINSTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
__weak void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartRobotTask */
/**
* @brief Function implementing the RobotTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRobotTask */
__weak void StartRobotTask(void *argument)
{
  /* USER CODE BEGIN StartRobotTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRobotTask */
}

/* USER CODE BEGIN Header_StartVOFATask */
/**
* @brief Function implementing the VOFATask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVOFATask */
void StartVOFATask(void *argument)
{
  /* USER CODE BEGIN StartVOFATask */
  /* Infinite loop */
  for(;;)
  {
    #ifdef CHASSIS_BOARD
    // static char buff[150];
		// sprintf(buff,"0.0,%f\r\n",chassis_feedback_data.real_wz);
		// HAL_UART_Transmit_DMA(&huart5,(uint8_t *)buff,strlen(buff));
    #endif
    #ifdef GIMBAL_BOARD
    // if (HAL_UART_GetState(&huart5) == HAL_UART_STATE_READY)
    // {
    //     static char buff[150];
    //     int len = sprintf(buff, "%f,%f,%f,%f\r\n",loader->motor_controller.angle_PID.Ref,
    //       loader->motor_controller.angle_PID.Measure,
    //       loader->motor_controller.speed_PID.Ref,
    //       loader->motor_controller.speed_PID.Measure);
    //     HAL_UART_Transmit_DMA(&huart5, (uint8_t*)buff, len);
    // }
    #endif
    osDelay(1);
  }
  /* USER CODE END StartVOFATask */
}

/* USER CODE BEGIN Header_StartRefreeTask */
/**
* @brief Function implementing the RefreeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRefreeTask */
__weak void StartRefreeTask(void *argument)
{
  /* USER CODE BEGIN StartRefreeTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRefreeTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

