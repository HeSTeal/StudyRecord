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
#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "usart.h"
#include "DJImotor.h"
#include "stdio.h"
#include "robot_def.h"
extern DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
extern float chassis_vx, chassis_vy,chassis_vw;   // 底盘目标速度
#ifdef CHASSIS_BOARD
extern Navi_Recv_s *navi_recv;
#endif
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
/* NOTE: the 1kHz motor task (StartMotorControlTask, implemented in
   MODULES/motor/motor_task.c) is created by the CubeMX generated "mymotor"
   thread below. Do NOT create it a second time here, otherwise
   DJIMotorControl()/CANTransmit() would run at 2kHz on the same CAN bus. */
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
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for VOFATask */
osThreadId_t VOFATaskHandle;
const osThreadAttr_t VOFATask_attributes = {
  .name = "VOFATask",
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
//		static char ch[120];
//		sprintf(ch,"%.1f,%.1f,%.1f,%.1f\r\n",
//           loader->motor_controller.angle_PID.Ref,
//           loader->measure.total_angle,
//					 loader->motor_controller.speed_PID.Ref,
//					 loader->measure.speed_aps);
//		HAL_UART_Transmit(&huart5,(uint8_t*)ch,strlen(ch),0xff);
		#ifdef CHASSIS_BOARD

		static char buff[130];
//	sprintf(buff,"%f,%f\r\n",motor_lf->measure.speed_aps,motor_lf->motor_controller.speed_PID.Ref);
		sprintf(buff,"%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n",navi_recv->V_X,navi_recv->V_Y,navi_recv->V_Z,chassis_vx,chassis_vy,chassis_vw);
		HAL_UART_Transmit(&huart5,(uint8_t *)buff,strlen(buff),0xff);
    osDelay(1);
		#endif
  }
  /* USER CODE END StartVOFATask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

