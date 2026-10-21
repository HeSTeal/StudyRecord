/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "robot_def.h"
#include "RGB.h"
#include "referee_task.h"
#include "usart.h"
//#include "referee_task.h"
//#include "master_process.h"
//#include "daemon.h"

//#include "bsp_log.h"

void StartINSTask(void *argument)
{
  /* USER CODE BEGIN StartINSTask */
  /* Infinite loop */
  for(;;)
  {
		#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
		INS_Task();
		#endif
    osDelay(1);
  }
  /* USER CODE END StartINSTask */
}

void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
//	static float motor_dt;
//  static float motor_start;
  /* Infinite loop */
  for(;;)
  {
//		motor_start = DWT_GetTimeline_ms();
		MotorControlTask();
//		motor_dt = DWT_GetTimeline_ms() - motor_start;
    osDelay(1);
  }
  /* USER CODE END StartMotorTask */
}

void StartRefreeTask(void *argument)
{
  /* USER CODE BEGIN StartRefreeTask */
  SentryTaskInit(&huart5);
  /* Infinite loop */
  for(;;)
  {
    SentryTask(); 
  }
  /* USER CODE END StartRefreeTask */
}

void StartRobotTask(void *argument)
{
  /* USER CODE BEGIN StartRobotTask */
//	static float robot_dt;
//  static float robot_start;
  /* Infinite loop */
  for(;;)
  {
//		robot_start = DWT_GetTimeline_ms();
		RobotTask();
//		robot_dt = DWT_GetTimeline_ms() - robot_start;
    osDelay(1);
  }
  /* USER CODE END StartRobotTask */
}

#endif
