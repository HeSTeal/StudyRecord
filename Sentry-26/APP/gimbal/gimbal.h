#include "robot_def.h"
#ifdef GIMBAL_BOARD
#ifndef GIMBAL_H
#define GIMBAL_H

/**
 * @file gimbal.h
 * @brief 云台应用(YAW/PITCH 双轴姿态闭环,反馈取自 IMU)
 */

/**
 * @brief 初始化云台,会被RobotInit()调用
 */
void GimbalInit(void);

/**
 * @brief 云台任务
 */
void GimbalTask(void);

#endif // !GIMBAL_H
#endif
