#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

/**
 * @file robot_cmd.h
 * @brief 机器人核心控制任务(遥控器/键鼠/视觉输入 → 各应用控制命令)
 */

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 */
void RobotCMDInit(void);

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 */
void RobotCMDTask(void);

#endif // !ROBOT_CMD_H
