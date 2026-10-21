#ifndef SHOOT_H
#define SHOOT_H

/**
 * @file shoot.h
 * @brief 发射机构应用(摩擦轮 + 拨盘)
 */

/**
 * @brief 发射初始化,会被RobotInit()调用
 */
void ShootInit(void);

/**
 * @brief 发射任务
 */
void ShootTask(void);

#endif // !SHOOT_H
