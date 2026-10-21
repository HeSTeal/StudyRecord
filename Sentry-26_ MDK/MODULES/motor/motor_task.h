/**
 * @file motor_task.h
 * @brief 电机控制任务的对外接口。
 *
 *        MotorControlTask() 必须以 1kHz 的频率在 FreeRTOS 线程中运行
 *        (见 APP/robot_task.c 的 StartMOTORTASK), 因为 DJIMotorControl()
 *        是**唯一**构建并发送 0x200 / 0x1FF / 0x2FF 控制帧的地方。
 *
 * @note   如果这个任务不存在, 链接器会把 DJIMotorControl() 和 CANTransmit()
 *         完全裁掉, CAN 总线上不会有任何报文, C620/C610 电调也就不会有反馈。
 */
#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

/**
 * @brief 电机控制循环的一次迭代: 串级 PID + 每个电机组的 CAN 帧发送。
 *
 * @note  请以 1kHz 调用。**不允许**在中断中调用。
 * @note  ⚠️ 整个工程里只能有一个 1kHz 的调用者, 否则控制帧会重复发送。
 */
void MotorControlTask(void);

#endif /* MOTOR_TASK_H */
