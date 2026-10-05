/**
 * @file motor_task.c
 * @brief 1kHz 电机控制任务。详见 motor_task.h。
 */
#include "motor_task.h"
#include "DJImotor.h"

/**
 * @brief 电机控制循环的一次迭代。
 *
 *        DJIMotorControl() 会遍历 dji_motor_instance[], 为每个已注册的电机计算
 *        串级 PID 并填入 sender_assignment[], 然后对每组至少有一个电机的
 *        sender_enable_flag[] 调用 CANTransmit()。
 *
 * @note  C620/C610 电调要求每 1ms 收到一次控制帧。一旦帧停止, 电调会掉入 idle
 *        状态并且不再回传反馈报文 —— 这正是以前这个任务不存在时的现象。
 *
 * @note  ⚠️ 本函数**只能**被 1kHz 的 MOTOR 任务调用一次!
 *        曾经 Core/Src/freertos.c 的 MotorTask 和 APP/robot_task.c 的 StartMOTORTASK
 *        同时存在, 会让 CAN 总线上每毫秒重复发两遍控制帧。
 */
void MotorControlTask(void)
{
    DJIMotorControl();
}
