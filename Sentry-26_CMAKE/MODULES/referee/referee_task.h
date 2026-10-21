#ifndef REFEREE_TASK_H
#define REFEREE_TASK_H

#include "rm_referee.h"

/**
 * @brief 初始化裁判系统任务 (纯数据通信)
 * @param referee_usart_handle 裁判系统串口句柄
 * @return referee_info_t* 裁判系统数据指针
 */
referee_info_t *SentryTaskInit(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief 哨兵主任务，放在 freertos.c 的循环中周期调用
 */
void SentryTask(void);

/**
 * @brief 发送哨兵自主决策指令 (0x0301 -> 0x0120)
 * @param cmd 32位指令位域结构体
 */
void SendSentryCommand(sentry_cmd_t cmd);

#endif // REFEREE_TASK_H