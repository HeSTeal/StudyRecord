#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include "usart.h"
#include "referee_protocol.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"

#pragma pack(1)
typedef struct
{
	uint8_t Robot_Color;		// 机器人颜色
	uint16_t Robot_ID;			// 本机器人ID
	uint16_t Cilent_ID;			// 本机器人对应的客户端ID
} referee_id_t;

// 此结构体包含裁判系统所有接收数据
typedef struct
{
	referee_id_t referee_id;

	xFrameHeader FrameHeader; 
	uint16_t CmdID;
	ext_game_state_t GameState;							   
	ext_game_result_t GameResult;						   
	ext_game_robot_HP_t GameRobotHP;					   
	ext_event_data_t EventData;							   
	ext_game_robot_state_t GameRobotState;				   
	ext_power_heat_data_t PowerHeatData;				   
	ext_game_robot_pos_t GameRobotPos;					   
	ext_buff_musk_t BuffMusk;							   
	ext_robot_hurt_t RobotHurt;							   
	ext_shoot_data_t ShootData;							   
	ext_sentry_info_t SentryInfo;                          // 0x020D 新增
	uint8_t init_flag;
} referee_info_t;

#pragma pack()

referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle);
void RefereeSend(uint8_t *send, uint16_t tx_len);

#endif // !REFEREE_H