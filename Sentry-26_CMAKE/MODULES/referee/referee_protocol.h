#ifndef referee_protocol_H
#define referee_protocol_H

#include "stdint.h"

/****************************宏定义部分****************************/
#define REFEREE_SOF 0xA5 // 起始字节,协议固定为0xA5
#define Robot_Red 0
#define Robot_Blue 1

#pragma pack(1)

/****************************通信协议格式****************************/
typedef enum
{
	FRAME_HEADER_Offset = 0,
	CMD_ID_Offset = 5,
	DATA_Offset = 7,
} JudgeFrameOffset_e;

typedef enum
{
	LEN_HEADER = 5, // 帧头长
	LEN_CMDID = 2,	// 命令码长度
	LEN_TAIL = 2,	// 帧尾CRC16
	LEN_CRC8 = 4,   // 帧头CRC8校验长度
} JudgeFrameLength_e;

typedef enum
{
	SOF = 0,		 // 起始位
	DATA_LENGTH = 1, // 帧内数据长度
	SEQ = 3,		 // 包序号
	CRC8 = 4		 // CRC8
} FrameHeaderOffset_e;

typedef struct
{
	uint8_t SOF;
	uint16_t DataLength;
	uint8_t Seq;
	uint8_t CRC8;
} xFrameHeader;

/****************************cmd_id命令码说明****************************/
typedef enum
{
	ID_game_state = 0x0001,				   // 比赛状态数据 (8 bytes)
	ID_game_result = 0x0002,			   // 比赛结果数据 (1 byte)
	ID_game_robot_survivors = 0x0003,	   // 比赛机器人血量数据 (20 bytes)
	ID_event_data = 0x0101,				   // 场地事件数据 (4 bytes)
	ID_game_robot_state = 0x0201,		   // 机器人状态数据 (17 bytes)
	ID_power_heat_data = 0x0202,		   // 实时功率热量数据 (14 bytes)
	ID_game_robot_pos = 0x0203,			   // 机器人位置数据 (16 bytes)
	ID_buff_musk = 0x0204,				   // 机器人增益数据 (8 bytes)
	ID_robot_hurt = 0x0206,				   // 伤害状态数据 (1 byte)
	ID_shoot_data = 0x0207,				   // 实时射击数据 (7 bytes)
	ID_student_interactive = 0x0301,	   // 机器人间交互数据
	ID_sentry_info = 0x020D,               // 哨兵自主决策信息同步 (14 bytes)
} CmdID_e;

typedef enum
{
	LEN_game_state = 8,						 // 0x0001
	LEN_game_result = 1,						 // 0x0002
	LEN_game_robot_HP = 20,						 // 0x0003 (2026改版)
	LEN_event_data = 4,							 // 0x0101
	LEN_game_robot_state = 17,					 // 0x0201 (2026改版)
	LEN_power_heat_data = 14,					 // 0x0202 (2026改版)
	LEN_game_robot_pos = 16,					 // 0x0203
	LEN_buff_musk = 8,							 // 0x0204 (2026改版)
	LEN_robot_hurt = 1,							 // 0x0206
	LEN_shoot_data = 7,							 // 0x0207
	LEN_sentry_info = 14,                        // 0x020D
} JudgeDataLength_e;

/****************************接收数据的详细说明 (2026版)****************************/

// 0x0001 (8 bytes)
typedef struct 
{ 
	uint8_t game_type : 4; 
	uint8_t game_progress : 4; 
	uint16_t stage_remain_time; 
	uint64_t SyncTimeStamp; 
} ext_game_state_t;

// 0x0002 (1 byte)
typedef struct
{
	uint8_t winner;
} ext_game_result_t;

// 0x0003 (20 bytes)
typedef struct
{
	uint16_t ally_1_robot_HP;
	uint16_t ally_2_robot_HP;
	uint16_t ally_3_robot_HP;
	uint16_t ally_4_robot_HP;
	int16_t  damage_difference;
	uint16_t ally_7_robot_HP;
	uint16_t ally_outpost_HP;
	uint16_t ally_base_HP;
	uint16_t enemy_outpost_HP;
	uint16_t enemy_base_HP;
} ext_game_robot_HP_t;

// 0x0101 (4 bytes)
typedef struct
{
	uint32_t event_data;
} ext_event_data_t;

// 0x0201 (17 bytes)
typedef struct
{
	uint8_t robot_id; 
	uint8_t robot_level; 
	uint16_t current_HP; 
	uint16_t maximum_HP; 
	uint16_t shooter_barrel_cooling_value; 
	uint16_t shooter_barrel_heat_limit; 
	uint16_t chassis_power_limit; 
	float bullet_speed_limit;
	uint8_t power_management_gimbal_output : 1; 
	uint8_t power_management_chassis_output : 1; 
	uint8_t power_management_shooter_output : 1; 
} ext_game_robot_state_t;

// 0x0202 (14 bytes)
typedef struct
{
	uint16_t reserved1; 
	uint16_t reserved2; 
	float reserved3; 
	uint16_t buffer_energy; 
	uint16_t shooter_17mm_barrel_heat; 
	uint16_t shooter_42mm_barrel_heat; 
} ext_power_heat_data_t;

// 0x0203 (16 bytes)
typedef struct
{
	float x;
	float y;
	float angle;
	uint8_t reserved[4];
} ext_game_robot_pos_t;

// 0x0204 (8 bytes)
typedef struct
{
	uint8_t recovery_buff; 
	uint8_t cooling_buff; 
	uint8_t defence_buff; 
	uint8_t vulnerability_buff; 
	uint16_t attack_buff; 
	uint8_t energy_feedback;
	uint8_t reserved;
} ext_buff_musk_t;

// 0x0206 (1 byte)
typedef struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
} ext_robot_hurt_t;

// 0x0207 (7 bytes)
typedef struct
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
} ext_shoot_data_t;

// 0x020D (14 bytes) - 哨兵自主决策信息同步
typedef struct
{
    // 字节 0-3
    uint32_t sentry_info_1; 
    // 字节 4-7
    uint32_t sentry_info_2;
    // 字节 8-13 (各姿态剩余时间)
    uint8_t posture_time[6];
} ext_sentry_info_t;

/****************************交互数据头与指令(发送)****************************/
typedef struct
{
	uint16_t data_cmd_id; 
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

// 0x0301 下的子内容ID
typedef enum
{
	Sentry_Data_ID_Cmd = 0x0120, // 哨兵自主决策指令
} Interactive_Data_ID_e;

// 0x0120 哨兵决策指令位域结构
typedef struct
{
    uint32_t confirm_respawn    : 1; // bit 0
    uint32_t confirm_buy_respawn: 1; // bit 1
    uint32_t exchange_ammo      : 11;// bit 2-12
    uint32_t remote_ammo_times  : 4; // bit 13-16
    uint32_t remote_hp_times    : 4; // bit 17-20
    uint32_t posture_cmd        : 3; // bit 21-23
    uint32_t activate_rune      : 1; // bit 24
    uint32_t reserved           : 7; // bit 25-31
} sentry_cmd_t;

// 发送哨兵指令的完整包结构体
typedef struct
{
	xFrameHeader FrameHeader;
	uint16_t CmdID;
	ext_student_interactive_header_data_t datahead;
	sentry_cmd_t cmd;
	uint16_t frametail;
} Sentry_Command_SendData_t;

#pragma pack()

#endif