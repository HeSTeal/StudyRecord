#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u 

static USARTInstance *referee_usart_instance; 
static DaemonInstance *referee_daemon;		  
static referee_info_t referee_info;			  

static void JudgeReadData(uint8_t *buff)
{
	uint16_t judge_length; 
	if (buff == NULL) return;

	memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);

	if (buff[SOF] == REFEREE_SOF)
	{
		if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
		{
			judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
			{
				referee_info.CmdID = (buff[6] << 8 | buff[5]);
				switch (referee_info.CmdID)
				{
				case ID_game_state:
					memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_game_state);
					break;
				case ID_game_result:
					memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_game_result);
					break;
				case ID_game_robot_survivors:
					memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_game_robot_HP);
					break;
				case ID_event_data:
					memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_event_data);
					break;
				case ID_game_robot_state:
					memcpy(&referee_info.GameRobotState, (buff + DATA_Offset), LEN_game_robot_state);
					break;
				case ID_power_heat_data:
					memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);
					break;
				case ID_game_robot_pos:
					memcpy(&referee_info.GameRobotPos, (buff + DATA_Offset), LEN_game_robot_pos);
					break;
				case ID_buff_musk:
					memcpy(&referee_info.BuffMusk, (buff + DATA_Offset), LEN_buff_musk);
					break;
				case ID_robot_hurt:
					memcpy(&referee_info.RobotHurt, (buff + DATA_Offset), LEN_robot_hurt);
					break;
				case ID_shoot_data:
					memcpy(&referee_info.ShootData, (buff + DATA_Offset), LEN_shoot_data);
					break;
				case ID_sentry_info: // 新增哨兵信息解析
					memcpy(&referee_info.SentryInfo, (buff + DATA_Offset), LEN_sentry_info);
					break;
				}
			}
		}
		if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{ 
			JudgeReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL);
		}
	}
}

static void RefereeRxCallback()
{
	DaemonReload(referee_daemon);
	JudgeReadData(referee_usart_instance->recv_buff);
}

static void RefereeLostCallback(void *arg)
{
	USARTServiceInit(referee_usart_instance);
}

referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = RefereeRxCallback;
	conf.usart_handle = referee_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE;
	referee_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = RefereeLostCallback,
		.owner_id = referee_usart_instance,
		.reload_count = 30, 
	};
	referee_daemon = DaemonRegister(&daemon_conf);

	return &referee_info;
}

void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	osDelay(35); // 2026协议交互数据上限30Hz(33ms)，这里留35ms余量
}