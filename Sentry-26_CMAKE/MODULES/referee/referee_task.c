#include "referee_task.h"
#include "string.h"
#include "cmsis_os.h"
#include "crc_ref.h"

static referee_info_t *referee_recv_info;
static uint8_t seq = 0; // 包序号

referee_info_t *SentryTaskInit(UART_HandleTypeDef *referee_usart_handle)
{
    referee_recv_info = RefereeInit(referee_usart_handle); 
    referee_recv_info->init_flag = 1;
    return referee_recv_info;
}

// 发送哨兵指令函数实现
void SendSentryCommand(sentry_cmd_t cmd)
{
    Sentry_Command_SendData_t send_data;
    uint8_t temp_datalength = 6 + 4; // 交互数据头(6) + 哨兵指令(4)

    send_data.FrameHeader.SOF = REFEREE_SOF;
    send_data.FrameHeader.DataLength = temp_datalength;
    send_data.FrameHeader.Seq = seq;
    send_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&send_data, LEN_CRC8, 0xFF);

    send_data.CmdID = ID_student_interactive;

    send_data.datahead.data_cmd_id = Sentry_Data_ID_Cmd; // 0x0120
    // 哨兵发给裁判系统服务器，接收者ID为 0x8080
    send_data.datahead.sender_ID = referee_recv_info->referee_id.Robot_ID; 
    send_data.datahead.receiver_ID = 0x8080; 

    // 拷贝指令
    memcpy(&send_data.cmd, &cmd, sizeof(sentry_cmd_t));

    // 计算CRC16并填入
    send_data.frametail = Get_CRC16_Check_Sum((uint8_t *)&send_data, LEN_HEADER + LEN_CMDID + temp_datalength, 0xFFFF);

    // 调用底层发送 (内部自带 35ms 延时)
    RefereeSend((uint8_t *)&send_data, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL); 

    seq++; 
}

/**
 * @brief 哨兵主任务逻辑框架
 * @note 请将此函数放在 FreeRTOS 的一个 Task 中周期调用 (例如 10Hz 或 30Hz)
 */
void SentryTask()
{
    // 如果没有初始化或者没收到裁判系统ID，直接返回
    if (!referee_recv_info->init_flag || referee_recv_info->GameRobotState.robot_id == 0) {
        return;
    }

    // 确定一次颜色和ID (只执行一次即可，这里为了安全每次都判断)
    if(referee_recv_info->referee_id.Robot_ID == 0) {
        referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
        referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
        referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; 
    }

    // ==========================================
    // 解析 0x020D 哨兵信息 (示例)
    // ==========================================
    // ext_sentry_info_t *sentry = &referee_recv_info->SentryInfo;
    // uint8_t is_free_respawn = (sentry->sentry_info_1 >> 19) & 0x01; // 是否可以免费复活
    // uint8_t current_posture = (sentry->sentry_info_2 >> 12) & 0x03; // 当前姿态

    // ==========================================
    // 构造并发送指令逻辑 (示例)
    // ==========================================
    sentry_cmd_t my_cmd = {0};
    
    // 1. 假设判断逻辑：如果血量归零且允许免费复活，则发送确认复活指令
    // if (referee_recv_info->GameRobotState.current_HP == 0 && is_free_respawn) {
    //     my_cmd.confirm_respawn = 1; // bit 0 置 1
    //     SendSentryCommand(my_cmd);
    //     return; // 复活指令发了就退出
    // }

    // 2. 假设判断逻辑：需要切换姿态为 进攻姿态(1) 
    // if (need_change_posture) {
    //     my_cmd.posture_cmd = 1; // bit 21-23 设为 1
    //     SendSentryCommand(my_cmd);
    // }
    
    // 3. 假设判断逻辑：远程兑换血量
    // my_cmd.remote_hp_times = 1; 
    // SendSentryCommand(my_cmd);

    // 注意：上述逻辑请根据你的具体步兵/哨兵策略自行实现。
    // 由于 SendSentryCommand 内部包含了 35ms 的延时，
    // 所以这个 Task 的调用频率建议在 10Hz(100ms) ~ 20Hz(50ms) 左右，避免阻塞。
}