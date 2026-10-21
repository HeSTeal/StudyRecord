/**
 * @file master_process.c
 * @brief vision/navi communication via bsp_usb
 */
#include "robot_def.h"        /* ← 拿 GIMBAL_BOARD / VISION_USE_VCP */
#include "master_process.h"   /* ← 拿 Vision_Recv_s / API 声明 */

#ifdef VISION_USE_VCP
#include "bsp_usb.h"

/* ================= 自瞄板 ================= */
#ifdef GIMBAL_BOARD

#include "Auto_Aim.h"
#include "usbd_cdc_if.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static uint8_t *vis_recv_buff;
static Send_PC_t    aim_send;

static void DecodeVision(uint16_t recv_len)
{
    if (recv_len >= MINIPC_FRAME_LENGTH &&
        vis_recv_buff[0] == 'R' && vis_recv_buff[1] == 'M')
    {
        memcpy((uint8_t*)&recv_data, vis_recv_buff, MINIPC_FRAME_LENGTH);
    }
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle);
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);
    return &recv_data;
}

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color  = enemy_color;
    send_data.work_mode    = work_mode;
    send_data.bullet_speed = bullet_speed;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw   = yaw;
    send_data.pitch = pitch;
    send_data.roll  = roll;
}

void VisionSend(void)
{
    static uint8_t send_buff[sizeof(Send_PC_t)];
    extern QEKF_INS_t QEKF_INS;

    aim_send.head[0] = 'R';
    aim_send.head[1] = 'M';
    aim_send.mode    = 0;
    aim_send.yaw     = send_data.yaw;
    aim_send.pitch   = send_data.pitch;
    aim_send.q[0]    =  QEKF_INS.q[0];
    aim_send.q[1]    = -QEKF_INS.q[2];
    aim_send.q[2]    =  QEKF_INS.q[1];
    aim_send.q[3]    =  QEKF_INS.q[3];
    aim_send.yaw_vel   = QEKF_INS.Gyro[2];
    aim_send.pitch_vel = QEKF_INS.Gyro[0];
    aim_send.bullet_speed = 5900;
    aim_send.bullet_count = 0;
    aim_send.colour = (send_data.enemy_color == COLOR_RED) ? 1 : 0;

    memcpy(send_buff, (uint8_t*)&aim_send, sizeof(Send_PC_t));
    USBTransmit(send_buff, sizeof(Send_PC_t));
}

#endif /* GIMBAL_BOARD */

/* ================= 导航板 ================= */
#ifdef CHASSIS_BOARD

#include "NAVI.h"
#include "crc_ref.h"

static Navi_Recv_s recv_data;
static Navi_Send_s send_data;
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    if (recv_len >= USB_Length &&
        vis_recv_buff[0] == 0x7C &&
        vis_recv_buff[USB_Length-1] == 0xC7)
    {
        if (Verify_CRC8_Check_Sum(&vis_recv_buff[1], 21) == TRUE)
        {
            memcpy((uint8_t*)&recv_data, vis_recv_buff, USB_Length);
        }
    }
}

Navi_Recv_s *NaviInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle);
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);
    return &recv_data;
}

void NaviSend(void)
{
    static uint8_t send_buff[sizeof(Judge_strcut)];
    Judge_union *ju = (Judge_union*)&send_data;
    unsigned char crc = Get_CRC8_Check_Sum(&ju->Iudge_arr[1], 18, 0xff);
    send_data.crc = crc;
    memcpy(send_buff, &send_data, sizeof(Judge_strcut));
    USBTransmit(send_buff, sizeof(Judge_strcut));
}

#endif /* CHASSIS_BOARD */

#endif /* VISION_USE_VCP */
