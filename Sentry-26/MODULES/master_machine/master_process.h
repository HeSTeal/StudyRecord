#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usb.h"
#include "robot_def.h"
#include "vision_types.h"
#pragma pack(1)
/* ============ 自瞄板（云台板） ============ */
#ifdef GIMBAL_BOARD
#include "Auto_Aim.h" 

typedef PC_Receive_t Vision_Recv_s;
typedef struct {
    Enemy_Color_e  enemy_color;
    Work_Mode_e    work_mode;
    Bullet_Speed_e bullet_speed;
    float yaw, pitch, roll;
} Vision_Send_s;

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);
void VisionSend(void);
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);
void VisionSetAltitude(float yaw, float pitch, float roll);

#endif /* GIMBAL_BOARD */

/* ============ 导航板（底盘板） ============ */
#ifdef CHASSIS_BOARD

#include "NAVI.h"
typedef NAVI_RX_struct Navi_Recv_s;
typedef Judge_strcut   Navi_Send_s;

Navi_Recv_s *NaviInit(UART_HandleTypeDef *_handle);
void NaviSend(void);

#endif /* CHASSIS_BOARD */

#pragma pack()

#endif /* MASTER_PROCESS_H */
