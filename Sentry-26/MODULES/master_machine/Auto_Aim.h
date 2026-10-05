#ifndef __AUTO_AIM_H__
#define __AUTO_AIM_H__

#include "main.h"
#define MINIPC_FRAME_LENGTH   sizeof(PC_Receive_t)   // 或者直接写 43
typedef __packed struct
{
    uint8_t head[2];
    uint8_t mode; 
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint8_t random;  
    uint8_t Sub_camera;  
}PC_Receive_t;  

typedef union
{
    PC_Receive_t PcData;
    unsigned char PcDataArray[sizeof(PC_Receive_t)];
} PC_Receive_Union_t;

typedef __packed struct 
{
    uint8_t head[2] ;
    uint8_t mode;
    float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
    uint8_t colour;
    uint8_t  hero_data; 
    uint8_t  engi_data; 
    uint8_t  balance1_data; 
    uint8_t  balance2_data; 
    uint8_t  sentry_data;
		uint8_t reserved;	
}Send_PC_t;


typedef union
{
    Send_PC_t PcDate;
    uint8_t Send_PcDataArray[sizeof(Send_PC_t)];
} PC_Send_Union_t;

void PC_Receive(void);
void PC_Send(void);


#endif
