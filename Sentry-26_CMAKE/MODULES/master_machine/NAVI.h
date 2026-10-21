#ifndef __NAVI_RX_H
#define __NAVI_RX_H
#define USB_Length  sizeof(NAVI_RX_struct)
#include "main.h"
#pragma pack(1)
typedef struct
{
	uint8_t   head;             //1
	float     V_X;              //4
	float     V_Y;              //4
	float     V_Z;              //4   
	short		  random;           //2
	short		  server_sta;       //2
	short		  decision_sta;     //2
	short     chassis_type;     //2
	char  	  crc8;             //1
	uint8_t   Tail;             //1
}NAVI_RX_struct;//23  注意内存对齐和结构体成员一一对应

typedef union
{
    NAVI_RX_struct NAVI_RX_Data;
    unsigned char NAVIRXArray[sizeof(NAVI_RX_struct)];
}NAVI_RX_union;

typedef struct 
{   
	uint8_t 	head;                                           //1
	short	sentry_HP;//哨兵血量                            //2
	short	mode;//比赛类型                                 //2
	short	race_time;//比赛剩余时间                        //2  
	short 	convertibility_bullet;//队内剩余可兑换弹量      //2 
	short	bullet_remaining_num_17mm;	//哨兵的允许发弹量  //2
	short 	defence_buff;//防御增益                         //2
	unsigned char myoutpost;//我方前哨站状态                //1
	unsigned char observeoutpost;//敌方前哨站状态           //1
	unsigned char start_race;//比赛进程（开始标志位）       //1
	unsigned char center_state;//中心增益点状态             //1
//	uint8_t hurt_reason;//受伤原因（这个不加）              //1
//    short   hero_HP;//英雄血量                              //2
//    short   balance3_HP;//3号步兵血量                       //2
	short 	random;//随机数                                 //2
	unsigned char crc;                                      //1
	uint8_t tail;                                           //1
}Judge_strcut; //21

typedef union
{
    Judge_strcut Judege_data_t;
    unsigned char Iudge_arr[sizeof(Judge_strcut)];
}Judge_union;



typedef struct 
{   
    uint8_t          head;                                           //1
	short	         sentry_HP;//哨兵血量                            //2
	short	         mode;//比赛类型                                 //2
	short	         race_time;//比赛剩余时间                        //2  
	short	         bullet_remaining_num_17mm;	//哨兵的允许发弹量   //2
    unsigned char    start_race;//比赛进程（开始标志位）             //1
	uint16_t         ally_outpost_HP_t; //己方前哨站血量             //2
    uint16_t         ally_base_HP_t; //己方基地血量                  //2
    uint16_t         enemy_outpost_HP_t;//对方前哨站血量             //2
    uint16_t         enemy_base_HP_t;//对方基地血量                  //2
	unsigned char    center_state;//中心增益点状态                   //1
    short 	         random;//随机数                                 //2
	unsigned char    crc;                                            //1
    uint8_t          tail;                                           //1
}NAVI_TX_struct; //23
#pragma pack()

/*Rb_status
bit 0： 对方英雄机器人主要状态（0 为不处于无敌，1 为处于无敌，下同）
bit 1： 对方工程机器人主要状态
bit 2： 对方3号步兵机器人主要状态
bit 3： 对方4号步兵机器人主要状态
bit 4： 对方哨兵机器人主要状态
*/
void NAVI_RX(void);
void NAVI_TX(void);

#endif
