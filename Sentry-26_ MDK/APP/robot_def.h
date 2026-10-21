/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
//#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
//#define CHASSIS_BOARD //底盘板
#define GIMBAL_BOARD  //云台板

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
#include "master_process.h"
#include "vision_types.h"

//底盘参数
#define CHASSIS_SPEED_MAX            2.0f	//底盘最大速度限幅 单位m/s

// 云台参数
#define GIMBAL_NAVI_YAW_REF   (-0.77f) 					//导航模式下 yaw 电机的编码器目标角
#define GIMBAL_FOLLOW_TARGET_OFFSET  (-0.77f)   //跟随模式下 yaw 电机的编码器目标角

#define BP_ALIGN_ENTER_TH     (0.10f)
#define BP_ALIGN_EXIT_TH      (0.20f)	/* 大 pitch 收放判定:进入阈值 < 退出阈值 => 迟滞;连续 BP_DEBOUNCE_CNT 拍意见一致才翻转 */
#define BP_DEBOUNCE_CNT       (20)   	/* 20 * 5ms = 100ms */
#define YAW_ALIGN_ANGLE       0.0f    // 对齐时 yaw 电机的弧度值
#define PITCH_HORIZON_ANGLE   0.0f    // pitch 水平时电机的弧度值

#define GIMBAL_YAW_SCALE             0.001f
#define GIMBAL_SMALL_PITCH_SCALE     0.0003f
#define GIMBAL_SMALLPITCH_DOWN_LIMIT (-10.0f)
#define GIMBAL_SMALLPITCH_UP_LIMIT   (25.0f)
#define GIMBAL_BIGPITCH_DOWN           (-1.46f)
#define GIMBAL_BIGPITCH_UP         (0.0f)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 1650   	// 发射一发弹丸拨盘编码器转动增量
#define REDUCTION_RATIO_LOADER 36.0f		// 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE         8    		// 拨盘一圈的装载量
#define FIRE_BURST_RATE        10.0f
// 堵弹参数
#define JAM_OUTPUT_TH    1000.0f        // 堵转实测 1596/3249,阈值取 1000
#define JAM_SPEED_TH     2500.0f        // 堵转实测 170~1963,阈值取 2500
#define JAM_DETECT_CNT   30             // 堵弹判定计数阈值
#define JAM_WAIT_CNT     300            // 退弹后等待稳定计数阈值
#define JAM_REVERSE_ANGLE 3.0f*1650.0f  // 退弹编码器值
#define JAM_REVERSE_SPEED 3000.0f 			// 手动反转速度
// 底盘参数,单位m(米)
#define WHEEL_DISTANCE 0.295f              // 轮子到底盘中心的距离
#define RADIUS_WHEEL 0.0725f             // 轮子半径
#define REDUCTION_RATIO_WHEEL 15.765f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
#define WHEEL_V_TO_APS  (REDUCTION_RATIO_WHEEL * 360.0f / (2.0f * PI * RADIUS_WHEEL))

#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
// 陀螺仪参数
#define GYRO2GIMBAL_DIR_YAW -1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_SMALL_PITCH -1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反
// 遥控器缩放最大数
#define RC_ROCKER_MAX                660.0f
// 小陀螺模式参数
#define CHASSIS_ROTATE_WZ_STEP  0.4f    // 每次拨动的步长
#define CHASSIS_ROTATE_WZ_MAX   4.8f   // 角速度上限
// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_NAVIGATION,        // 导航测试模式
    CHASSIS_COMPETITION,       // 竞赛模式,底盘完全交给导航控制
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
		GIMBAL_AUTO_MODE,				 // 自瞄云台
} gimbal_mode_e;

typedef enum
{
    Gimbal_Remote,
    Gimbal_NAVI,
		Gimbal_Cruise_360,
} Gimbal_Control_Mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;
typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;
// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float Spitch;
    float chassis_rotate_wz;
		
    float Bpitch;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    // 后续增加底盘的真实速度
    float real_vx;
     float real_vy;
     float real_wz;

    uint8_t rest_heat;           // 剩余枪口热量
    Bullet_Speed_e bullet_speed; // 弹速限制
    Enemy_Color_e enemy_color;   // 0 for blue, 1 for red

} Chassis_Upload_Data_s;


typedef struct
{
    attitude_t gimbal_imu_data;
//    uint16_t yaw_motor_single_round_angle;
	float yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

typedef struct
{
	uint8_t jam_state;      // 0=正常 1=退弹中 2=延时中
    // code to go here
    // ...
} Shoot_Upload_Data_s;
/* ----------------状态集中管理----------------*/
typedef struct
{
	uint8_t friction_on;          //摩擦轮开关
	uint8_t Bpitch_target;        //越隧目标
	uint8_t continue_fire;        //连发
	uint8_t single_shot_pending;  //单发
  float   rotate_wz;            //小陀螺角速度设定值
}CmdState_s;
/* ----------------堵弹检测----------------*/

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H
