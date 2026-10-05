/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "chassis.h"
#ifdef CHASSIS_BOARD

#include "robot_def.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "math.h"
#include "DJIMotor.h"
/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
//#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
extern Navi_Recv_s *navi_recv;  		  // 导航数据接收指针
#endif // CHASSIS_BOARD                                             
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
static DJIMotorInstance *motor_lf,*motor_rf,*motor_lb,*motor_rb;
static PIDInstance chassis_follow_pid;   // 底盘跟随云台 yaw 的 PID
/* 用于自旋变速策略的时间变量 */
// static float t;
#include "stdio.h"
#include "usart.h"
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
 float chassis_vx, chassis_vy,chassis_vw;   // 底盘目标速度
 float NAVI_vx, NAVI_vy, NAVI_vw;           // 导航目标速度
static float vt_lf, vt_rf, vt_lb, vt_rb;          // 底盘速度解算后的临时输出,待进行限幅

void ChassisInit()
{
	Motor_Init_Config_s chassis_motor_config={
		.can_init_config.can_handle=&hcan2,
		.controller_param_init_config={
			.speed_PID=
			{
				.Kp=2.2f,.Ki=0,.Kd=0,
			  .IntegralLimit=1000,
			  .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
			  .MaxOut=15000,
			},
		},
		.controller_setting_init_config=
		{
			.angle_feedback_source=MOTOR_FEED,
			.speed_feedback_source=MOTOR_FEED,
			.outer_loop_type = SPEED_LOOP, 
      .close_loop_type = SPEED_LOOP,
		},
		.motor_type=M3508,
	};
	chassis_motor_config.can_init_config.tx_id=1;
	chassis_motor_config.controller_setting_init_config.motor_reverse_flag=MOTOR_DIRECTION_NORMAL;
	motor_lb = DJIMotorInit(&chassis_motor_config);
	chassis_motor_config.can_init_config.tx_id=2;
	chassis_motor_config.controller_setting_init_config.motor_reverse_flag=MOTOR_DIRECTION_NORMAL;
	motor_rb = DJIMotorInit(&chassis_motor_config);
	chassis_motor_config.can_init_config.tx_id=3;
	chassis_motor_config.controller_setting_init_config.motor_reverse_flag=MOTOR_DIRECTION_NORMAL;
	motor_rf = DJIMotorInit(&chassis_motor_config);
	chassis_motor_config.can_init_config.tx_id=4;
	chassis_motor_config.controller_setting_init_config.motor_reverse_flag=MOTOR_DIRECTION_NORMAL;
	motor_lf = DJIMotorInit(&chassis_motor_config);
	
	PID_Init_Config_s follow_pid_config = {
    .Kp             = 10.0f,                          // 先调这个
    .Ki             = 0.0f,                          // 跟随环不要 I
    .Kd             = 0.3f,                         // 再加这个防超调
    .MaxOut         = 6.28f,                         // 输出限幅，就是最大 chassis_vw
    .DeadBand       = 0.01f,                         // 死区，约 0.57°
    .Improve        = PID_Derivative_On_Measurement, // 微分先行，防设定值跳变
	};
	PIDInit(&chassis_follow_pid, &follow_pid_config);

  CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
}
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    vt_lf = (1.414f*( chassis_vx - chassis_vy) + chassis_vw * WHEEL_DISTANCE)*WHEEL_V_TO_APS;
    vt_rf = (1.414f*(-chassis_vx - chassis_vy) + chassis_vw * WHEEL_DISTANCE)*WHEEL_V_TO_APS;
    vt_lb = (1.414f*( chassis_vx + chassis_vy) + chassis_vw * WHEEL_DISTANCE)*WHEEL_V_TO_APS;
    vt_rb = (1.414f*(-chassis_vx + chassis_vy) + chassis_vw * WHEEL_DISTANCE)*WHEEL_V_TO_APS;

    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}
/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
		if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
			static float sin_theta, cos_theta;
			case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
				chassis_vw=chassis_cmd_recv.wz;
				
				cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle);
				sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle);
				chassis_vx = chassis_cmd_recv.vx * cos_theta + chassis_cmd_recv.vy * sin_theta;
				chassis_vy = -chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
			break;		
			case CHASSIS_FOLLOW_GIMBAL_YAW:
			{	// 跟随云台,不单独设置pid
				#define GIMBAL_FOLLOW_TARGET_OFFSET  (-0.77f)   // 目标相对角度
				chassis_vw = -PIDCalculate(&chassis_follow_pid,chassis_cmd_recv.offset_angle,GIMBAL_FOLLOW_TARGET_OFFSET);
				chassis_vx = chassis_cmd_recv.vx;
				chassis_vy = chassis_cmd_recv.vy;
			break;
			}		
			case CHASSIS_ROTATE: //小陀螺
				chassis_vw = chassis_cmd_recv.wz;
		
				cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle);
				sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle);
				chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
				chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
			break;
			case CHASSIS_NAVIGATION: // 导航模式,由视觉或其他传感器控制底盘
				if(navi_recv->V_X!=0 && navi_recv->V_Y!=0 && navi_recv->V_Z!=0)
				{
					NAVI_vx = navi_recv->V_X;
					NAVI_vy = navi_recv->V_Y;
					NAVI_vw = navi_recv->V_Z;
					 if(navi_recv->V_X==0 && navi_recv->V_Y!=0)
					{
						chassis_vx = 0;
						chassis_vy = NAVI_vy;
						chassis_vw = NAVI_vw;
					}
					else if(navi_recv->V_X!=0 && navi_recv->V_Y==0)
					{
						chassis_vx = NAVI_vx;
						chassis_vy = 0;
						chassis_vw = NAVI_vw;
					}
					else
					{
						chassis_vx = NAVI_vx;
						chassis_vy = NAVI_vy;
						chassis_vw = NAVI_vw;
					}
					LIMIT_MIN_MAX(chassis_vx,-0.5f,0.5f);
					LIMIT_MIN_MAX(chassis_vy,-0.5f,0.5f);
					LIMIT_MIN_MAX(chassis_vw,-4.0f,4.0f);
				}
				else
				{
					chassis_vx = 0;
					chassis_vy = 0;
					chassis_vw = 0.0f;
				}
				
			break;
			default:
			break;
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
//    EstimateSpeed();

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // 推送反馈消息
		CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
}
#endif
