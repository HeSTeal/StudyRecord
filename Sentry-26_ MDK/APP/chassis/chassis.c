#include "chassis.h"
#ifdef CHASSIS_BOARD

#include "robot_def.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "math.h"
#include "DJIMotor.h"

#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
//#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
extern Navi_Recv_s *navi_recv;  		  // 导航数据接收指针
#endif // CHASSIS_BOARD                                             
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
__align(4) Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
static DJIMotorInstance *motor_lf,*motor_rf,*motor_lb,*motor_rb;
static PIDInstance chassis_follow_pid;   // 底盘跟随云台 yaw 的 PID
/* 用于自旋变速策略的时间变量 */
// static float t;
#include "stdio.h"
#include "usart.h"
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
 float chassis_vx, chassis_vy,chassis_vw;   // 底盘目标速度
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
		.Kp= 10.0f,.Ki= 0.0f,.Kd= 0.3f,                         
		.MaxOut= 6.28f,.DeadBand= 0.01f,.Improve= PID_Derivative_On_Measurement, // 微分先行，防设定值跳变
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

//坐标系旋转函数
static void RotateVec(float angle, float ix, float iy, float *ox, float *oy)
{
    float c = arm_cos_f32(angle);
    float s = arm_sin_f32(angle);
    *ox = ix * c - iy * s;
    *oy = ix * s + iy * c;
}
static void ForwardKinematics(float *vx, float *vy, float *vw)
{
    /* 取电机反馈转速(APS)，换算回轮缘线速度(m/s) */
    float v_lf = motor_lf->measure.speed_aps / WHEEL_V_TO_APS;
    float v_rf = motor_rf->measure.speed_aps / WHEEL_V_TO_APS;
    float v_lb = motor_lb->measure.speed_aps / WHEEL_V_TO_APS;
    float v_rb = motor_rb->measure.speed_aps / WHEEL_V_TO_APS;

    *vx = ((v_lf + v_lb) - (v_rf + v_rb)) / (4.0f * 1.414f);
    *vy = ((v_lb + v_rb) - (v_lf + v_rf)) / (4.0f * 1.414f);
    *vw =  (v_lf + v_rf + v_lb + v_rb) / (4.0f * WHEEL_DISTANCE);
}

/* 将正解算结果写入静态变量（可选包装） */
static void EstimateSpeed(void)
{
    ForwardKinematics(&chassis_feedback_data.real_vx,&chassis_feedback_data.real_vy,&chassis_feedback_data.real_wz);
}
//底盘解算函数
static void MecanumCalculate()
{
    vt_lf = (1.414f*( chassis_vx - chassis_vy) + chassis_vw * WHEEL_DISTANCE)*WHEEL_V_TO_APS;
    vt_rf = (1.414f*(-chassis_vx - chassis_vy) + chassis_vw * WHEEL_DISTANCE)*WHEEL_V_TO_APS;
    vt_lb = (1.414f*( chassis_vx + chassis_vy) + chassis_vw * WHEEL_DISTANCE)*WHEEL_V_TO_APS;
    vt_rb = (1.414f*(-chassis_vx + chassis_vy) + chassis_vw * WHEEL_DISTANCE)*WHEEL_V_TO_APS;
		
//		// 四轮等比缩放，防止某个轮子目标速度过大导致四轮比例失衡
//    #define MAX_LINEAR_SPEED 1.5f   // 单位 m/s
//    float max_wheel_speed = MAX_LINEAR_SPEED * WHEEL_V_TO_APS;

//    float max_abs_speed = 0.0f;
//    max_abs_speed = fmaxf(max_abs_speed, fabsf(vt_lf));
//    max_abs_speed = fmaxf(max_abs_speed, fabsf(vt_rf));
//    max_abs_speed = fmaxf(max_abs_speed, fabsf(vt_lb));
//    max_abs_speed = fmaxf(max_abs_speed, fabsf(vt_rb));

//    if (max_abs_speed > max_wheel_speed)
//    {
//        float scale = max_wheel_speed / max_abs_speed;
//        vt_lf *= scale;
//        vt_rf *= scale;
//        vt_lb *= scale;
//        vt_rb *= scale;
//    }
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

    // 底盘模式状态机
    switch (chassis_cmd_recv.chassis_mode)
    {
			case CHASSIS_NO_FOLLOW: //分离
			case CHASSIS_ROTATE: 		//陀螺
			{
				chassis_vw=chassis_cmd_recv.wz;
				RotateVec(chassis_cmd_recv.offset_angle,chassis_cmd_recv.vx,chassis_cmd_recv.vy,&chassis_vx,&chassis_vy);
			}break;		
			
			case CHASSIS_FOLLOW_GIMBAL_YAW: //跟随
			{			
				chassis_vw = -PIDCalculate(&chassis_follow_pid,chassis_cmd_recv.offset_angle,GIMBAL_FOLLOW_TARGET_OFFSET);
				chassis_vx = chassis_cmd_recv.vx;
				chassis_vy = chassis_cmd_recv.vy;
			}break;
			
			case CHASSIS_NAVIGATION: // 导航
				chassis_vw = chassis_cmd_recv.wz;
				if(navi_recv!=NULL)
				{
					chassis_vx = navi_recv->V_X;
					chassis_vy = navi_recv->V_Y;
//					chassis_vw = navi_recv->V_Z;
//					LIMIT_MIN_MAX(chassis_vx, -0.6f, 0.6f);
//					LIMIT_MIN_MAX(chassis_vy, -0.6f, 0.6f);
//					LIMIT_MIN_MAX(chassis_vw, -3.0f, 3.0f);
				}
				else
				{
					chassis_vx = 0.0f;
					chassis_vy = 0.0f;
//					chassis_vw = 0.0f;
				}break;
				
			default:
				chassis_vx = 0.0f;
				chassis_vy = 0.0f;
//				chassis_vw = 0.0f;
			break;
    }
    MecanumCalculate();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

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
