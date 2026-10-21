#include "gimbal.h"
#ifdef GIMBAL_BOARD
#include "robot_def.h"
#include "DMmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"

#include "master_process.h"
extern Vision_Recv_s *vision_recv;

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DMMotorInstance *yaw_motor, *big_pitch_motor, *small_pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

/* ---------- yaw 两套 PID:两种反馈源的单位/零点/噪声都不同,必须分别整定 ---------- */
/* IMU 反馈:角度 = YawTotalAngle(度,多圈) / 速度 = Gyro[2](rad/s) */
static PID_Init_Config_s yaw_angle_cfg_imu = {
    .Kp = 1, .Ki = 0, .Kd = 0, .MaxOut = 50, .IntegralLimit = 10,
    .Improve = (PID_Improvement_e)(PID_Integral_Limit | PID_Derivative_On_Measurement),
};
static PID_Init_Config_s yaw_speed_cfg_imu = {
    .Kp = 1, .Ki = 0, .Kd = 0, .MaxOut = 10, .IntegralLimit = 10,
    .Improve = (PID_Improvement_e)(PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter),
};
/* 编码器反馈:角度 = measure.position(±3.14, 1 单位≈3.98rad) / 速度 = measure.velocity(约 1.5×rad/s) */
static PID_Init_Config_s yaw_angle_cfg_enc = {
    .Kp = 25, .Ki = 0, .Kd = 0.3, .MaxOut = 120, .IntegralLimit = 50,
    .Improve = (PID_Improvement_e)(PID_Integral_Limit | PID_Derivative_On_Measurement),
};
static PID_Init_Config_s yaw_speed_cfg_enc = {
    .Kp = 1, .Ki = 0, .Kd = 0, .MaxOut = 10, .IntegralLimit = 10,
    .Improve = (PID_Improvement_e)(PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter),
};

static int yaw_feed_now = -1;   /* -1 表示上电后尚未设置过 */

/* 反馈源和 PID 参数一起切,只在切换的那一拍执行。
   PIDInit() 内部先 memset 整个 PIDInstance(controller.c:231),顺带清掉
   积分/微分历史,正好避免两个坐标系之间的历史互相污染 */
static void YawSetFeedSource(Feedback_Source_e src)
{
    if ((int)src == yaw_feed_now)
        return;
    yaw_feed_now = (int)src;

    DMMotorChangeFeed(yaw_motor, ANGLE_LOOP, src);
    DMMotorChangeFeed(yaw_motor, SPEED_LOOP, src);

    if (src == OTHER_FEED)   /* IMU */
    {
        PIDInit(&yaw_motor->angle_PID, &yaw_angle_cfg_imu);
        PIDInit(&yaw_motor->speed_PID, &yaw_speed_cfg_imu);
    }
    else                     /* 电机编码器 */
    {
        PIDInit(&yaw_motor->angle_PID, &yaw_angle_cfg_enc);
        PIDInit(&yaw_motor->speed_PID, &yaw_speed_cfg_enc);
    }
}
//static BMI088Instance *bmi088; // 云台IMU
void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
			.can_init_config = {
				.can_handle = &hcan1,
				.tx_id = 0x01,
				.rx_id = 0x11,
    },
			.controller_param_init_config = {
        .angle_PID = { .Kp=1, .Ki=0, .Kd=0, .MaxOut=50,
                       .IntegralLimit=10, 
											 .Improve = (PID_Improvement_e)(PID_Integral_Limit|PID_Derivative_On_Measurement) },
        .speed_PID = { .Kp=1, .Ki=0, .Kd=0, .MaxOut=10,
                       .IntegralLimit=10, 
				               .Improve = (PID_Improvement_e)(PID_Integral_Limit|PID_Derivative_On_Measurement|PID_OutputFilter) },
				.other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
			  .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
		},			
			.controller_setting_init_config = {
				.angle_feedback_source = OTHER_FEED,
				.speed_feedback_source = OTHER_FEED,
				.outer_loop_type = ANGLE_LOOP,
				.close_loop_type = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP),
				.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		},
			.motor_type = DM4310
	};
	//BP
	Motor_Init_Config_s big_pitch_config = {
			.can_init_config = {
				.can_handle = &hcan2,
				.tx_id = 0x02,
				.rx_id = 0x12,
    },
			.controller_param_init_config = {
        .angle_PID = { .Kp=35, .Ki=0, .Kd=0, .MaxOut=50,
                       .IntegralLimit=10, 
											 .Improve = (PID_Improvement_e)(PID_Integral_Limit|PID_Derivative_On_Measurement) },
        .speed_PID = { .Kp=1, .Ki=0, .Kd=0, .MaxOut=10,
                       .IntegralLimit=10, 
				               .Improve = (PID_Improvement_e)(PID_Integral_Limit|PID_Derivative_On_Measurement|PID_OutputFilter) },
				.other_angle_feedback_ptr = NULL,
			  .other_speed_feedback_ptr = NULL,
		},			
			.controller_setting_init_config = {
				.angle_feedback_source = MOTOR_FEED,
				.speed_feedback_source = MOTOR_FEED,
				.outer_loop_type = ANGLE_LOOP,
				.close_loop_type = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP),
				.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		},
			.motor_type = DM4340
	};
	//SP
	Motor_Init_Config_s small_pitch_config = {
			.can_init_config = {
				.can_handle = &hcan2,
				.tx_id = 0x03,
				.rx_id = 0x13,
    },
			.controller_param_init_config = {
        .angle_PID = { .Kp=0.8, .Ki=0, .Kd=0.02, .MaxOut=100,
                       .IntegralLimit=10, 
											 .Improve = (PID_Improvement_e)(PID_Integral_Limit|PID_Derivative_On_Measurement) },
        .speed_PID = { .Kp=0.8, .Ki=0, .Kd=0, .MaxOut=100,
                       .IntegralLimit=10, 
				               .Improve = (PID_Improvement_e)(PID_Integral_Limit|PID_Derivative_On_Measurement|PID_OutputFilter) },
				.other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
			  .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[0],
		},			
			.controller_setting_init_config = {
				.angle_feedback_source = OTHER_FEED,
				.speed_feedback_source = OTHER_FEED,
				.outer_loop_type = ANGLE_LOOP,
				.close_loop_type = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP),
				.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		},
			.motor_type = DM4310
	};
	yaw_motor = DMMotorInit(&yaw_config);
	big_pitch_motor = DMMotorInit(&big_pitch_config);
	small_pitch_motor = DMMotorInit(&small_pitch_config);
	
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
	
	DMMotorControlInit();
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
	
		VisionSetFlag(COLOR_RED, VISION_MODE_AIM, SMALL_AMU_15);
    VisionSetAltitude(QEKF_INS.Yaw, QEKF_INS.Pitch, 0);
    VisionSend();
	
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
		DMMotorStop(yaw_motor);
		DMMotorStop(big_pitch_motor);
		DMMotorStop(small_pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
		DMMotorEnable(yaw_motor);
    DMMotorEnable(big_pitch_motor);
		DMMotorEnable(small_pitch_motor);
		YawSetFeedSource(OTHER_FEED);
		DMMotorOuterLoop(yaw_motor,ANGLE_LOOP);
		DMMotorSetRef(yaw_motor,gimbal_cmd_recv.yaw);
		DMMotorSetRef(big_pitch_motor,gimbal_cmd_recv.Bpitch);
		DMMotorSetRef(small_pitch_motor,gimbal_cmd_recv.Spitch);        
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
		DMMotorEnable(yaw_motor);
		DMMotorEnable(big_pitch_motor);
		DMMotorEnable(small_pitch_motor);
		YawSetFeedSource(MOTOR_FEED);
		DMMotorOuterLoop(yaw_motor,ANGLE_LOOP);
		DMMotorSetRef(yaw_motor,gimbal_cmd_recv.yaw);
		DMMotorSetRef(big_pitch_motor,gimbal_cmd_recv.Bpitch);
		DMMotorSetRef(small_pitch_motor,gimbal_cmd_recv.Spitch);
		break;
		// 360巡航模式
		case GIMBAL_AUTO_MODE:
		DMMotorEnable(yaw_motor);
		DMMotorEnable(big_pitch_motor);
		DMMotorEnable(small_pitch_motor);
		YawSetFeedSource(MOTOR_FEED);
		DMMotorOuterLoop(yaw_motor,SPEED_LOOP);
		DMMotorSetRef(yaw_motor,gimbal_cmd_recv.yaw);
		DMMotorSetRef(big_pitch_motor,gimbal_cmd_recv.Bpitch);
		DMMotorSetRef(small_pitch_motor,gimbal_cmd_recv.Spitch);
       break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
//    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
//    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.position;
	gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.position;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
#endif
