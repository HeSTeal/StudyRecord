#include "chassis.h"
#include "robot_def.h"
#ifdef CHASSIS_BOARD

#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "math.h"
#include "DJIMotor.h"
#include "SMC.h"

#include "can_comm.h"
//#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; //
extern Navi_Recv_s *navi_recv;            // 

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 
static Chassis_Upload_Data_s chassis_feedback_data ;
static DJIMotorInstance *motor_lf,*motor_rf,*motor_lb,*motor_rb;
// PIDInstance chassis_follow_pid;  
// PIDInstance chassis_follow_pid;      // 外环：角度环，输入=相对角，输出=目标角速度
// PIDInstance chassis_follow_wz_pid;   // 内环：角速度环，输入=目标角速度，反馈=real_wz
// static float t;

static float chassis_vx, chassis_vy,chassis_vw;  
static float vt_lf, vt_rf, vt_lb, vt_rb;    

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
    //---------- 跟随单环 ----------
    // PID_Init_Config_s follow_pid_config = {
    //  .Kp= 9.0f,.Ki= 0.0f,.Kd= 0.8f,                         
    //  .MaxOut= 6.28f,.DeadBand= 0.01f,.Improve= PID_Derivative_On_Measurement, 
    // };
    // PIDInit(&chassis_follow_pid, &follow_pid_config);
    // ---------- 跟随双环 ----------
    // PID_Init_Config_s follow_angle_pid_config = {
    //  .Kp = 10.0f, .Ki = 0.0f, .Kd = 0.3f,   
    //  .MaxOut = 10.0f,                       
    //  .DeadBand = 0.005f,
    //  .Improve = PID_Integral_Limit,
    // };
    // PIDInit(&chassis_follow_pid, &follow_angle_pid_config);
    // PID_Init_Config_s follow_wz_pid_config = {
    //  .Kp = 1.2f, .Ki = 0.0f, .Kd = 0.0f,
    //  .MaxOut = 6.28f,                       
    //  .DeadBand = 0.01f,
    //  .Improve = PID_Derivative_On_Measurement | PID_Integral_Limit,
    // };
    // PIDInit(&chassis_follow_wz_pid, &follow_wz_pid_config);
#ifdef CHASSIS_BOARD
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
#endif
}
//坐标系旋转
static void RotateVec(float angle, float ix, float iy, float *ox, float *oy)
{
    float c = arm_cos_f32(angle-0.007*chassis_cmd_recv.wz);
    float s = arm_sin_f32(angle-0.007*chassis_cmd_recv.wz);
    *ox = ix * c - iy * s;
    *oy = ix * s + iy * c;
}
//底盘正解算速度
static void ForwardKinematics(float *vx, float *vy, float *vw)
{
    float v_lf = motor_lf->measure.speed_aps / WHEEL_V_TO_APS;
    float v_rf = motor_rf->measure.speed_aps / WHEEL_V_TO_APS;
    float v_lb = motor_lb->measure.speed_aps / WHEEL_V_TO_APS;
    float v_rb = motor_rb->measure.speed_aps / WHEEL_V_TO_APS;

    *vx = ((v_lf + v_lb) - (v_rf + v_rb)) / (4.0f * 1.414f);
    *vy = ((v_lb + v_rb) - (v_lf + v_rf)) / (4.0f * 1.414f);
    *vw =  (v_lf + v_rf + v_lb + v_rb) / (4.0f * WHEEL_DISTANCE);
}
static void EstimateSpeed(void)
{
    ForwardKinematics(&chassis_feedback_data.real_vx,&chassis_feedback_data.real_vy,&chassis_feedback_data.real_wz);
}
//底盘运动学解算
static void MecanumCalculate()
{
    vt_lf = (SQRT2*( chassis_vx - chassis_vy) + chassis_vw * WHEEL_DISTANCE)*WHEEL_V_TO_APS;
    vt_rf = (SQRT2*(-chassis_vx - chassis_vy) + chassis_vw * WHEEL_DISTANCE)*WHEEL_V_TO_APS;
    vt_lb = (SQRT2*( chassis_vx + chassis_vy) + chassis_vw * WHEEL_DISTANCE)*WHEEL_V_TO_APS;
    vt_rb = (SQRT2*(-chassis_vx + chassis_vy) + chassis_vw * WHEEL_DISTANCE)*WHEEL_V_TO_APS;
        
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

void ChassisTask()
{
    EstimateSpeed();
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { 
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
        return;
    }
    else
    { 
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    switch (chassis_cmd_recv.chassis_mode)
    {
            case CHASSIS_NO_FOLLOW: 
            case CHASSIS_ROTATE:        
            {
                chassis_vw=chassis_cmd_recv.wz;
                RotateVec(chassis_cmd_recv.offset_angle,chassis_cmd_recv.vx,chassis_cmd_recv.vy,&chassis_vx,&chassis_vy);
            }break;     
            
            case CHASSIS_FOLLOW_GIMBAL_YAW: //锟斤拷锟斤拷
            {           
                // // 外环：相对角偏差 -> 目标角速度（保留原来的负号，方向不对再调）
                // float target_wz = -PIDCalculate(&chassis_follow_pid,chassis_cmd_recv.offset_angle,GIMBAL_FOLLOW_TARGET_OFFSET);
                // // 内环：目标角速度 vs 底盘实际角速度 -> 底盘旋转输出
                // chassis_vw = -PIDCalculate(&chassis_follow_wz_pid,target_wz,chassis_feedback_data.real_wz);
                // 单环
                // chassis_vw = -PIDCalculate(&chassis_follow_pid,chassis_cmd_recv.offset_angle,GIMBAL_FOLLOW_TARGET_OFFSET);
                chassis_vw = ChassisFollowSMC(chassis_cmd_recv.offset_angle,GIMBAL_FOLLOW_TARGET_OFFSET,chassis_feedback_data.real_wz);
                chassis_vx = chassis_cmd_recv.vx;
                chassis_vy = chassis_cmd_recv.vy;
            }break;
            
            case CHASSIS_NAVIGATION: 
            case CHASSIS_COMPETITION:
                chassis_vx = navi_recv->V_X;
                chassis_vy = navi_recv->V_Y;
                if(chassis_cmd_recv.wz==0){
                    chassis_vw = navi_recv->V_Z;
                }
                else{
                    chassis_vw = chassis_cmd_recv.wz;   
                }
                chassis_feedback_data.chassis_type = (uint8_t)navi_recv->chassis_type;
                // chassis_feedback_data.chassis_type = 1;
            break;    
            default:
                chassis_vx = 0.0f;
                chassis_vy = 0.0f;
                chassis_vw = 0.0f;
            break;
    }
    MecanumCalculate();    

    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif
}
#endif


