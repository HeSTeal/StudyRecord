#include "gimbal.h"
#include "vision_types.h"
#ifdef GIMBAL_BOARD

#include "robot_def.h"
#include "DMmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "master_process.h"

extern Vision_Recv_s *vision_recv;

/* -------------------- ģ���ڲ�״̬ -------------------- */
static attitude_t        *gimba_IMU_data;                              // ��̨ IMU ����
 DMMotorInstance   *yaw_motor, *big_pitch_motor, *small_pitch_motor;

static Publisher_t       *gimbal_pub;                                  // ��̨ -> cmd ��������
static Subscriber_t      *gimbal_sub;                                  // cmd -> ��̨���ƶ���
static Gimbal_Upload_Data_s gimbal_feedback_data;
static Gimbal_Ctrl_Cmd_s    gimbal_cmd_recv;

/* -------------------- yaw ���� PID ---Q-----------------
 * ���ַ���Դ�ĵ�λ/���/��������ͬ������ֱ�������
 *   IMU ����   : �Ƕ� = YawTotalAngle(��, ��Ȧ) / �ٶ� = Gyro[2](rad/s)
 *   ���������� : �Ƕ� = measure.position(��3.14) / �ٶ� = measure.velocity(Լ 1.5��rad/s)
 */
static PID_Init_Config_s yaw_angle_cfg_imu = {
    .Kp = 1.1, .Ki = 0, .Kd = 0.006,
    .MaxOut = 50, .IntegralLimit = 10,
    .Improve = (PID_Improvement_e)(PID_Integral_Limit | PID_Derivative_On_Measurement),
};
static PID_Init_Config_s yaw_speed_cfg_imu = {
    .Kp = 1.0, .Ki = 0.1, .Kd = 0,
    .MaxOut = 10, .IntegralLimit = 10,
    .Improve = (PID_Improvement_e)(PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter),
};

static PID_Init_Config_s yaw_angle_cfg_enc = {
    .Kp = 20, .Ki = 0, .Kd = 0.0,
    .MaxOut = 120, .IntegralLimit = 50,
    .Improve = (PID_Improvement_e)(PID_Integral_Limit | PID_Derivative_On_Measurement),
};
static PID_Init_Config_s yaw_speed_cfg_enc = {
    .Kp = 1, .Ki = 0, .Kd = 0,
    .MaxOut = 10, .IntegralLimit = 10,
    .Improve = (PID_Improvement_e)(PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter),
};

static int yaw_feed_now = -1;   // -1 ��ʾ�ϵ����δ���ù�

/* ����Դ�� PID ����һ���У�ֻ���л���һ��ִ�С�
 * PIDInit() �ڲ��� memset ���� PIDInstance��˳���������/΢����ʷ��
 * ���ñ�����������ϵ֮�����ʷ������Ⱦ��
 */
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
    else                     /* ��������� */
    {
        PIDInit(&yaw_motor->angle_PID, &yaw_angle_cfg_enc);
        PIDInit(&yaw_motor->speed_PID, &yaw_speed_cfg_enc);
    }
}

/* -------------------- ��ʼ�� -------------------- */
void GimbalInit()
{
    gimba_IMU_data = INS_Init();   // IMU �ȳ�ʼ����yaw/pitch ʹ������̬����

    /* YAW��Ĭ��ʹ�� IMU ���� */
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 0x01,
            .rx_id      = 0x11,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 1.1, .Ki = 0, .Kd = 0.006,
                .MaxOut = 50, .IntegralLimit = 10,
                .Improve = (PID_Improvement_e)(PID_Integral_Limit | PID_Derivative_On_Measurement),
            },
            .speed_PID = {
                .Kp = 1.0, .Ki = 0.1, .Kd = 0,
                .MaxOut = 10, .IntegralLimit = 10,
                .Improve = (PID_Improvement_e)(PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter),
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP),
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = DM4310,
    };

    /* �� pitch */
    Motor_Init_Config_s big_pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id      = 0x02,
            .rx_id      = 0x12,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 35, .Ki = 0, .Kd = 0,
                .MaxOut = 50, .IntegralLimit = 10,
                .Improve = (PID_Improvement_e)(PID_Integral_Limit | PID_Derivative_On_Measurement),
            },
            .speed_PID = {
                .Kp = 1.1, .Ki = 0, .Kd = 0,
                .MaxOut = 10, .IntegralLimit = 10,
                .Improve = (PID_Improvement_e)(PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter),
            },
            .other_angle_feedback_ptr = NULL,
            .other_speed_feedback_ptr = NULL,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP),
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = DM4340,
    };

    /* С pitch */
    Motor_Init_Config_s small_pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id      = 0x03,
            .rx_id      = 0x13,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 1.0, .Ki = 0, .Kd = 0.005,
                .MaxOut = 100, .IntegralLimit = 10,
                .Improve = (PID_Improvement_e)(PID_Integral_Limit | PID_Derivative_On_Measurement),
            },
            .speed_PID = {
                .Kp = 0.9, .Ki = 0, .Kd = 0,
                .MaxOut = 100, .IntegralLimit = 10,
                .Improve = (PID_Improvement_e)(PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter),
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[0],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP),
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = DM4310,
    };

    yaw_motor         = DMMotorInit(&yaw_config);
    big_pitch_motor   = DMMotorInit(&big_pitch_config);
    small_pitch_motor = DMMotorInit(&small_pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd",  sizeof(Gimbal_Ctrl_Cmd_s));

    DMMotorControlInit();
}

/* -------------------- ģʽ���� -------------------- */

/* ����"ͨ������"ģʽ�Ĺ����Ǽܣ�
 * enable ȫ����� -> ���� yaw ����Դ -> ���� yaw �⻷ -> �·�����ο�
 */
static void GimbalRunMode(Feedback_Source_e feed, Closeloop_Type_e outer_loop)
{
    DMMotorEnable(yaw_motor);
    DMMotorEnable(big_pitch_motor);
    DMMotorEnable(small_pitch_motor);

    YawSetFeedSource(feed);
    DMMotorOuterLoop(yaw_motor, outer_loop);

    DMMotorSetRef(yaw_motor,         gimbal_cmd_recv.yaw);
    DMMotorSetRef(big_pitch_motor,   gimbal_cmd_recv.Bpitch);
    DMMotorSetRef(small_pitch_motor, gimbal_cmd_recv.Spitch);
}

static void GimbalHandleZeroForce(void)
{
    DMMotorStop(yaw_motor);
    DMMotorStop(big_pitch_motor);
    DMMotorStop(small_pitch_motor);
}
/* -------------------- �Զ�����״̬ -------------------- */
// #define YAW_AUTO_STEP_DEG    180.0f
// #define YAW_AUTO_STEP_TICKS  1000U   // ������ 1kHz����ÿ 1 ��תһ��

// static uint32_t yaw_step_cnt = 0;
// static float    yaw_step_off = 0.0f;
// static int yaw_last_mode = -1;   // �� int ��¼�������Ƚϵ�������ö��ֵ
/* -------------------- ������� -------------------- */
void GimbalTask()
{
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    VisionSetFlag(COLOR_BLUE, VISION_MODE_AIM, SMALL_AMU_15);
    VisionSetAltitude(QEKF_INS.Yaw, QEKF_INS.Pitch, 0);
    VisionSend();

    // if (gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE &&
    // yaw_last_mode != GIMBAL_GYRO_MODE)
    // {
    //     yaw_step_cnt = 0;
    //     yaw_step_off = 0.0f;
    // }
    // yaw_last_mode = (int)gimbal_cmd_recv.gimbal_mode;
    /* ��������ֻ���� IMU ������yaw ����� offset �������õ��̸�����̨��
     * �Ӿ�ģʽ�� robot_cmd �����ȷ����gimbal ֻ�� yaw_ref / pitch_ref�� */
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_ZERO_FORCE:
        GimbalHandleZeroForce();
        break;

    /* �����Ƿ��������̸��� yaw ��� offset ������̨���Ӿ�ģʽҲ������ */
    case GIMBAL_GYRO_MODE:
    //    if (++yaw_step_cnt >= YAW_AUTO_STEP_TICKS)
    //     {
    //         yaw_step_cnt = 0;
    //         yaw_step_off += YAW_AUTO_STEP_DEG;
    //     }

    //     GimbalRunMode(OTHER_FEED, ANGLE_LOOP);
    //     DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw + yaw_step_off);
        GimbalRunMode(OTHER_FEED, ANGLE_LOOP);
        break;

    /* ��������������̨����̽������̨��ת�����ڵ���̬/�������ص� */
    case GIMBAL_FREE_MODE:
        GimbalRunMode(MOTOR_FEED, ANGLE_LOOP);
        break;

    /* 360 Ѳ�������������� + �ٶ��⻷ */
    case GIMBAL_AUTO_MODE:
        GimbalRunMode(OTHER_FEED, SPEED_LOOP);
        break;

    default:
        break;
    }

    /* @todo pitch ��������ǰ�����أ�
     *   ���� IMU ��̬ / pitch ����Ƕȷ������㵱ǰ�����µ������أ�
     *   ���� big_pitch / small_pitch �Ĳο�������ϡ� */

    /* �������ݣ���Ҫ�� IMU �� yaw ��Ȧ�� */
    gimbal_feedback_data.gimbal_imu_data               = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle  = yaw_motor->measure.position;

    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
#endif
