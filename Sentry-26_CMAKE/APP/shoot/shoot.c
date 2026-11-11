// #include "shoot.h"
// #include "robot_def.h"
//
// #include "DJImotor.h"
// #include "message_center.h"
// #include "bsp_dwt.h"
// #include "general_def.h"
//
// #include "usart.h"
//
// DJIMotorInstance *friction_l, *friction_r, *loader;
//
// static Publisher_t *shoot_pub;
// static Shoot_Ctrl_Cmd_s shoot_cmd_recv;
// static Subscriber_t *shoot_sub;
// static Shoot_Upload_Data_s shoot_feedback_data;
//
// static uint8_t single_shot_active = 0;
// static float single_shot_target = 0;
//
// static uint8_t three_shot_active = 0;
// static float three_shot_target = 0;
//
// /* 卡弹处理 */
// static uint16_t card_time      = 0;
// static uint8_t  card_mode      = 0;   // 0正常 1退弹
// static uint8_t  solve_mode     = 0;   // 0等待 1执行
// static uint16_t solve_mode_CNT = 0;
//
// void ShootInit()
// {
//     Motor_Init_Config_s friction_config = {
//         .can_init_config = {
//             .can_handle = &hcan2,
//         },
//         .controller_param_init_config = {
//             .speed_PID = {
//                 .Kp = 4.0,
//                 .Ki = 0,
//                 .Kd = 0,
//                 .Improve = PID_Integral_Limit,
//                 .IntegralLimit = 10000,
//                 .MaxOut = 15000,
//             },
//         },
//         .controller_setting_init_config = {
//             .angle_feedback_source = MOTOR_FEED,
//             .speed_feedback_source = MOTOR_FEED,
//             .outer_loop_type = SPEED_LOOP,
//             .close_loop_type = (Closeloop_Type_e)(SPEED_LOOP),
//             .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
//         },
//         .motor_type = M3508
//     };
//
//     friction_config.can_init_config.tx_id = 2;
//     friction_l = DJIMotorInit(&friction_config);
//
//     friction_config.can_init_config.tx_id = 3;
//     friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
//     friction_r = DJIMotorInit(&friction_config);
//
//     Motor_Init_Config_s loader_config = {
//         .can_init_config = {
//             .can_handle = &hcan1,
//             .tx_id = 1,
//         },
//         .controller_param_init_config = {
//             .angle_PID = {
//                 .Kp = 35,
//                 .Ki = 0,
//                 .Kd = 0,
//                 .MaxOut = 16000,
//             },
//             .speed_PID = {
//                 .Kp = 0.3,
//                 .Ki = 0,
//                 .Kd = 0,
//                 .Improve = PID_Integral_Limit,
//                 .IntegralLimit = 5000,
//                 .MaxOut = 16000,
//             },
//         },
//         .controller_setting_init_config = {
//             .angle_feedback_source = MOTOR_FEED,
//             .speed_feedback_source = MOTOR_FEED,
//             .outer_loop_type = SPEED_LOOP,
//             .close_loop_type = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP),
//             .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
//         },
//         .motor_type = M2006
//     };
//
//     loader = DJIMotorInit(&loader_config);
//
//     shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
//     shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
// }
//
// static void SingleShotControl()
// {
//     float angle_error;
//     angle_error = single_shot_target - loader->measure.total_angle;
//
//     if (angle_error > -30.0f && angle_error < 30.0f)
//     {
//         single_shot_active = 0;
//         DJIMotorOuterLoop(loader, SPEED_LOOP);
//         DJIMotorSetRef(loader, 0);
//         return;
//     }
//
//     DJIMotorOuterLoop(loader, ANGLE_LOOP);
//     DJIMotorSetRef(loader, single_shot_target);
// }
//
// static void ThreeShotControl()
// {
//     float angle_error;
//     angle_error = three_shot_target - loader->measure.total_angle;
//
//     if (angle_error > -200.0f && angle_error < 200.0f)
//     {
//         three_shot_active = 0;
//         DJIMotorOuterLoop(loader, SPEED_LOOP);
//         DJIMotorSetRef(loader, 0);
//         return;
//     }
//
//     DJIMotorOuterLoop(loader, ANGLE_LOOP);
//     DJIMotorSetRef(loader, three_shot_target);
// }
//
// static void card_Bullet(void)  // 卡弹处理
// {
//     if (solve_mode == 0)      // 阶段0：检测堵弹
//     {
//         if (card_mode == 0)
//         {
//             if (fabsf(loader->motor_controller.speed_PID.Output) >= JAM_OUTPUT_TH &&
//                 fabsf(loader->measure.speed_aps) <= JAM_SPEED_TH)
//             {
//                 card_time++;
//                 if (card_time >= JAM_DETECT_CNT)
//                     card_mode = 1;
//             }
//             else
//             {
//                 card_time = 0;
//             }
//         }
//         else if (card_mode == 1)  // 只进一次 阶段1：退弹一次
//         {
//             DJIMotorOuterLoop(loader, ANGLE_LOOP);
//             DJIMotorSetRef(loader, loader->measure.total_angle - JAM_REVERSE_ANGLE);
//             solve_mode = 1;   // 进入阶段2
//         }
//     }
//     else if (solve_mode == 1)  // 阶段2：延时复位
//     {
//         solve_mode_CNT++;
//         if (solve_mode_CNT == JAM_WAIT_CNT)
//         {
//             solve_mode = 0;
//             card_mode = 0;
//             card_time = 0;
//             solve_mode_CNT = 0;
//         }
//     }
// }
//
// static void LoaderControl()
// {
//     card_Bullet();
//
//     if (solve_mode == 1)
//         return;
//
//     switch (shoot_cmd_recv.load_mode)
//     {
//         case LOAD_STOP:
//             if (single_shot_active)
//             {
//                 SingleShotControl();
//             }
//             else if (three_shot_active)
//             {
//                 ThreeShotControl();
//             }
//             else
//             {
//                 DJIMotorOuterLoop(loader, SPEED_LOOP);
//                 DJIMotorSetRef(loader, 0);
//             }
//             break;
//
//         case LOAD_1_BULLET:
//             if (!single_shot_active)
//             {
//                 single_shot_active = 1;
//                 single_shot_target = loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE;
//             }
//             SingleShotControl();
//             break;
//
//         case LOAD_3_BULLET:
//             if (!three_shot_active)
//             {
//                 three_shot_active = 1;
//                 three_shot_target = loader->measure.total_angle + 3.0f * ONE_BULLET_DELTA_ANGLE;
//             }
//             ThreeShotControl();
//             break;
//
//         case LOAD_BURSTFIRE:
//             single_shot_active = 0;
//             three_shot_active = 0;
//             DJIMotorOuterLoop(loader, SPEED_LOOP);
//             DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360.0f * REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE);
//             break;
//
//         case LOAD_REVERSE:
//             single_shot_active = 0;
//             three_shot_active = 0;
//             DJIMotorOuterLoop(loader, SPEED_LOOP);
//             DJIMotorSetRef(loader, -JAM_REVERSE_SPEED);
//             break;
//
//         default:
//             while (1);
//     }
// }
//
// void ShootTask()
// {
//     SubGetMessage(shoot_sub, &shoot_cmd_recv);
//
//     if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
//     {
//         DJIMotorStop(friction_l);
//         DJIMotorStop(friction_r);
//         DJIMotorStop(loader);
//
//         single_shot_active = 0;
//         three_shot_active = 0;
//
//         card_time      = 0;
//         card_mode      = 0;
//         solve_mode     = 0;
//         solve_mode_CNT = 0;
//     }
//     else
//     {
//         DJIMotorEnable(friction_l);
//         DJIMotorEnable(friction_r);
//         DJIMotorEnable(loader);
//
//         LoaderControl();
//     }
//
//     if (shoot_cmd_recv.friction_mode == FRICTION_ON)
//     {
//         switch (shoot_cmd_recv.bullet_speed)
//         {
//             case SMALL_AMU_15:
//                 DJIMotorSetRef(friction_l, 0);
//                 DJIMotorSetRef(friction_r, 0);
//                 break;
//
//             case SMALL_AMU_18:
//                 DJIMotorSetRef(friction_l, 0);
//                 DJIMotorSetRef(friction_r, 0);
//                 break;
//
//             case SMALL_AMU_30:
//                 DJIMotorSetRef(friction_l, 0);
//                 DJIMotorSetRef(friction_r, 0);
//                 break;
//
//             default:
//                 DJIMotorSetRef(friction_l, 30000);
//                 DJIMotorSetRef(friction_r, 30000);
//                 break;
//         }
//     }
//     else
//     {
//         DJIMotorSetRef(friction_l, 0);
//         DJIMotorSetRef(friction_r, 0);
//     }
//
//     /* 更新堵弹状态: 0 正常 / 1 退弹 / 2 等待 */
//     if (solve_mode == 1)      shoot_feedback_data.jam_state = 2;
//     else if (card_mode == 1)  shoot_feedback_data.jam_state = 1;
//     else                      shoot_feedback_data.jam_state = 0;
//
//     PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
// }


#include "shoot.h"
#include "robot_def.h"

#include "DJImotor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

#include "usart.h"

#include "SMC.h"
#include <math.h>

DJIMotorInstance *friction_l, *friction_r, *loader;

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data;

static uint8_t single_shot_active = 0;
static float   single_shot_target = 0;

static uint8_t three_shot_active = 0;
static float   three_shot_target = 0;

/* 卡弹处理 */
static uint16_t card_time      = 0;
static uint8_t  card_mode      = 0;   // 0正常 1退弹
static uint8_t  solve_mode     = 0;   // 0等待 1执行
static uint16_t solve_mode_CNT = 0;
static float    jam_reverse_target = 0.0f;

/* 记录上一拍的 load_mode,用于切换时清 SMC 积分 */
static loader_mode_e last_load_mode = LOAD_STOP;

/* ==================== 拨盘双环 SMC ==================== */
 SMC loader_pos_smc;
 SMC loader_vel_smc;
/* 只清积分和微分缓存, 不动 position_error_threshold (SMC_Clear 会把阈值也清掉, 不要用) */
static void LoaderSMCReset(void)
{
    SMC_Integval_Clear(&loader_pos_smc);
    SMC_Integval_Clear(&loader_vel_smc);

    loader_pos_smc.error.target_previous      = loader_pos_smc.error.target_current;
    loader_vel_smc.error.target_previous      = loader_vel_smc.error.target_current;
    loader_pos_smc.error.target_derivative_prev = 0.0f;
    loader_vel_smc.error.target_derivative_prev = 0.0f;
}

void ShootInit()
{
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 1.5,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = (Closeloop_Type_e)(SPEED_LOOP),
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508
    };

    friction_config.can_init_config.tx_id = 2;
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 3;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    /* 拨盘: 关闭内置所有闭环, pid_ref 直通作为 CAN 电流, 由 SMC 负责 */
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 30,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 16000,
            },
            .speed_PID = {
                .Kp = 0.3,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 16000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,                /* 已无影响 */
            .close_loop_type = (Closeloop_Type_e)0,       /* 关键: 全部内置环关闭 */
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006
    };

    loader = DJIMotorInit(&loader_config);

    SMC_Init(&loader_pos_smc,
             POS_SMC_C, POS_SMC_K, POS_SMC_THRESHOLD,
             POS_SMC_UMAX, POS_SMC_J, POS_SMC_EPSILON,
             EXPONENT, 0, 0, 0, 0, 0);

    SMC_Init(&loader_vel_smc,
             VEL_SMC_C, VEL_SMC_K, VEL_SMC_THRESHOLD,
             VEL_SMC_UMAX, VEL_SMC_J, VEL_SMC_EPSILON,
             VELSMC, 0, 0, 0, 0, 0);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 位置环 SMC: 输出速度指令 (aps) */
static float LoaderPosSMC(float target_angle)
{
    float cur = loader->measure.total_angle;
    float vel = loader->measure.speed_aps;
    float pos_err = cur - target_angle;      /* 与 SMC 内部符号一致 (pos - ref) */

    loader_pos_smc.error.target_current   = target_angle;
    loader_pos_smc.error.position_current = cur;
    loader_pos_smc.error.velocity_current = vel;
    loader_pos_smc.error.position_error   = pos_err;
    loader_pos_smc.error.velocity_error   = vel;    /* v_ref = 0, 只做阻尼 */
    loader_pos_smc.error.target_derivative        = 0.0f;  /* 阶梯目标必须置 0 */
    loader_pos_smc.error.target_second_derivative = 0.0f;

    /* EXPONENT 模式其实不用积分, 保留以备切换到 EISMC */
    loader_pos_smc.error.position_error_integral += pos_err * LOADER_DT;
    if (fabsf(loader_pos_smc.error.position_error_integral) > position_integral_max)
        loader_pos_smc.error.position_error_integral =
            position_integral_max * SMC_Signal(loader_pos_smc.error.position_error_integral);

    return SMC_SmcCalculate(&loader_pos_smc);
}

/* 速度环 SMC: 输出电流 */
static float LoaderVelSMC(float target_speed)
{
    float cur = loader->measure.speed_aps;

    loader_vel_smc.error.target_current   = target_speed;
    loader_vel_smc.error.velocity_current = cur;
    loader_vel_smc.error.velocity_error   = cur - target_speed;
    loader_vel_smc.error.target_derivative        = 0.0f;  /* 不做前馈 */
    loader_vel_smc.error.target_second_derivative = 0.0f;

    loader_vel_smc.error.velocity_error_integral += loader_vel_smc.error.velocity_error * LOADER_DT;
    if (fabsf(loader_vel_smc.error.velocity_error_integral) > velocity_integral_max)
        loader_vel_smc.error.velocity_error_integral =
            velocity_integral_max * SMC_Signal(loader_vel_smc.error.velocity_error_integral);

    return SMC_SmcCalculate(&loader_vel_smc);
}

/* 双环: 位置目标 -> 电流 */
static void LoaderSetAngleSMC(float target_angle)
{
    float vel_cmd = LoaderPosSMC(target_angle);
    float cur_cmd = LoaderVelSMC(vel_cmd);
    DJIMotorSetRef(loader, cur_cmd);
}

/* 单速度环: 速度目标 -> 电流 */
static void LoaderSetSpeedSMC(float target_speed)
{
    float cur_cmd = LoaderVelSMC(target_speed);
    DJIMotorSetRef(loader, cur_cmd);
}

static void SingleShotControl()
{
    float angle_error = single_shot_target - loader->measure.total_angle;

    if (fabsf(angle_error) < POS_SMC_DONE_TH)
    {
        single_shot_active = 0;
        LoaderSMCReset();
        LoaderSetSpeedSMC(0.0f);      /* 速度环主动刹车 */
        return;
    }

    LoaderSetAngleSMC(single_shot_target);
}

static void ThreeShotControl()
{
    float angle_error = three_shot_target - loader->measure.total_angle;

    if (fabsf(angle_error) < POS_SMC_DONE_TH)
    {
        three_shot_active = 0;
        LoaderSMCReset();
        LoaderSetSpeedSMC(0.0f);
        return;
    }

    LoaderSetAngleSMC(three_shot_target);
}

/* 卡弹处理 */
static void card_Bullet(void)
{
    if (solve_mode == 0)
    {
        if (card_mode == 0)
        {
            /* 用速度环 SMC 的输出 u 代替原 speed_PID.Output 做堵转判断 */
            if (fabsf(loader_vel_smc.u) >= JAM_OUTPUT_TH &&
                fabsf(loader->measure.speed_aps) <= JAM_SPEED_TH)
            {
                card_time++;
                if (card_time >= JAM_DETECT_CNT)
                    card_mode = 1;
            }
            else
            {
                card_time = 0;
            }
        }
        else if (card_mode == 1)
        {
            /* 记录反转目标, 交给 LoaderControl 走 SMC 退弹 */
            jam_reverse_target = loader->measure.total_angle - JAM_REVERSE_ANGLE;
            solve_mode = 1;
            LoaderSMCReset();
        }
    }
    else if (solve_mode == 1)
    {
        solve_mode_CNT++;
        if (solve_mode_CNT == JAM_WAIT_CNT)
        {
            solve_mode = 0;
            card_mode = 0;
            card_time = 0;
            solve_mode_CNT = 0;
            LoaderSMCReset();
        }
    }
}

static void LoaderControl()
{
    /* 模式切换时清 SMC 积分, 防止残留 */
    if (shoot_cmd_recv.load_mode != last_load_mode)
    {
        LoaderSMCReset();
        last_load_mode = shoot_cmd_recv.load_mode;
    }

    card_Bullet();

    if (solve_mode == 1)
    {
        /* 退弹: 位置环 SMC 拉到反转目标角度 */
        LoaderSetAngleSMC(jam_reverse_target);
        return;
    }

    switch (shoot_cmd_recv.load_mode)
    {
        case LOAD_STOP:
            if (single_shot_active)
            {
                SingleShotControl();
            }
            else if (three_shot_active)
            {
                ThreeShotControl();
            }
            else
            {
                LoaderSetSpeedSMC(0.0f);
            }
            break;

        case LOAD_1_BULLET:
            if (!single_shot_active)
            {
                single_shot_active = 1;
                single_shot_target = loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE;
                LoaderSMCReset();
            }
            SingleShotControl();
            break;

        case LOAD_3_BULLET:
            if (!three_shot_active)
            {
                three_shot_active = 1;
                three_shot_target = loader->measure.total_angle + 3.0f * ONE_BULLET_DELTA_ANGLE;
                LoaderSMCReset();
            }
            ThreeShotControl();
            break;

        case LOAD_BURSTFIRE:
            single_shot_active = 0;
            three_shot_active = 0;
            LoaderSetSpeedSMC(shoot_cmd_recv.shoot_rate * 360.0f *
                              REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE);
            break;

        case LOAD_REVERSE:
            single_shot_active = 0;
            three_shot_active = 0;
            LoaderSetSpeedSMC(-JAM_REVERSE_SPEED);
            break;

        default:
            while (1);
    }
}

void ShootTask()
{
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);

        single_shot_active = 0;
        three_shot_active = 0;

        card_time      = 0;
        card_mode      = 0;
        solve_mode     = 0;
        solve_mode_CNT = 0;
        last_load_mode = LOAD_STOP;
        LoaderSMCReset();
    }
    else
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);

        LoaderControl();
    }

    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        switch (shoot_cmd_recv.bullet_speed)
        {
            case SMALL_AMU_15:
            case SMALL_AMU_18:
            case SMALL_AMU_30:
                DJIMotorSetRef(friction_l, 0);
                DJIMotorSetRef(friction_r, 0);
                break;

            default:
                DJIMotorSetRef(friction_l, 34000);
                DJIMotorSetRef(friction_r, 34000);
                break;
        }
    }
    else
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    /* 更新堵弹状态: 0 正常 / 1 退弹 / 2 等待 */
    if (solve_mode == 1)      shoot_feedback_data.jam_state = 2;
    else if (card_mode == 1)  shoot_feedback_data.jam_state = 1;
    else                      shoot_feedback_data.jam_state = 0;

    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
