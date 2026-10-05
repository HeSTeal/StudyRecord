// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "DJImotor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
//#include "bsp_log.h"

#include "math.h" // fabsf

/* ==================== cmd应用包含的模块实例和状态 ==================== */
#ifdef GIMBAL_BOARD 
#include "can_comm.h"
static CANCommInstance *cmd_can_comm;        // 双板通信
static Chassis_Ctrl_Cmd_s chassis_cmd_send;  // 发送给底盘应用的信息

static CmdState_s cmdstate_set;              // 需要跨周期记忆的遥控器状态
static uint8_t  shoot_single_pending = 0;    // 单发待发
static uint8_t  shoot_continue       = 0;    // 连发中
static uint8_t  bp_down_latched      = 0;    // 大 pitch 是否已锁在"收起"位
static uint16_t bp_debounce_cnt      = 0;    // 大 pitch 判定去抖计数
#endif

static RC_ctrl_t *rc_data;                   // 遥控器数据

static Publisher_t *gimbal_cmd_pub;
static Subscriber_t *gimbal_feed_sub;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;
static Subscriber_t *shoot_feed_sub;
static Shoot_Ctrl_Cmd_s shoot_cmd_send;        // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data;   // 从发射获取的反馈信息

static Robot_Status_e robot_state;             // 机器人整体工作状态
#ifdef GIMBAL_BOARD
//最短路径折算
static float WrapToPi(float a)
{
    while (a >  3.14159265f) a -= 6.28318531f;
    while (a < -3.14159265f) a += 6.28318531f;
    return a;
}
#endif
//同步Yaw为当前参考角
static void YawRefSyncToIMU(void)
{
    gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
}

//命令中心初始化
void RobotCMDInit(void)
{
    gimbal_cmd_pub  = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub   = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub  = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    gimbal_cmd_send.Spitch = 0.0f;
    gimbal_cmd_send.Bpitch = 0.0f;

#ifdef GIMBAL_BOARD
    rc_data = RemoteControlInit(&huart2);

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);

    /* 底盘默认停止,等遥控器给出模式 */
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 0.0f;
#endif
    robot_state = ROBOT_READY; // 启动时机器人进入工作模式
}

/* ====================== 云台和底盘正方向的夹角 ====================== */
/**
 * @brief 由云台回传的 yaw 电机编码器值算底盘和云台正方向的夹角
 *        单位与编码器一致(±3.14),折算到 ±pi
 */
static void CalcOffsetAngle(void)
{
#ifdef GIMBAL_BOARD
    chassis_cmd_send.offset_angle = WrapToPi(gimbal_fetch_data.yaw_motor_single_round_angle - YAW_ALIGN_ANGLE);
#endif
}
#ifdef GIMBAL_BOARD
/* ============================ 发射控制 ============================ */
/**
 * @brief 发射控制(拨轮归发射使用的模式才调用)
 * @param ev 拨轮事件
 */
static void HandleShoot(DialEvent_e ev)
{
    switch (ev)
    {
        case DIAL_EVENT_SHORT_UP:
            cmdstate_set.friction_on ^= 1;
            break;
        case DIAL_EVENT_SHORT_DOWN:
            shoot_single_pending = 1;
            break;
        case DIAL_EVENT_LONG_DOWN:
            shoot_continue = 1;
            shoot_single_pending = 0;
            break;
        case DIAL_EVENT_RELEASE_DOWN:
            shoot_continue = 0;
            break;
        default:
            break;
    }

    shoot_cmd_send.shoot_mode    = SHOOT_ON;
    shoot_cmd_send.friction_mode = cmdstate_set.friction_on ? FRICTION_ON : FRICTION_OFF;
    shoot_cmd_send.shoot_rate    = 0.0f;
    /* 注意:bullet_speed 这里一直没赋值(= BULLET_SPEED_NONE),shoot.c 会走到 default
       分支给摩擦轮 30000。要按弹速分级就必须两边一起改,见 shoot.c:263 */
    if (shoot_continue)
    {
        shoot_cmd_send.load_mode  = LOAD_BURSTFIRE;
        shoot_cmd_send.shoot_rate = FIRE_BURST_RATE;
    }
    else if (shoot_single_pending)
    {
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        shoot_single_pending = 0;
    }
    else
    {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}
/* 本模式不发射:关摩擦轮、清掉发射意图 */
static void ShootOff(void)
{
    shoot_cmd_send.shoot_mode    = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode     = LOAD_STOP;
    shoot_cmd_send.shoot_rate    = 0.0f;
    shoot_single_pending = 0;
    shoot_continue       = 0;
}
/* ============================ 云台 yaw ============================ */
/**
 * @brief 计算本拍 yaw 的目标值,并决定用哪路反馈
 * @param mode Gimbal_NAVI  : 反馈切到电机编码器,目标锁 GIMBAL_NAVI_YAW_REF(走最短的一边)
 *             Gimbal_Remote: 反馈切回 IMU,切换那一拍重同步,之后叠加摇杆增量
 * @note  反馈源由 gimbal_mode 决定,gimbal.c 按它切 DMMotorChangeFeed
 */
static void HandleGimbalYaw(Gimbal_Control_Mode_e mode)
{
    static uint8_t last_mode = 0xFF;              // 0xFF:上电后还没设置过
    uint8_t entering = ((uint8_t)mode != last_mode); // 模式切换那一拍
    last_mode = (uint8_t)mode;

    if (mode == Gimbal_NAVI)
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE; // -> MOTOR_FEED(编码器)
        float m = gimbal_fetch_data.yaw_motor_single_round_angle;
        gimbal_cmd_send.yaw = m + WrapToPi(GIMBAL_NAVI_YAW_REF - m); // 参考 = 当前值 + 最短角差
    }
    else
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; // -> OTHER_FEED(IMU)
        if (entering)                                   // 从编码器/停车切回来,先重同步
            YawRefSyncToIMU();
        gimbal_cmd_send.yaw += GIMBAL_YAW_SCALE * rc_data[TEMP].rc.rocker_l_ * GYRO2GIMBAL_DIR_YAW;
    }
}
/* ========================== 云台 big pitch ========================== */
/**
 * @brief 计算本拍大 pitch 的目标值
 * @param ev       拨轮事件
 * @param mode     Gimbal_NAVI  : 按 yaw 是否对准自动判定
 *                 Gimbal_Remote: 用 cmdstate_set.Bpitch_target 记的目标位
 * @param use_dial 1 = 本模式拨轮空闲,可用来切换大 pitch 目标位;
 *                 0 = 拨轮被发射/小陀螺占用,忽略 ev
 */
static void HandleGimbalBPitch(DialEvent_e ev, Gimbal_Control_Mode_e mode, uint8_t use_dial)
{
    if (mode == Gimbal_NAVI)
    {
        /* 判定量统一用"目标 - 实际"的折算值,以后改 GIMBAL_NAVI_YAW_REF 不用再挪判定窗口 */
        float err = fabsf(WrapToPi(GIMBAL_NAVI_YAW_REF - gimbal_fetch_data.yaw_motor_single_round_angle));
        uint8_t want = bp_down_latched ? (err < BP_ALIGN_EXIT_TH)   // 迟滞:退出用宽阈值
                                       : (err < BP_ALIGN_ENTER_TH); //       进入用窄阈值

        if (want == bp_down_latched)
        {
            bp_debounce_cnt = 0;                 // 意见一致,不翻转
        }
        else if (++bp_debounce_cnt >= BP_DEBOUNCE_CNT)
        {
            bp_down_latched = want;              // 连续 100ms 意见一致才真的动
            bp_debounce_cnt = 0;
        }
        gimbal_cmd_send.Bpitch = bp_down_latched ? GIMBAL_BIGPITCH_DOWN : GIMBAL_BIGPITCH_UP;
    }
    else
    {
        if (use_dial && ev == DIAL_EVENT_SHORT_DOWN)
            cmdstate_set.Bpitch_target ^= 1;     // 拨轮短按切换大 pitch 目标位

        gimbal_cmd_send.Bpitch = cmdstate_set.Bpitch_target ? GIMBAL_BIGPITCH_DOWN : GIMBAL_BIGPITCH_UP;
        bp_down_latched = 0;                     // 离开导航,清掉自动判定的锁存
        bp_debounce_cnt = 0;
    }
}

/* ========================= 云台 small pitch ========================= */
/**
 * @brief 计算本拍小 pitch 的目标值
 * @param from_stick 1 = 本模式小 pitch 吃左摇杆竖直通道; 0 = 固定水平(0)
todo 封装枚举
 */
static void HandleGimbalSPitch(uint8_t from_stick)
{
    if (from_stick)
        gimbal_cmd_send.Spitch += GIMBAL_SMALL_PITCH_SCALE * rc_data[TEMP].rc.rocker_l1 * GYRO2GIMBAL_DIR_SMALL_PITCH;
    else
        gimbal_cmd_send.Spitch = 0.0f;

    LIMIT_MIN_MAX(gimbal_cmd_send.Spitch, GIMBAL_SMALLPITCH_DOWN_LIMIT, GIMBAL_SMALLPITCH_UP_LIMIT);
}

/* ============================ 底盘 ============================ */
//底盘控制命令函数
static void HandleChassis(chassis_mode_e mode)
{
    chassis_cmd_send.chassis_mode = mode;
    chassis_cmd_send.vx = CHASSIS_SPEED_MAX * ((float)rc_data[TEMP].rc.rocker_r1 / RC_ROCKER_MAX);
    chassis_cmd_send.vy = -CHASSIS_SPEED_MAX * ((float)rc_data[TEMP].rc.rocker_r_ / RC_ROCKER_MAX);
    chassis_cmd_send.wz = 0.0f;
}

//底盘小陀螺命令函数
static void HandleRotateMode(DialEvent_e ev)
{
    switch (ev)
    {
        case DIAL_EVENT_SHORT_UP:
            cmdstate_set.rotate_wz += CHASSIS_ROTATE_WZ_STEP;
            break;
        case DIAL_EVENT_SHORT_DOWN:
            cmdstate_set.rotate_wz -= CHASSIS_ROTATE_WZ_STEP;
            break;
        default:
            break;
    }
    LIMIT_MIN_MAX(cmdstate_set.rotate_wz, -CHASSIS_ROTATE_WZ_MAX, CHASSIS_ROTATE_WZ_MAX);
}
#endif

/* ======================== 遥控器 -> 各应用模式 ======================== */
/**
 * @brief 根据遥控器开关位置,决定本拍三个应用各自的模式和控制量
 * @note  每个分支都把这拍的输出写全(模式 + 目标值),加上开头的默认值,
 *        不会再出现"某个分支漏设 -> 残留上一个模式的值"的问题
 */
static void RemoteControlSet(void)
{
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
    static uint8_t last_left_sw = 0xFF;

    //默认安全输出值，后续进行覆盖
    gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE;   // 默认 IMU,导航分支会覆盖成 FREE
    shoot_cmd_send.shoot_mode     = SHOOT_OFF;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 0.0f;

    //更改左侧拨杆位置后复位拨轮状态机
    uint8_t cur_left_sw = rc_data[TEMP].rc.switch_left;
    if (cur_left_sw != last_left_sw)
    {
        RemoteDialReset();
        last_left_sw = cur_left_sw;
    }
    DialEvent_e dial_event = RemoteDialGetEvent(rc_data[TEMP].rc.dial);

    //拨杆控制模式切换
    if (switch_is_mid(rc_data[TEMP].rc.switch_right))          // 右侧开关[中]
    {
        if (switch_is_down(rc_data[TEMP].rc.switch_left))      // 左[下] 底盘分离
        {
            HandleShoot(dial_event);                           // 拨轮归发射
            HandleGimbalYaw(Gimbal_Remote);                    // yaw   : IMU + 摇杆
            HandleGimbalBPitch(dial_event, Gimbal_Remote, 0);  // 大pitch: 保持目标位
            HandleGimbalSPitch(1);                             // 小pitch: 摇杆
            HandleChassis(CHASSIS_NO_FOLLOW);
        }
        else if (switch_is_mid(rc_data[TEMP].rc.switch_left))  // 左[中] 底盘跟随
        {
            ShootOff();
            HandleGimbalYaw(Gimbal_Remote);
            HandleGimbalBPitch(dial_event, Gimbal_Remote, 1);  // 拨轮空闲: 可切大pitch
            HandleGimbalSPitch(0);
            HandleChassis(CHASSIS_FOLLOW_GIMBAL_YAW);
        }
        else if (switch_is_up(rc_data[TEMP].rc.switch_left))   // 左[上] 小陀螺
        {
            ShootOff();
            HandleRotateMode(dial_event);                      // 拨轮归小陀螺转速
            HandleGimbalYaw(Gimbal_Remote);
            HandleGimbalBPitch(dial_event, Gimbal_Remote, 0);
            HandleGimbalSPitch(1);
            HandleChassis(CHASSIS_ROTATE);
            chassis_cmd_send.wz = cmdstate_set.rotate_wz;
        }
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))      // 右侧开关[上]
    {
        if (switch_is_down(rc_data[TEMP].rc.switch_left))      // 左[下] 自瞄
        {
            HandleShoot(dial_event);
            HandleGimbalYaw(Gimbal_Remote);                    // 云台仍走 IMU
            HandleGimbalBPitch(dial_event, Gimbal_Remote, 0);
            HandleGimbalSPitch(0);
            HandleChassis(CHASSIS_NO_FOLLOW);                  // 底盘全向平移
        }
        else if (switch_is_mid(rc_data[TEMP].rc.switch_left))  // 左[中] 导航
        {
            ShootOff();
            HandleGimbalYaw(Gimbal_NAVI);                      // yaw: 编码器反馈,锁标定角
            HandleGimbalBPitch(dial_event, Gimbal_NAVI, 0);    // 大pitch: 按是否对准自动判定
            HandleGimbalSPitch(0);
            HandleChassis(CHASSIS_NAVIGATION);
        }
        else if (switch_is_up(rc_data[TEMP].rc.switch_left))   // 左[上] 竞赛
        {
            ShootOff();
            HandleGimbalYaw(Gimbal_Remote);
            HandleGimbalBPitch(dial_event, Gimbal_Remote, 0);
            HandleGimbalSPitch(0);
            HandleChassis(CHASSIS_COMPETITION);
        }
    }
    else                                                       // 右侧开关[下]:未定义组合
    {                                                          // 按"安全保持"处理
        ShootOff();
        HandleGimbalYaw(Gimbal_Remote);                        // 会做 IMU 重同步,不会甩头
        HandleGimbalBPitch(dial_event, Gimbal_Remote, 0);
        HandleGimbalSPitch(0);
        HandleChassis(CHASSIS_ZERO_FORCE);                     // 底盘停
    }
#endif
}

/* ============================== 急停 ============================== */
/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 */
static void EmergencyHandler(void)
{
    if (switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left))
    {   // 右+左侧开关状态[下]
        robot_state = ROBOT_STOP;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        YawRefSyncToIMU();                     // 记下当前朝向,恢复时不会甩头
    }
    else
    {
        robot_state = ROBOT_READY;
    }
}

/* ================== 机器人核心控制任务,200Hz运行 ================== */
void RobotCMDTask(void)
{
    static Robot_Status_e last_robot_state = ROBOT_READY;

    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 根据遥控器开关,确定当前使用的控制模式
    RemoteControlSet();
    // 处理模块离线和遥控器急停等紧急情况
    EmergencyHandler();

    if (last_robot_state == ROBOT_STOP && robot_state == ROBOT_READY)
    {
        // 急停恢复:模式没变,HandleGimbalYaw 里的 entering 不会触发,这里显式重同步一次
        YawRefSyncToIMU();
    }
    last_robot_state = robot_state;

    if (robot_state == ROBOT_STOP)
    {
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
#endif
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        shoot_cmd_send.shoot_mode   = SHOOT_OFF;
    }

    // 推送消息,双板通信,视觉通信等
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
