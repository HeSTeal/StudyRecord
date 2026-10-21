#ifndef SMC_H
#define SMC_H

#include "main.h"
#include "arm_math.h"
#define dt_SMC 0.001f
#define velocity_integral_max 2000.0f
#define position_integral_max 2000.0f
typedef enum
{
    EXPONENT, // 指数趋近律
    POWER,    // 幂次趋近律
    TFSMC,    // 终端滑模控制
    VELSMC,   // 比例积分速度滑模控制
    EISMC     // 比例积分位置滑模控制
} Rmode;

typedef struct
{
    float target_current;          // 当前目标值
    float target_previous;         // 上一次目标值
    float target_derivative;       // 目标值一阶导
    float target_derivative_prev;  // 上一次目标值一阶导
    float target_second_derivative;// 目标值二阶导

    float position_current;        // 当前位置
    float velocity_current;        // 当前速度

    float position_error;          // 位置误差 (pos_now - ref)
    float velocity_error;          // 速度误差 (vol_now - tar_differential)

    float position_error_integral; // 位置误差积分
    float velocity_error_integral; // 速度误差积分

    float position_error_threshold;// 位置误差死区阈值
    float velocity_error_threshold;// 速度误差死区阈值
    float position_error_last;     // 上一步的位置误差
} SError;

// 滑模参数结构体
typedef struct
{
    float J; // 惯量估计值
    float K; // 趋近率增益 K
    float c; // 线性滑模面系数 c

    float c1; // EISMC参数
    float c2; // EISMC参数

    float p;       // TFSMC参数，正奇数 p>q
    float q;       // TFSMC参数，正奇数
    float beta;    // TFSMC参数
    float epsilon; // ε 趋近率或饱和函数相关参数
} SParam;

typedef struct
{
    float u; // 控制器输出
    float s; // 滑模面计算储存

    SParam param;      // 当前参数
    SParam param_last; // 上一次参数

    SError error; // 误差状态
    float u_max;  // 输出限幅
    Rmode flag;   // 趋近律模式
    float limit;  // 饱和函数限幅值
} SMC;

/**
 * @brief 初始化滑模控制器参数和误差结构体，并设置所有参数和模式
 */
void SMC_Init(
    SMC *smc,
    float C,
    float K,
    float error_eps,
    float u_max,
    float J,
    float epsilon,
    Rmode flag,
    float c1,
    float c2,
    float p,
    float q,
    float beta);

// static void SMC_param_init(
//     SMC *smc,
//     float C,
//     float K,
//     float error_eps,
//     float u_max,
//     float J,
//     float epsilon,
//     Rmode flag,
//     float c1,
//     float c2,
//     float p,
//     float q,
//     float beta);

float SMC_Signal(float s);
float SMC_Sat(SMC *smc, float s);

void SMC_ErrorUpdate_Pos(SMC *smc, float ref, float pos_now, float vol_now); // 位置环误差更新
void SMC_ErrorUpdate_Vel(SMC *smc, float ref, float vol_now);                // 速度环误差更新
void SMC_ErrorUpdate_Pos_MSC(SMC *smc, float ref, float pos_now, float vol_now);
    
float SMC_SmcCalculate(SMC *smc);
void SMC_Clear(SMC *smc);
void SMC_Integval_Clear(SMC *smc);

void SMC_OutContinuation(SMC *smc);

float ChassisFollowSMC(float offset_angle,float target_offset,float real_wz);

extern SMC Chassis_Supper[4];
	
#endif // SLIDING_H
