/**
  ******************************************************************************
  * @file	 controller.h
  * @author  Wang Hongxi
  * @author  Zhang Hongyu (fuzzy pid)
  * @version V1.1.3
  * @date    2021/7/3
  * @brief   
  ******************************************************************************
  * @attention 
  *
  ******************************************************************************
  */
#ifndef _CONTROLLER_H
#define _CONTROLLER_H


#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "user_lib.h"
#include "arm_math.h"
#include <math.h>

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

/******************************** FUZZY PID **********************************/
#define NB -3
#define NM -2
#define NS -1
#define ZE 0
#define PS 1
#define PM 2
#define PB 3

typedef __packed struct
{
    float KpFuzzy;
    float KiFuzzy;
    float KdFuzzy;

    float (*FuzzyRuleKp)[7];
    float (*FuzzyRuleKi)[7];
    float (*FuzzyRuleKd)[7];

    float KpRatio;
    float KiRatio;
    float KdRatio;

    float eStep;
    float ecStep;

    float e;
    float ec;
    float eLast;

    uint32_t DWT_CNT;
    float dt;
} FuzzyRule_t;

void Fuzzy_Rule_Init(FuzzyRule_t *fuzzyRule, float (*fuzzyRuleKp)[7], float (*fuzzyRuleKi)[7], float (*fuzzyRuleKd)[7],
                     float kpRatio, float kiRatio, float kdRatio,
                     float eStep, float ecStep);
void Fuzzy_Rule_Implementation(FuzzyRule_t *fuzzyRule, float measure, float ref);

/******************************* PID CONTROL *********************************/
// PID 优化环节使能标志位,通过位与可以判断启用的优化环节;也可以改成位域的形式
typedef enum
{
    PID_IMPROVE_NONE               = 0x00,  // 0b00000000
		PID_Integral_Limit             = 0x01,  // 0b00000001
		PID_Derivative_On_Measurement  = 0x02,  // 0b00000010
		PID_Trapezoid_Intergral        = 0x04,  // 0b00000100
		PID_Proportional_On_Measurement= 0x08,  // 0b00001000
		PID_OutputFilter               = 0x10,  // 0b00010000
		PID_ChangingIntegrationRate    = 0x20,  // 0b00100000
		PID_DerivativeFilter           = 0x40,  // 0b01000000
		PID_ErrorHandle                = 0x80   // 0b10000000
} PID_Improvement_e;

/* PID 报错类型枚举*/
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U
} ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

/* PID结构体 */
typedef struct
{
    //---------------------------------- init config block
    // config parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit;     // 积分限幅
    float CoefA;             // 变速积分 For Changing Integral
    float CoefB;             // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
    float Derivative_LPF_RC; // 微分滤波器系数

    //-----------------------------------
    // for calculating
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    uint32_t DWT_CNT;
    float dt;

    PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

/* 用于PID初始化的结构体*/
typedef struct // config parameter
{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // 输出限幅
    float DeadBand; // 死区

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit; // 积分限幅
    float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;
} PID_Init_Config_s;

/**
 * @brief 初始化PID实例
 * @todo 待修改为统一的PIDRegister风格
 * @param pid    PID实例指针
 * @param config PID初始化配置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);

/**
 * @brief 计算PID输出
 *
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param ref     设定值
 * @return float  PID计算输出
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);
/*************************** FEEDFORWARD CONTROL *****************************/
typedef __packed struct
{
    float c[3]; // G(s) = 1/(c2s^2 + c1s + c0)

    float Ref;
    float Last_Ref;

    float DeadBand;

    uint32_t DWT_CNT;
    float dt;

    float LPF_RC; // RC = 1/omegac

    float Ref_dot;
    float Ref_ddot;
    float Last_Ref_dot;

    uint16_t Ref_dot_OLS_Order;
    Ordinary_Least_Squares_t Ref_dot_OLS;
    uint16_t Ref_ddot_OLS_Order;
    Ordinary_Least_Squares_t Ref_ddot_OLS;

    float Output;
    float MaxOut;

} Feedforward_t;

void Feedforward_Init(
    Feedforward_t *ffc,
    float max_out,
    float *c,
    float lpf_rc,
    uint16_t ref_dot_ols_order,
    uint16_t ref_ddot_ols_order);

float Feedforward_Calculate(Feedforward_t *ffc, float ref);

/************************* LINEAR DISTURBANCE OBSERVER *************************/
typedef __packed struct
{
    float c[3]; // G(s) = 1/(c2s^2 + c1s + c0)

    float Measure;
    float Last_Measure;

    float u; // system input

    float DeadBand;

    uint32_t DWT_CNT;
    float dt;

    float LPF_RC; // RC = 1/omegac

    float Measure_dot;
    float Measure_ddot;
    float Last_Measure_dot;

    uint16_t Measure_dot_OLS_Order;
    Ordinary_Least_Squares_t Measure_dot_OLS;
    uint16_t Measure_ddot_OLS_Order;
    Ordinary_Least_Squares_t Measure_ddot_OLS;

    float Disturbance;
    float Output;
    float Last_Disturbance;
    float Max_Disturbance;
} LDOB_t;

void LDOB_Init(
    LDOB_t *ldob,
    float max_d,
    float deadband,
    float *c,
    float lpf_rc,
    uint16_t measure_dot_ols_order,
    uint16_t measure_ddot_ols_order);

float LDOB_Calculate(LDOB_t *ldob, float measure, float u);

/*************************** Tracking Differentiator ***************************/
typedef __packed struct
{
    float Input;

    float h0;
    float r;

    float x;
    float dx;
    float ddx;

    float last_dx;
    float last_ddx;

    uint32_t DWT_CNT;
    float dt;
} TD_t;

void TD_Init(TD_t *td, float r, float h0);
float TD_Calculate(TD_t *td, float input);

//// PID 优化环节使能标志位,通过位与可以判断启用的优化环节;也可以改成位域的形式
//typedef enum
//{
//    PID_IMPROVE_NONE = 0b00000000,                // 0000 0000
//    PID_Integral_Limit = 0b00000001,              // 0000 0001
//    PID_Derivative_On_Measurement = 0b00000010,   // 0000 0010
//    PID_Trapezoid_Intergral = 0b00000100,         // 0000 0100
//    PID_Proportional_On_Measurement = 0b00001000, // 0000 1000
//    PID_OutputFilter = 0b00010000,                // 0001 0000
//    PID_ChangingIntegrationRate = 0b00100000,     // 0010 0000
//    PID_DerivativeFilter = 0b01000000,            // 0100 0000
//    PID_ErrorHandle = 0b10000000,                 // 1000 0000
//} PID_Improvement_e;

///* PID 报错类型枚举*/
//typedef enum errorType_e
//{
//    PID_ERROR_NONE = 0x00U,
//    PID_MOTOR_BLOCKED_ERROR = 0x01U
//} ErrorType_e;

//typedef struct
//{
//    uint64_t ERRORCount;
//    ErrorType_e ERRORType;
//} PID_ErrorHandler_t;

///* PID结构体 */
//typedef struct
//{
//    //---------------------------------- init config block
//    // config parameter
//    float Kp;
//    float Ki;
//    float Kd;
//    float MaxOut;
//    float DeadBand;

//    // improve parameter
//    PID_Improvement_e Improve;
//    float IntegralLimit;     // 积分限幅
//    float CoefA;             // 变速积分 For Changing Integral
//    float CoefB;             // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
//    float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
//    float Derivative_LPF_RC; // 微分滤波器系数

//    //-----------------------------------
//    // for calculating
//    float Measure;
//    float Last_Measure;
//    float Err;
//    float Last_Err;
//    float Last_ITerm;

//    float Pout;
//    float Iout;
//    float Dout;
//    float ITerm;

//    float Output;
//    float Last_Output;
//    float Last_Dout;

//    float Ref;

//    uint32_t DWT_CNT;
//    float dt;

//    PID_ErrorHandler_t ERRORHandler;
//} PIDInstance;

///* 用于PID初始化的结构体*/
//typedef struct // config parameter
//{
//    // basic parameter
//    float Kp;
//    float Ki;
//    float Kd;
//    float MaxOut;   // 输出限幅
//    float DeadBand; // 死区

//    // improve parameter
//    PID_Improvement_e Improve;
//    float IntegralLimit; // 积分限幅
//    float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
//    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
//    float Output_LPF_RC; // RC = 1/omegac
//    float Derivative_LPF_RC;
//} PID_Init_Config_s;

///**
// * @brief 初始化PID实例
// * @todo 待修改为统一的PIDRegister风格
// * @param pid    PID实例指针
// * @param config PID初始化配置
// */
//void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);

///**
// * @brief 计算PID输出
// *
// * @param pid     PID实例指针
// * @param measure 反馈值
// * @param ref     设定值
// * @return float  PID计算输出
// */
//float PIDCalculate(PIDInstance *pid, float measure, float ref);

#endif
