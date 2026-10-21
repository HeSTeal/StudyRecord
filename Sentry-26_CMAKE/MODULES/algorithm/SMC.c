#include "SMC.h"
#include "math.h"
SMC Chassis_Supper[4];

/**
 * @brief 符号函数，返回s的符号（1, 0, -1）
 * @param s 输入值
 * @return float 符号值
 */
float SMC_Signal(float s)
{
    if (s > 0.0f)
        return 1.0f;
    else if (s == 0.0f)
        return 0.0f;
    else
        return -1.0f;
}

/**
 * @brief 饱和函数，限制滑模面s的输出范围，消除抖振
 * @param smc 滑模控制器结构体指针
 * @param s 滑模面变量
 * @return float 饱和处理后的值
 */
float SMC_Sat(SMC *smc, float s)
{
    float y;
    y = s / smc->param.epsilon;
    if (fabsf(y) <= smc->limit)
        return y;
    else
        return SMC_Signal(y);
}
/***************************SMC param initialize******************************/
/**
 * @brief 内部参数初始化函数，设置滑模控制器参数和状态变量。
 * @param smc 滑模控制器结构体指针
 * @param C 线性滑模面系数 c
 * @param K 趋近率增益 K
 * @param error_eps 位置误差死区阈值
 * @param u_max 输出限幅 从
 * @param J 系统惯量
 * @param epsilon 抖振抑制参数
 * @param flag 控制模式
 * @param c1 EISMC参数 c1
 * @param c2 EISMC参数 c2
 * @param p TFSMC参数 p
 * @param q TFSMC参数 q
 * @param beta TFSMC参数 beta
 */
static void SMC_param_init(
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
    float beta)
{
    smc->param.c = C;
    smc->param.K = K;
    smc->error.position_error_threshold = error_eps;
    smc->u_max = u_max;
    smc->param.J = J;
    smc->param.epsilon = epsilon;
    smc->flag = flag;
    smc->limit = 1.0f;

    smc->param.c1 = c1;
    smc->param.c2 = c2;
    smc->param.p = p;
    smc->param.q = q;
    smc->param.beta = beta;

    smc->error.target_current = 0.0f;
    smc->error.target_previous = 0.0f;
    smc->error.position_error = 0.0f;
    smc->error.target_derivative = 0.0f;
    smc->error.target_second_derivative = 0.0f;
    smc->s = 0.0f;
    smc->u = 0.0f;
    smc->error.position_current = 0.0f;
    smc->error.velocity_current = 0.0f;

    smc->error.target_derivative_prev = 0.0f;
    smc->error.velocity_error = 0.0f;
    smc->error.position_error_integral = 0.0f;
    smc->error.velocity_error_integral = 0.0f;
    smc->error.velocity_error_threshold = 0.0f;
    smc->error.position_error_last = 0.0f;

    smc->param_last = smc->param;
}
/***************************SMC initialize******************************/
/**
 * @brief 初始化滑模控制器参数和误差结构体，并设置所有参数和模式
 * @param smc 滑模控制器结构体指针
 * @param C 线性滑模面系数 c
 * @param K 趋近率增益 K
 * @param error_eps 位置误差死区阈值
 * @param u_max 输出限幅
 * @param J 系统惯量
 * @param epsilon 抖振抑制参数
 * @param flag 控制模式
 * @param c1 EISMC参数 c1
 * @param c2 EISMC参数 c2
 * @param p TFSMC参数 p
 * @param q TFSMC参数 q
 * @param beta TFSMC参数 beta
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
    float beta)
{
    // 调用内部初始化函数
    SMC_param_init(smc, C, K, error_eps, u_max, J, epsilon, flag, c1, c2, p, q, beta);
}



/**
 * @brief 位置环误差更新，计算目标与实际位置、速度的误差
 * @param smc 滑模控制器结构体指针
 * @param ref 目标位置
 * @param pos_now 当前实际位置
 * @param vol_now 当前实际速度
 */
void SMC_ErrorUpdate_Pos(SMC *smc, float ref, float pos_now, float vol_now)
{
    
         /***********过零处理***********************************/
    float deviation = ref - pos_now;
    
    // 假设角度传感器周期为8191（与PID代码一致）
    const float ANGLE_MAX = 6.28f;
    
    // 如果偏差超过半周期，调整当前位置到合适周期
    if(deviation > 3.14)
       pos_now+= ANGLE_MAX;
    else if(deviation < -3.14)
        pos_now -= ANGLE_MAX;
    /**************************************************/
    smc->error.target_current = ref;
    smc->error.target_derivative = (smc->error.target_current - smc->error.target_previous) / dt_SMC;
    smc->error.target_second_derivative = (smc->error.target_derivative - smc->error.target_derivative_prev) / dt_SMC;
    smc->error.position_current = pos_now;
    smc->error.velocity_current = vol_now;
    smc->error.position_error = pos_now - ref;
    smc->error.velocity_error = vol_now - smc->error.target_derivative;
    smc->error.target_previous = smc->error.target_current;
    smc->error.position_error_integral += smc->error.position_error * dt_SMC;
    smc->error.target_derivative_prev = smc->error.target_derivative;
}
/**
 * @brief 位置环误差更新，计算目标与实际位置、速度的误差
 * @param smc 滑模控制器结构体指针
 * @param ref 目标位置
 * @param pos_now 当前实际位置
 * @param vol_now 当前实际速度
 */
void SMC_ErrorUpdate_Pos_MSC(SMC *smc, float ref, float pos_now, float vol_now)
{
    /***********过零处理***********************************/
    float deviation = ref - pos_now;
   
    // 如果偏差超过半周期，调整当前位置到合适周期
    if(deviation>180)
		ref+=360;
	else if(deviation<-180)
		ref-=360;	
    /**************************************************/
    smc->error.target_current = ref;
    smc->error.target_derivative = (smc->error.target_current - smc->error.target_previous) / dt_SMC;
    smc->error.target_second_derivative = (smc->error.target_derivative - smc->error.target_derivative_prev) / dt_SMC;
    smc->error.position_current = pos_now;
    smc->error.velocity_current = vol_now;
    smc->error.position_error = pos_now - ref;
    smc->error.velocity_error = vol_now - smc->error.target_derivative;
    smc->error.target_previous = smc->error.target_current;
    smc->error.position_error_integral += smc->error.position_error * dt_SMC;
    smc->error.target_derivative_prev = smc->error.target_derivative;
}

/**
 * @brief 速度环误差更新，计算目标与实际速度的误差
 * @param smc 滑模控制器结构体指针
 * @param ref 目标速度
 * @param vol_now 当前实际速度
 */
void SMC_ErrorUpdate_Vel(SMC *smc, float ref, float vol_now)
{


    smc->error.target_current = ref;
    smc->error.target_derivative = (smc->error.target_current - smc->error.target_previous) / dt_SMC;
    smc->error.velocity_current = vol_now;
    smc->error.velocity_error = vol_now - smc->error.target_current;
    smc->error.velocity_error_integral += smc->error.velocity_error * dt_SMC;
    if (fabsf(smc->error.velocity_error_integral) > velocity_integral_max)
    {
        smc->error.velocity_error_integral = velocity_integral_max * SMC_Signal(smc->error.velocity_error_integral);
    }
    smc->error.target_previous = smc->error.target_current;
}

/**
 * @brief 清除所有误差和积分项，重置控制器状态
 * @param smc 滑模控制器结构体指针
 */
void SMC_Clear(SMC *smc)
{
    smc->error.target_current = 0.0f;
    smc->error.target_previous = 0.0f;
    smc->error.target_derivative = 0.0f;
    smc->error.position_error = 0.0f;
    smc->error.velocity_error = 0.0f;
    smc->error.velocity_error_integral = 0.0f;
    smc->error.position_error_threshold = 0.0f;
    smc->error.velocity_error_threshold = 0.0f;
    smc->error.position_current = 0.0f;
    smc->error.velocity_current = 0.0f;
    smc->error.target_second_derivative = 0.0f;
    smc->error.target_derivative_prev = 0.0f;
    smc->error.position_error_integral = 0.0f;
}
/**
 * @brief 清除积分项（位置和速度误差积分）
 * @param smc 滑模控制器结构体指针
 */
void SMC_Integval_Clear(SMC *smc)
{
    smc->error.velocity_error_integral = 0.0f;
    smc->error.position_error_integral = 0.0f;
}

/**
 * @brief 滑模控制器主计算函数，根据不同模式计算控制量
 * @param smc 滑模控制器结构体指针
 * @return float 控制器输出
 */
float SMC_SmcCalculate(SMC *smc)
{
    float u = 0.0f;
    float fun;
    static float pos_pow;
    switch (smc->flag)
    {
    case EXPONENT: /// 线性滑模面，指数趋近律

        if (fabsf(smc->error.position_error) - smc->error.position_error_threshold < 0.0f)
        {
            smc->error.position_error = 0.0f;
            return 0.0f;
        }

        // 滑模面 S = c*e + dot_e
        smc->s = smc->param.c * smc->error.position_error + smc->error.velocity_error;
        fun = SMC_Sat(smc, smc->s); // 饱和函数

        // 控制器计算 u = J * (u_eq + u_sw)
        // error.tar_differential_second 二阶导数项可能会导致抖震
        // u = smc->param.J * ((-smc->param.c * smc->error.v_error) - smc->param.K * smc->s - smc->param.epsilon * fun + smc->error.tar_differential_second);
        u = smc->param.J * ((-smc->param.c * smc->error.velocity_error) - smc->param.K * smc->s - smc->param.epsilon * fun);
        break;

    case POWER: /// 线性滑模面，幂次趋近律

        if (fabsf(smc->error.position_error) - smc->error.position_error_threshold < 0.0f)
        {
            smc->error.position_error = 0.0f;
            return 0.0f;
        }

        // 滑模面 S = c*e + dot_e
        smc->s = smc->param.c * smc->error.position_error + smc->error.velocity_error;
        fun = SMC_Sat(smc, smc->s); // 饱和函数

        // 控制器计算 u = J * (u_eq + u_sw)
        // u_sw 包含幂次项: -K * |s|^epsilon * sgn(s) (fun 仅为 sgn(s) 或线性项)
        u = smc->param.J * ((-smc->param.c * smc->error.velocity_error) - smc->param.K * smc->s - smc->param.K * (powf(fabsf(smc->s), smc->param.epsilon)) * fun);
        break;

    case TFSMC: /// 终端滑模控制 (Terminal Sliding Mode Control)

        // 误差死区处理
        if (fabsf(smc->error.position_error) - smc->error.position_error_threshold < 0.0f)
        {
            smc->error.position_error = 0.0f;
            return 0.0f;
        }

        // 幂次项：|e|^(q/p) * sgn(e)
        pos_pow = powf(fabsf(smc->error.position_error), smc->param.q / smc->param.p);
        if (smc->error.position_error <= 0)
            pos_pow = -pos_pow;

        // 滑模面 S = beta * |e|^(q/p) * sgn(e) + dot_e
        smc->s = smc->param.beta * pos_pow + smc->error.velocity_error;

        fun = SMC_Sat(smc, smc->s);

        if (smc->error.position_error != 0.0f)
        {
            // u = J * (ddot_r - K*s - epsilon*SAT(S) - dot_e * (beta * (q/p) * |e|^(q/p - 1) * sgn(e)))
            u = smc->param.J * (smc->error.target_second_derivative - smc->param.K * smc->s - smc->param.epsilon * fun - smc->error.velocity_error * ((smc->param.q * smc->param.beta) * pos_pow) / (smc->param.p * smc->error.position_error));
        }
        else
            u = 0.0f;
        break;

    case VELSMC: /// 比例积分速度滑模控制，指数趋近率，速度控制
        // 滑模面 S = v_error + c * Integral(v_error)
        smc->s = smc->error.velocity_error + smc->param.c * smc->error.velocity_error_integral;
        fun = SMC_Sat(smc, smc->s);

        // u = J * (ddot_r + dot_r - c*v_error - K*s - epsilon*SAT(S))

        // u = smc->param.J * (smc->error.tar_differential_second + smc->error.tar_differential - (smc->param.c * smc->error.v_error) - smc->param.K * smc->s - smc->param.epsilon * fun);
        u = smc->param.J * (smc->error.target_derivative - (smc->param.c * smc->error.velocity_error) - smc->param.K * smc->s - smc->param.epsilon * fun);
        break;

    case EISMC: /// 比例积分位置滑模控制 (Extended Integral SMC)，指数趋近率，位置控制
        // 误差死区处理
        // if (fabsf(smc->error.position_error) - smc->error.position_error_threshold < 0.0f)
        // {
        //     smc->error.position_error = 0.0f;
        //     return 0.0f;
        // }

        // 滑模面 S = c1*e + dot_e + c2*Integral(e)
        smc->s = smc->param.c1 * smc->error.position_error + smc->error.velocity_error + smc->param.c2 * smc->error.position_error_integral;
        fun = SMC_Sat(smc, smc->s);

        // u = J * (u_eq + u_sw)
        // u_eq 对应 ddot_r - c1*dot_e - c2*e
        u = smc->param.J * ((-smc->param.c1 * smc->error.velocity_error) // -c1*dot_e
                            - smc->param.c2 * smc->error.position_error  // -c2*e
                            - smc->param.K * smc->s - smc->param.epsilon * fun);
        break;
    }

    smc->error.position_error_last = smc->error.position_error; // 更新上一步的误差

    // 控制量限幅
    if (u > smc->u_max)
    {
        u = smc->u_max;
    }
    else if (u < -smc->u_max)
    {
        u = -smc->u_max;
    }
    smc->u = u;
    return u;
}

#define FOLLOW_SMC_C       11.0f     // 滑模面斜率
#define FOLLOW_SMC_K       2.5f     // 趋近律增益
#define FOLLOW_SMC_PHI     0.15f    // 饱和边界层，防抖
#define FOLLOW_SMC_KI      0.0f     // 先给 0，有静差再加
#define FOLLOW_SMC_I_MAX   2.0f
#define FOLLOW_SMC_WZ_MAX  6.28f
#define FOLLOW_SMC_DT      0.001f   // 改成 ChassisTask 实际周期
#define FOLLOW_SMC_D	   1.2f     // 阻尼增益，先给 0.6f，后续可调
static float follow_smc_i = 0.0f;

static float WrapPi(float x)
{
    while (x > PI)  x -= 2.0f * PI;
    while (x < -PI) x += 2.0f * PI;
    return x;
}
float ChassisFollowSMC(float offset_angle,float target_offset,float real_wz)
{
    float e = WrapPi(offset_angle - target_offset);
    // 可选积分，先给 0
    follow_smc_i += e * FOLLOW_SMC_DT;
    if (follow_smc_i > FOLLOW_SMC_I_MAX)  follow_smc_i = FOLLOW_SMC_I_MAX;
    if (follow_smc_i < -FOLLOW_SMC_I_MAX) follow_smc_i = -FOLLOW_SMC_I_MAX;
    // 一阶滑模面：s = e + ki*∫e
    float s = e + FOLLOW_SMC_KI * follow_smc_i;
    // 饱和函数 sat(s/phi)
    float sat = s / FOLLOW_SMC_PHI;
    if (sat > 1.0f)  sat = 1.0f;
    if (sat < -1.0f) sat = -1.0f;
    // 控制律：u = C*e + KI*∫e + K*sat(s/phi)
    float vw = FOLLOW_SMC_C * e+ FOLLOW_SMC_KI * follow_smc_i+ FOLLOW_SMC_K * sat- FOLLOW_SMC_D * real_wz;   
    if (vw > FOLLOW_SMC_WZ_MAX)  vw = FOLLOW_SMC_WZ_MAX;
    if (vw < -FOLLOW_SMC_WZ_MAX) vw = -FOLLOW_SMC_WZ_MAX;
    return vw;
}
