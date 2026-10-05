#include <stdarg.h>

#include "cmsis_os2.h"
//#include "bsp_log.h"
#include "bsp_tools.h"

#define MX_SIG_LIST_SIZE 32 // 不可修改,最大信号量数量
#define CBK_FLAG         0x01U

typedef struct
{
    void *callback;        // 回调函数指针 void f(void const *)
    void *ins;             // 传给回调的参数
    osThreadId_t thread_id; // 本回调对应的任务句柄
} CallbackTask_t;

// 递增记录全局信号量的数组,最大32个信号量
static uint8_t sig_idx = 0;

//static osThreadId_t cbkid_list[MX_SIG_LIST_SIZE];
static CallbackTask_t cbkinfo_list[MX_SIG_LIST_SIZE];

// 死循环任务:执行 cbk 函数,每次执行完阻塞等待触发标志
__attribute__((noreturn)) static void CallbackTaskBase(void *argument)
{
    CallbackTask_t *cbk = (CallbackTask_t *)argument;
    void (*cbk_func)(void const *) = (void (*)(void const *))cbk->callback;
    void const *ins = cbk->ins;

    for (;;)
    {
        cbk_func(ins);
        // 等待触发标志,收到后自动清除,永久阻塞
        osThreadFlagsWait(CBK_FLAG, osFlagsWaitAny, osWaitForever);
    }
}

osThreadId_t CreateCallbackTask(const char *name, void *cbk, void *ins, osPriority_t priority)
{
    // if (sig_idx >= MX_SIG_LIST_SIZE)
    //     while (1)
    //         LOGERROR("[rtos:cbk_register] CreateCallbackTask: sig_idx >= MX_SIG_LIST_SIZE");

    CallbackTask_t *info = &cbkinfo_list[sig_idx];
    info->callback = cbk;
    info->ins      = ins;

    const osThreadAttr_t attr = {
        .name       = name,
        .priority   = priority,
        .stack_size = 512,   // ★ v2 单位是字节,原来 128 word = 512 字节,等价
    };

    info->thread_id = osThreadNew(CallbackTaskBase, info, &attr);
//    cbkid_list[sig_idx] = info->thread_id;

    sig_idx++;
    return info->thread_id;   // ★ 返回任务句柄,不再是 sig
}
