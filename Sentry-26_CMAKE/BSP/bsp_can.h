#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "main.h"
#include "can.h"

#define CAN_MX_REGISTER_CNT 24
#define MX_CAN_FILTER_CNT (2*14)
#define DEVICE_CAN_CNT 2
#pragma pack(1)

typedef struct _
{
    CAN_HandleTypeDef *can_handle;
    CAN_TxHeaderTypeDef txconf;
    uint32_t tx_id;
		uint32_t tx_mailbox;
    uint8_t tx_buff[8];           
    uint8_t rx_buff[8];            
    uint32_t rx_id;              
    uint8_t rx_len;
    void (*can_module_callback)(struct _ *);
    void *id;
}CANInstance;

#pragma pack()
/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
    CAN_HandleTypeDef *can_handle;              // can句柄
    uint32_t tx_id;                             // 发送id
    uint32_t rx_id;                             // 接收id
    void (*can_module_callback)(CANInstance *); // 处理接收数据的回调函数
    void *id;                                   // 拥有can实例的模块地址,用于区分不同的模块(如果有需要的话),如果不需要可以不传入
} CAN_Init_Config_s;

CANInstance *CANRegister(CAN_Init_Config_s *config);
void CANSetDLC(CANInstance *_instance, uint8_t length);
uint8_t CANTransmit(CANInstance *_instance,float timeout);

#endif
