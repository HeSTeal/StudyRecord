#include "bsp_RS485.h"
#include "usart.h"

uint8_t TX_Data_1[3]={1,1,1},TX_Data_2[3]={2,2,2};

void RS485_1_rx_Enable(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
}

void RS485_1_tx_Enable(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}

void RS485_2_rx_Enable(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
}

void RS485_2_tx_Enable(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
}

void RS485_1_TX(void)
{
    HAL_UART_Transmit_DMA(&huart1, TX_Data_1 ,3u);
}

void RS485_2_TX(void)
{
    HAL_UART_Transmit_DMA(&huart6, TX_Data_2 ,3u);
}


