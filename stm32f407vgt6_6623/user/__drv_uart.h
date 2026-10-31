/**
 * @file drv_uart.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的UART通信初始化与配置流程
 * @version 0.1
 * @date 2022-08-05
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DRV_UART_H
#define DRV_UART_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART通信接收回调函数数据类型
 *
 */
typedef void (*UART_Call_Back)(uint8_t *Buffer, uint16_t Length);//Buffer:数据放在哪，length：数据多长

/**
 * @brief UART通信处理结构体
 */


 #pragma pack(push, 1)          // 不许塞填充字节
typedef struct Struct_Telemetry
{
    uint32_t timestamp_ms;      //4位整数存 HAL_GetTick()
    uint8_t  mode;             // System_Status四个身份
    float    target_angle;      //目标角度
    float    actual_angle;      //实际角度
    float    error;             //误差
    float    output;            //输出
    int16_t  current;          // D2 实测后填真的
    int16_t  temperature;       //锁机与状态机的出口
    uint8_t  fault;            // 故障码
} Struct_Telemetry;
#pragma pack(pop)


typedef struct Struct_UART_Manage_Object//收纳盒
{
    UART_HandleTypeDef *UART_Handler;//哪个
    uint8_t *Rx_Buffer;//地址
    uint16_t Rx_Buffer_Size;//大小
    UART_Call_Back Callback_Function;//回调
} Struct_UART_Manage_Object;

/* Exported variables --------------------------------------------------------*/

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern Struct_UART_Manage_Object UART1_Manage_Object;
extern Struct_UART_Manage_Object UART2_Manage_Object;
extern Struct_UART_Manage_Object UART3_Manage_Object;



extern uint8_t UART1_Tx_Data[];//发送缓冲货架
extern uint8_t UART2_Tx_Data[];
extern uint8_t UART3_Tx_Data[];


/* Exported function declarations --------------------------------------------*/

void Uart_Init(UART_HandleTypeDef *huart, uint8_t *Rx_Buffer, uint16_t Rx_Buffer_Size, UART_Call_Back Callback_Function);

uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length);//返回状态码

void TIM_UART_PeriodElapsedCallback();

uint8_t CRC8(const uint8_t *data, uint8_t len) ;//算遥测帧的指纹

#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
