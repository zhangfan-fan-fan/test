/**
 * @file dvc_motor.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 串口绘图
 * @version 0.1
 * @date 2022-08-07
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DVC_SERIALPLOT_H
#define DVC_SERIALPLOT_H

/* Includes ------------------------------------------------------------------*/

#include <stdarg.h>
#include <string.h>
#include <math.h>
#include "__drv_uart.h"

/* Exported macros -----------------------------------------------------------*/

//串口绘图单条指令最大长度
#define SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH (100)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 串口绘图传输数据类型
 * 
 */
enum Enum_Serialplot_Data_Type
{
    Serialplot_Data_Type_UINT8 = 0,//枚举下面的自动递增
    Serialplot_Data_Type_UINT16,
    Serialplot_Data_Type_UINT32,
    Serialplot_Data_Type_INT8,
    Serialplot_Data_Type_INT16,
    Serialplot_Data_Type_INT32,
    Serialplot_Data_Type_FLOAT,
    Serialplot_Data_Type_DOUBLE,
};

/**
 * @brief 串口绘图工具, 最多支持12个通道
 *
 */
class Class_Serialplot
{
public://公共的信息外部随便调
    void Init(UART_HandleTypeDef *__huart, uint8_t __Serialplot_Rx_Variable_Assignment_Num, char **__Serialplot_Rx_Variable_Assignment_List, Enum_Serialplot_Data_Type __Serialplot_Data_Type = Serialplot_Data_Type_FLOAT, uint8_t __Frame_Header = 0xAB);
    //绑串口、登记字典Variable变量/参数，Assignmen赋值，Num数量（电脑可以通过串口绘图发多少个“变量名=值”的命令）、选数据类型、设帧头
    //这是用来接收的时候改pid用的，比如电脑发来kp=1.6，kp是字典中的变量名，1.6是值
    //_Serialplot_Rx_Variable_Assignment_Num“字典列表”，里面放的是允许被赋值的变量名__Serialplot_Rx_Variable_Assignment_List就是字典列表
    int8_t Get_Variable_Index();//刚才那条指令匹配到字典第几号？
    double Get_Variable_Value();//指令里的数值是多少？

    void Set_Data(uint8_t Number, ...);//登记要画曲线的变量地址

    void TIM_Add_PeriodElapsedCallback();//把数据装进发货架
    uint16_t Get_Tx_Length(void);// 本帧该发多少字节 = 4×通道数 + 4

    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);//收到指令时调用：解析名字(字典)=数值，Length 是本帧真实字节数

protected://私有的子类可继承
    //初始化相关常量

    //绑定的UART编号
    UART_HandleTypeDef *huart;
    //发送缓冲区
    uint8_t *UART_Tx_Data;//发货架
    //接收缓冲区
    uint8_t *UART_Rx_Data;//收货架
    
    
    //接收指令字典的数量    
    uint8_t UART_Rx_Variable_Num;
    //接收指令字典列表指针
    char **UART_Rx_Variable_List;
    //串口绘图数据类型
    Enum_Serialplot_Data_Type UART_Tx_Data_Type;

    //常量

    //内部变量

    //需要绘图的各个变量数据地址；12 个通道的变量地址数组
    const void *Data[12];//const表明只读值，不写
    //当前发送的数据长度, 等价于新数据偏移量
    uint8_t Data_Number = 0;
    //当前接收的指令在指令字典中的编号
    int8_t Variable_Index = 0;//对应上边的int8_t Get_Variable_Index()
    //当前接收的指令在指令字典中的值
    double Variable_Value = 0;
    //本帧真实字节数, 解析的边界（DMA 空闲中断给的长度）
    uint16_t UART_Rx_Length = 0;
    //DMA 发送连续忙的帧数, 超过 10 帧(100ms)就 abort 一次, 防止状态机卡死
    uint16_t UART_Tx_Busy_Count = 0;

    //读变量

    //写变量

    //读写变量

    //内部函数
    //外部代码不许直接调，只能被本类的公开函数内部调用
    uint8_t Judge_Variable_Name();//从电脑发来的字符串里取出变量名
    void Judge_Variable_Value(int flag);//把数字解析出来
    void Output();//	把要画的变量打包成 49 字节
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
