/**
 * @file dvc_motor.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 串口绘图
 * @version 0.1
 * @date 2022-08-07
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include <__dvc_serialplot.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
// 注：这里原本有一行 Enum_System_Status System_Status = System_Status_IDLE;
// 是从 dvc_motor.cpp 复制文件头时带过来的残留。
// System_Status 属于电机模块（定义在 __dvc_motor 里），串口绘图用不到它，
// 而本文件没有 include __dvc_motor.h，留着会报 unknown type name，故删除。

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 串口绘图初始化
 *
 * @param __huart 指定的UART
 * @param __Serialplot_Rx_Variable_Assignment_Num 接收指令字典的数量
 * @param __Serialplot_Rx_Variable_Assignment_List 接收指令字典列表
 * @param __Serialplot_Tx_Data_Type 传输数据类型, 默认float
 * @param __Frame_Header 帧头标识符
 */
void Class_Serialplot::Init(UART_HandleTypeDef *__huart, uint8_t __Serialplot_Rx_Variable_Assignment_Num, char **__Serialplot_Rx_Variable_Assignment_List, Enum_Serialplot_Data_Type __Serialplot_Tx_Data_Type, uint8_t __Frame_Header)
{
    //双冒号叫“作用域解析运算符”，读作“……的”：告诉编译器这个函数是 Class_Serialplot 的成员，不是野函数。.cpp 里每个成员函数定义都带这个前缀，靠它把实现和类捆在一起
    huart = __huart;//因为类中的成员是huart所以变量要换名字
    if (huart->Instance == USART1)
    {
        UART_Rx_Data = UART1_Manage_Object.Rx_Buffer;//从收货架取
        UART_Tx_Data = UART1_Tx_Data;//发货的架子
    }
    else if (huart->Instance == USART2)
    {
        UART_Rx_Data = UART2_Manage_Object.Rx_Buffer;
        UART_Tx_Data = UART2_Tx_Data;
    }
    else if (huart->Instance == USART3)
    {
        UART_Rx_Data = UART3_Manage_Object.Rx_Buffer;
        UART_Tx_Data = UART3_Tx_Data;
    }
   
    UART_Rx_Variable_Num = __Serialplot_Rx_Variable_Assignment_Num;
    UART_Rx_Variable_List = __Serialplot_Rx_Variable_Assignment_List;
    UART_Tx_Data_Type = __Serialplot_Tx_Data_Type;
    (void)__Frame_Header;   // JustFloat 没有帧头，该参数仅为保持函数接口兼容而保留

    UART_Rx_Length = 0;
    Variable_Index = -1;
}

/**
 * @brief 获取当前接收的指令在指令字典中的编号
 *
 * @return int8_t 当前接收的指令在指令字典中的编号
 */
int8_t Class_Serialplot::Get_Variable_Index()
{
    return (Variable_Index);
}

/**
 * @brief 获取当前接收的指令在指令字典中的值
 *
 * @return double 当前接收的指令在指令字典中的值
 */
double Class_Serialplot::Get_Variable_Value()
{
    return (Variable_Value);
}

/**
 * @brief 添加被发送的数据
 *
 * @param Number 添加的数据数量
 * @param ... 每个数据的指针, 至多12个
 */
void Class_Serialplot::Set_Data(uint8_t Number, ...)
{
    va_list data_ptr;//游标，记录现在取到第几个参数
    va_start(data_ptr, Number);//初始化游标，指向第一个可变参数（Number 是最后一个固定参数，可变参数从它后面开始）
    for (int i = 0; i < Number; i++)
    {
        Data[i] = (void *)va_arg(data_ptr, int);//按类型取参数，游标后移
    }
    va_end(data_ptr);//收尾清理
    Data_Number = Number;
    //Serialplot.Set_Data(3, &angle, &current, &target);传的是地址不是值，传给Data[]这是发送的，字典是接收的
}

/**
 * @brief 判断指令变量名
 *
 * @return uint8_t 指令数值位置的指针, 也就是"variable=value#"中v的坐标
 */
uint8_t Class_Serialplot::Judge_Variable_Name()//协议解析
{
    char tmp_variable_name[SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH];//栈上 100 字节，临时存变量名
    uint16_t flag = 0;
    uint8_t name_len = 0;

    Variable_Index = -1;

    // 只在本帧真实长度内找 '='。原来的写法没有边界：单独发一个 'e'（缓冲区里没有 '='）
    // 会从 64 字节缓冲区一路往后扫，同时往下面这个 100 字节栈数组里写，直接把栈踩穿。
    while (flag < UART_Rx_Length)
    {
        uint8_t ch = UART_Rx_Data[flag];

        if (ch == '=')
        {
            break;
        }
        if (ch == 0 || ch == '\r' || ch == '\n')
        {
            return 0;//没有 '='，不是调参指令
        }
        if (name_len >= SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH - 1)
        {
            return 0;//名字太长，丢弃，绝不越界
        }
        tmp_variable_name[name_len] = (char)ch;
        name_len++;
        flag++;
    }

    if (flag >= UART_Rx_Length || UART_Rx_Data[flag] != '=' || name_len == 0)
    {
        return 0;
    }

    tmp_variable_name[name_len] = 0;//补字符串结尾

    for (uint8_t i = 0; i < UART_Rx_Variable_Num; i++)//查字典
    {
        if (strcmp(tmp_variable_name, (char *)((int)UART_Rx_Variable_List + SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH * i)) == 0)
        {
            Variable_Index = (int8_t)i;//记下行号也就是第几个
            break;
        }
    }

    return (uint8_t)(flag + 1);//= 后面的第一个字符，交给取值函数
}

/**
 * @brief 判断指令变量数值
 *
 */
void Class_Serialplot::Judge_Variable_Value(int flag)
{
    int tmp_sign_coefficient = 1;
    double tmp_value = 0.0;
    double tmp_scale = 1.0;
    bool tmp_dot = false;
    uint16_t i = (uint16_t)flag;

    Variable_Value = 0.0;

    if (Variable_Index == -1)//没匹配到直接走人
    {
        return;
    }

    if (i < UART_Rx_Length && UART_Rx_Data[i] == '-')//负号
    {
        tmp_sign_coefficient = -1;
        i++;
    }

    for (; i < UART_Rx_Length; i++)
    {
        uint8_t ch = UART_Rx_Data[i];

        if (ch >= '0' && ch <= '9')
        {
            tmp_value = tmp_value * 10.0 + (double)(ch - '0');
            if (tmp_dot)
            {
                tmp_scale *= 10.0;
            }
        }
        else if (ch == '.' && !tmp_dot)
        {
            tmp_dot = true;
        }
        else
        {
            break;//'#'、0、'\r'、'\n' 都当结束，兼容串口助手自动加的换行
        }
    }

    Variable_Value = tmp_value / tmp_scale * (double)tmp_sign_coefficient;
}

/**
 * @brief 串口绘图数据输出到UART发送缓冲区
 *
 */
void Class_Serialplot::Output()
{
    // 第 1 步：把每个通道的 float 按小端排进发货架
    for (int i = 0; i < Data_Number; i++)
    {
        const float *p = (const float *)Data[i];   // 顺着地址找到那个 float
        uint32_t bits;
        memcpy(&bits, p, 4);                       // 原样搬 4 字节，不做数值换算

        UART_Tx_Data[4 * i + 0] = (uint8_t)(bits >> 0);    // 最低字节在前 = 小端
        UART_Tx_Data[4 * i + 1] = (uint8_t)(bits >> 8);
        UART_Tx_Data[4 * i + 2] = (uint8_t)(bits >> 16);
        UART_Tx_Data[4 * i + 3] = (uint8_t)(bits >> 24);   // 最高字节在后
    }

    // 第 2 步：紧跟着补 4 字节帧尾
    uint8_t tail = (uint8_t)(4 * Data_Number);     // 帧尾从第几个字节开始
    UART_Tx_Data[tail + 0] = 0x00;
    UART_Tx_Data[tail + 1] = 0x00;
    UART_Tx_Data[tail + 2] = 0x80;
    UART_Tx_Data[tail + 3] = 0x7F;
}

/**
 * @brief 返回本帧应该发送的字节数
 */
uint16_t Class_Serialplot::Get_Tx_Length(void)
{
    return ((uint16_t)(4 * Data_Number + 4));
}

/**
 * @brief TIM定时器中断增加数据写入回调函数
 *
 */
void Class_Serialplot::TIM_Add_PeriodElapsedCallback()
{
    // 用 DMA 发：20 字节 @115200 要 1.7ms，阻塞发会把主循环按在这里；DMA 这一句立刻返回，
    // 10ms 一帧的节奏很宽松，缓冲区又是全局的，不存在 DMA 读到栈上野指针的问题。
    //
    // 这里只能看 huart->gState，不能用 HAL_UART_GetState()：那个函数把 gState 和 RxState
    // 按位或，而 RX 的 ReceiveToIdle_DMA 是一直挂着的，GetState() 永远不会返回 READY。
    if (huart->gState == HAL_UART_STATE_READY)
    {
        UART_Tx_Busy_Count = 0;
        Output();//只有真要发的时候才装车，避免动到正在 DMA 的缓冲区
        HAL_UART_Transmit_DMA(huart, UART_Tx_Data, Get_Tx_Length());
    }
    else
    {
        // 正常情况下 10ms 一帧撞不上（1.7ms 就发完了）。连续 10 帧（100ms）还卡在 BUSY，
        // 说明 DMA 的完成中断没跑到（UART_DMATransmitCplt 不执行，gState 就永远停在
        // BUSY_TX），abort 一次把它拉回 READY。之前“发着发着永久 BUSY”就是这个坑。
        if (UART_Tx_Busy_Count < 10)
        {
            UART_Tx_Busy_Count++;
        }
        else
        {
            HAL_UART_AbortTransmit(huart);
            UART_Tx_Busy_Count = 0;
        }
    }
}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Serialplot::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)//把查字典中的名字和名字对应的数值进行打包
{
    int flag;

    (void)Rx_Data;//数据已经在 UART_Rx_Data 指向的缓冲区里，这里只需要长度做边界

    UART_Rx_Length = Length;
    flag = Judge_Variable_Name();//记下行号Variable_Index
    Judge_Variable_Value(flag);//记下Variable_Value
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
