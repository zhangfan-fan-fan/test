/**
 * @file dvc_motor.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief CAN电机配置与操作
 * @version 0.1
 * @date 2022-08-03
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include <__dvc_motor.h>
// 6623 校准帧 ID（大疆官方协议：标准帧, 8字节, Data[0]='c', 其余 0）
#define CAN_ID_6623_CALIBRATE (0x3F0)
// 这里补一声明，否则链接会报 undefined reference
extern uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID);


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 全局状态机变量（头文件里已经改成 extern，真正的定义放在这里，只此一处）
Enum_System_Status System_Status = System_Status_IDLE;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID)//获取指令数据应该写在什么地址上
{
    //初始化的时候调用过了
    uint8_t *tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[0]);//存的是我应该把命令写到那个地址上(先给个安全默认值,防止野指针)
    //然后写tx[0],tx[1]再CAN_Send_Data(&hcan1, 0x1FF, CAN1_0x1ff_Tx_Data, 8) 整帧发出去。
    if (hcan == &hcan1)
    {
        switch (__CAN_ID)
        {
        case (CAN_Motor_ID_0x201):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);
        }
        break;
        case (CAN_Motor_ID_0x202):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[2]);
        }
        break;
        case (CAN_Motor_ID_0x203):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[4]);
        }
        break;
        case (CAN_Motor_ID_0x204):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[6]);
        }
        break;
        case (CAN_Motor_ID_0x205):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[0]);
        }
        break;
        case (CAN_Motor_ID_0x206):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[2]);
        }
        break;
        case (CAN_Motor_ID_0x207):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[4]);
        }
        break;
        case (CAN_Motor_ID_0x208):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[6]);
        }
        break;
        case (CAN_Motor_ID_0x209):
        {
            tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[0]);
        }
        break;
        case (CAN_Motor_ID_0x20A):
        {
            tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[2]);
        }
        break;
        case (CAN_Motor_ID_0x20B):
        {
            tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[4]);
        }
        break;
        }
    }
 
    


    return (tmp_tx_data_ptr);
}

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_ID 绑定的CAN ID
 * @param __Control_Method 电机控制方式, 默认角度
 * @param __Encoder_Offset 编码器偏移, 默认0
 * @param __Omega_Max 最大速度, 需根据不同负载测量后赋值, 也就开环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意
 */
void Class_Motor_GM6623::Init(CAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method, int32_t __Encoder_Offset, float __Omega_Max)
{
    CAN_Manage_Object = &CAN1_Manage_Object;

    CAN_ID = __CAN_ID;
    Control_Method = __Control_Method;
    Encoder_Offset = __Encoder_Offset;//编码器偏移
    Omega_Max = __Omega_Max;//最大速度
    CAN_Tx_Data = allocate_tx_data(hcan, __CAN_ID);//获取货架地址，那两个字节应该填什么
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void Class_Motor_GM6623::Output()
{
    int16_t tmp;

    // 限幅必须作用在 Out 本身：原来限的是局部变量 tmp，写回总线的还是没限幅的 Out
    Math_Constrain(&Out, -(float)Output_Max, (float)Output_Max);

    tmp = (int16_t)Out;
    CAN_Tx_Data[0] = (uint8_t)((uint16_t)tmp >> 8);//取高八位高字节进[0]
    CAN_Tx_Data[1] = (uint8_t)((uint16_t)tmp & 0xFF);//只留低八位
		CAN_Tx_Data[2] = (uint8_t)((uint16_t)tmp >> 8);//取高八位高字节进[0]
    CAN_Tx_Data[3] = (uint8_t)((uint16_t)tmp & 0xFF);//只留低八位

}

/**
 * @brief 获取最大输出电压
 *
 * @return uint16_t 最大输出电压
 */
uint16_t Class_Motor_GM6623::Get_Output_Max()
{
    return (Output_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_CAN_Motor_Status 电机状态
 */
Enum_CAN_Motor_Status Class_Motor_GM6623::Get_CAN_Motor_Status()
{
    return (CAN_Motor_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
float Class_Motor_GM6623::Get_Now_Angle()
{
    return (Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
float Class_Motor_GM6623::Get_Now_Omega()
{
    return (Now_Omega);
}

/**
 * @brief 获取当前的扭矩, 直接采用反馈值
 *
 * @return 当前的扭矩, 直接采用反馈值
 */
float Class_Motor_GM6623::Get_Now_Torque()
{
    return (Now_Torque);
}


/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Control_Method 电机控制方式
 */
Enum_Control_Method Class_Motor_GM6623::Get_Control_Method()
{
    return (Control_Method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
float Class_Motor_GM6623::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
float Class_Motor_GM6623::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取输出量
 *
 * @return float 输出量
 */
float Class_Motor_GM6623::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __Control_Method 电机控制方式
 */
void Class_Motor_GM6623::Set_Control_Method(Enum_Control_Method __Control_Method)
{
    Control_Method = __Control_Method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
void Class_Motor_GM6623::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
    Target_Angle_Deg = __Target_Angle * (180.0f / PI);
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
void Class_Motor_GM6623::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
    Target_Omega_RPM = __Target_Omega * (60.0f / (2.0f * PI));
}

void Class_Motor_GM6623::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定输出量
 *
 * @param __Output_Voltage 输出量
 */
void Class_Motor_GM6623::Set_Out(float __Out)
{
    Out = __Out;
}


 
/* * 6623 反馈帧 8 个字节：
 *   [0][1] 机械角度 0~8191
 *   [2][3] 实际转矩电流（±13000 左右，仅诊断）
 *   [4][5] 转矩电流给定值（就是电调当前在执行我们发的指令，诊断用）
 *   [6]    电流
 *   [7]    Null 补位
 */
void Class_Motor_GM6623::CAN_RxCpltCallback(uint8_t *Rx_Data)//拆帧函数
{
    //传进来的Rx_Data是接收缓冲区里的
    uint16_t raw_encoder;
    int16_t delta_encoder;

    Flag += 1;//收一帧加 1,用来判断电机死没死的依据

    //协议说高位在前，而stm32是小端，所以
    raw_encoder = (uint16_t)(((uint16_t)Rx_Data[0] << 8) | (uint16_t)Rx_Data[1]);//机械角度，唯一控制用得上的反馈
    Rx_Torque = (int16_t)(((uint16_t)Rx_Data[2] << 8) | (uint16_t)Rx_Data[3]);//实际转矩电流
    Now_Torque = Rx_Torque;

    // 上电第一帧只当基准：Pre_Encoder 初值是 0，直接差分会被误判成“转过一圈”，
    // 角度平白差 2π、速度还会有一个上千 rad/s 的尖峰
    if (First_Feedback)
    {
        Rx_Encoder = raw_encoder;
        Pre_Encoder = raw_encoder;
        Now_Omega_Raw = 0.0f;
        Now_Omega = 0.0f;
        Delta_Encoder = 0.0f;
        First_Feedback = false;
        return;
    }

    // 14 位编码器：相邻两帧的差值一定落在 ±4096 以内，
    // 超出就说明过了一圈（8191→0 或 0→8191），补/减一整圈
    delta_encoder = (int16_t)(raw_encoder - Pre_Encoder);
    if (delta_encoder > (int16_t)(Encoder_Num_Per_Round / 2))
    {
        delta_encoder -= (int16_t)Encoder_Num_Per_Round;
        Total_Round--;
    }
    else if (delta_encoder < -(int16_t)(Encoder_Num_Per_Round / 2))
    {
        delta_encoder += (int16_t)Encoder_Num_Per_Round;
        Total_Round++;
    }

    // 实测：正电流让编码器往负方向走，反馈方向与指令相反 → 这里取反，
    // 让“正电流 = 角度增大”，角度环/速度环才是负反馈。
    // 只翻这一处：Now_Angle / Now_Omega / Delta_Encoder 都从 delta_encoder 算。
    delta_encoder = -delta_encoder;

    // 物理限幅：600 rpm 时 1ms 只有 82 counts，超过 100 不可能是真实运动。
    // 总线上偶发的 ±3500 野值就从这里挡掉
    if (delta_encoder > 100 || delta_encoder < -100)
    {
        Delta_Drop_Count++;
        return;//不更新角度、不更新速度，只计数
    }

    Pre_Encoder = raw_encoder;
    Rx_Encoder = raw_encoder;
    Delta_Encoder = (float)delta_encoder;//诊断通道：这一帧的编码器增量, counts

    // 每毫秒角度增量(rad) 角速度 = delta/8192*2π，再除以 1ms 得到 rad/s
    Now_Omega_Raw = (float)delta_encoder / (float)Encoder_Num_Per_Round * 2.0f * PI / Feedback_Dt;
    // 先做 5 点滑动平均：1 个 LSB = 0.767 rad/s，平均 5 拍后等效 0.153 rad/s，群延迟 2ms
    Omega_Sum -= Omega_Buffer[Omega_Buffer_Index];
    Omega_Buffer[Omega_Buffer_Index] = Now_Omega_Raw;
    Omega_Sum += Now_Omega_Raw;
    Omega_Buffer_Index++;
    if (Omega_Buffer_Index >= 5)
    {
        Omega_Buffer_Index = 0;
    }
    float omega_avg = Omega_Sum * 0.2f;

    // 再一阶低通 α=0.4（81Hz，群延迟 2ms）；两级合计噪声约 0.22 倍、延迟约 4ms
    Now_Omega += Omega_Filter_Alpha * (omega_avg - Now_Omega);

    // 改成积分式：每帧只累加增量，换圈已经在上面处理过，
    // 不再用“圈数×8192+当前值”反算，避免首帧/丢帧时整体跳一圈
    Total_Encoder += delta_encoder;

    Now_Angle = (float)(Total_Encoder + (int32_t)Encoder_Offset) / (float)Encoder_Num_Per_Round * 2.0f * PI;
    //总的角度=值/一圈多少值    *2pi

    // 只给上位机看的两路：角度用度、速度用 rpm；PID 内部仍然是 rad / rad/s
    Now_Angle_Deg = Now_Angle * (180.0f / PI);
    Now_Omega_RPM = Now_Omega * (60.0f / (2.0f * PI));
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Motor_GM6623::TIM_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过电机数据（本函数 2ms 调一次）
    if (Flag == Pre_Flag)
    {
        // 连续丢帧到阈值才判掉线：原来 2ms 没收到就永久 FAULT，丢一帧就停机太脆
        if (Alive_Miss_Count < Alive_Miss_Max)
        {
            Alive_Miss_Count++;
        }
        if (Alive_Miss_Count >= Alive_Miss_Max && System_Status != System_Status_FAULT)
        {
            //电机断开连接
            System_Status = System_Status_FAULT;
            CAN_Motor_Status = CAN_Motor_Status_DISABLE;
            First_Feedback = true;//重新插上时先重新取基准，别拿旧值硬差
            Set_Out(0.0f);
            Output();//立刻把 0 写进 CAN 缓冲，不能等下一轮 PID
            PID_Angle.Set_Integral_Error(0.0f);
            PID_Omega.Set_Integral_Error(0.0f);
        }
    }
    else
    {
        //电机保持连接
        Alive_Miss_Count = 0;
        CAN_Motor_Status = CAN_Motor_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Motor_GM6623::TIM_PID_PeriodElapsedCallback()
{
    switch (System_Status)
    {
    case System_Status_IDLE:
    case System_Status_FAULT:
        Set_Out(0.0f);      // 待机和故障都只置零
        Output();           // 关键：0 也要写进 CAN 缓冲，否则 0x1FF 里还是上一次的电流
        return;             // 不算 PID
    case System_Status_POSITION:
    case System_Status_SPEED:
        break;              // 放行，往下走原来的 PID 代码
    }

    // 过温保护暂时停用：6623 的 8 字节反馈里没有温度字段，Now_Temperature 永远是 0。
    // 原来这段只会把速度环限幅从 5000 悄悄改成 500（电流砍到 1/10），
    // 等附录 A 确认反馈帧格式、真能读到温度，再按 65/75℃ 恢复降额。

    switch (Control_Method)
    {
    case (Control_Method_TORQUE):
    {
        // 直接给转矩电流指令，同样受 Out_Max 限幅
        Set_Out(Target_Torque);
    }
    break;
    case (Control_Method_OPENLOOP):
    {
        //默认开环速度控制
        Set_Out(Target_Omega / Omega_Max * Output_Max);
    }
    break;
    case (Control_Method_OMEGA_ONLY):
    {
        PID_Omega.Set_Target(Target_Omega);
        PID_Omega.Set_Now(Now_Omega);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        //根据你上面输入的Target_Omega，Now_Omega来进行pid运算，把结果存进out中
        Set_Out(PID_Omega.Get_Out());
    }
    break;
    case (Control_Method_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Now_Angle);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega     = PID_Angle.Get_Out();
        Target_Omega_RPM = Target_Omega * (60.0f / (2.0f * PI));

        PID_Omega.Set_Target(Target_Omega);
        PID_Omega.Set_Now(Now_Omega);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Omega.Get_Out());//速度环的输出就是电流指令
    }
    break;
     case (Control_Method_ANGLE_ONLY):
    {
        // 只闭角度环：角度 PID 算出来的值直接当电流，中间不串速度环
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Now_Angle);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();
        Set_Out(PID_Angle.Get_Out());
    }
    break;
    default:
    {
        Set_Out(0.0f);
    }
    break;
    }

    // 堵转保护：输出已经打满、但 1 秒内几乎不转 → 认为堵住/卡死，锁存故障。
    // 考核文档要求“不要长时间堵转”，5.3A 堵转超过 1 秒就停。
    if (Math_Abs(Out) > Stall_Out_Threshold && Math_Abs(Now_Omega) < Stall_Omega_Threshold)
    {
        if (Stall_Time < 1000)
        {
            Stall_Time++;
        }
        if (Stall_Time >= 1000)
        {
            Fault_Latched = true;
            System_Status = System_Status_FAULT;
            Set_Out(0.0f);
        }
    }
    else
    {
        Stall_Time = 0;
    }

    Output();//把out的值存进 CAN_Tx_Data[0]里面发出去
}


void Class_Motor_GM6623::Calibrate()//校准
{
    // 校准帧数据：'c' + 7 个 0
    uint8_t Calibrate_Data[8] = {'c', 0, 0, 0, 0, 0, 0, 0};

    // 连发 10 次，间隔 10ms，确保电调一定收到
    for (uint8_t i = 0; i < 10; i++)
    {
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_ID_6623_CALIBRATE, Calibrate_Data, 8);
        HAL_Delay(10);
    }

    // 等电调自己找零：空载堵转约 6 秒，给 7 秒保险
    HAL_Delay(7000);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
