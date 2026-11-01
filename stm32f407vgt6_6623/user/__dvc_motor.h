/**
 * @file dvc_motor.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief CAN电机配置与操作
 * @version 0.1
 * @date 2022-08-03
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DVC_MOTOR_H
#define DVC_MOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "__alg_pid.h"
#include "__drv_can.h"

/* Exported macros -----------------------------------------------------------*/

// RPM换算到rad/s
//转/分钟rpm × 6 = 度/秒;
#define RPM_TO_RADPS (2.0f * PI / 60.0f)//360/360*6

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机状态
 *
 */
enum Enum_CAN_Motor_Status
{
    CAN_Motor_Status_DISABLE = 0,
    CAN_Motor_Status_ENABLE,
};
//判断电机上电没有(通过看收没收到反馈报文来确定他的状态)
//根据上没上电来确定现在应不应该打开积分项的计算，因为电机掉电时积分项会加到非常大导致严重的超调，所以断电了就把积分项清零

/**
 * @brief CAN电机的ID枚举类型
 *
 */
enum Enum_CAN_Motor_ID
{
    CAN_Motor_ID_UNDEFINED = 0,
    CAN_Motor_ID_0x201,
    CAN_Motor_ID_0x202,
    CAN_Motor_ID_0x203,
    CAN_Motor_ID_0x204,
    CAN_Motor_ID_0x205,
    CAN_Motor_ID_0x206,
    CAN_Motor_ID_0x207,
    CAN_Motor_ID_0x208,
    CAN_Motor_ID_0x209,
    CAN_Motor_ID_0x20A,
    CAN_Motor_ID_0x20B,
};


/**
 * @brief 电机控制方式
 *
 */
enum Enum_Control_Method
{
    Control_Method_OPENLOOP = 0,//开环
    Control_Method_TORQUE,//电流环
    Control_Method_OMEGA_ONLY,//速度环
    Control_Method_ANGLE,//角度环串级
     Control_Method_ANGLE_ONLY//单角度环
};
enum Enum_System_Status
{
    System_Status_IDLE = 0,       // 上电默认：输出 0，不响应运动命令
    System_Status_POSITION,       // 位置闭环
    System_Status_SPEED,          // 速度闭环
    System_Status_FAULT,          // 故障锁存
};
// 全局状态机变量：只在 __dvc_motor.cpp 里定义一次，其他文件靠 extern 引用
// 原来写在这里是“在头文件里定义”，被两个 .cpp 同时 include 时链接会报 multiply defined
extern Enum_System_Status System_Status;

/**
 * @brief GM6020无刷电机, 单片机控制输出电压
 *
 */
class Class_Motor_GM6623
{
public:
    // PID角度环控制
    Class_PID PID_Angle;//角度环
    // PID角速度环控制
    Class_PID PID_Omega;//速度环
    bool Fault_Latched = false;//故障锁存用的

    void Init(CAN_HandleTypeDef *__hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method = Control_Method_ANGLE, int32_t __Encoder_Offset = 0, float __Omega_Max = 320.0f * RPM_TO_RADPS);
    //哪条 CAN、哪个电机号、什么控制方式、编码器零点偏移、最大速度
    void Calibrate();//上电校准：发 0x3F0 'c'，电调空载堵转约 6 秒。必须先 Init 再 Calibrate


    uint16_t Get_Output_Max();
    Enum_CAN_Motor_Status Get_CAN_Motor_Status();
    float Get_Now_Angle();
    float Get_Now_Omega();
    float Get_Now_Torque();
    Enum_Control_Method Get_Control_Method();
    float Get_Target_Angle();
    float Get_Target_Omega();
    float Get_Target_Torque();
    float Get_Out();

    void Set_Control_Method(Enum_Control_Method __Control_Method);
    void Set_Target_Angle(float __Target_Angle);
    void Set_Target_Omega(float __Target_Omega);
    void Set_Target_Torque(float __Target_Torque);
    void Set_Out(float __Out);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_PID_PeriodElapsedCallback();


    float Now_Angle = 0.0f;         //当前的角度, rad
    float Now_Omega = 0.0f;         //当前的速度, rad/s（6623 不反馈转速，差分自算）
    float Now_Torque = 0.0f;        //当前实际转矩电流
    float Now_Omega_Raw = 0.0f;     //1ms 差分的原始速度, rad/s（滤波前，调参对照用）
    float Now_Angle_Deg = 0.0f;     //当前角度, 度（只给上位机看）
    float Now_Omega_RPM = 0.0f;     //当前速度, rpm（只给上位机看）
    float Delta_Encoder = 0.0f;     //每帧编码器增量, counts（诊断通道：正常在 ±82 以内）
    uint16_t Delta_Drop_Count = 0;  //被物理限幅丢弃的帧数（Keil watch 看）
    Enum_CAN_Motor_Status CAN_Motor_Status = CAN_Motor_Status_DISABLE;
    Enum_Control_Method Control_Method = Control_Method_ANGLE;
    float Target_Angle = 0.0f;      //目标的角度, rad
    float Target_Angle_Deg = 0.0f;  //目标的角度, 度（只给上位机看）
    float Target_Omega = 0.0f;      //目标的速度, rad/s
    float Target_Omega_RPM = 0.0f;  //目标的速度, rpm（只给上位机看）
    float Out = 0.0f;               //输出量, 即发给电调的电流指令
protected:
    //初始化相关变量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_CAN_Motor_ID CAN_ID;
    //发送缓存区（就是 0x1FF 或 0x2FF 帧里属于本电机的那两个字节）
    uint8_t *CAN_Tx_Data;
    //编码器偏移
    uint32_t Encoder_Offset;
    //最大速度, 需根据不同负载测量后赋值, 也就开环输出用得到
    float Omega_Max = 320.0f * RPM_TO_RADPS;
    //串级外环的梯形速度限制（只用于 Control_Method_ANGLE）

    //常量

    //一圈编码器刻度
    uint16_t Encoder_Num_Per_Round = 8192;
    //最大输出电压
    uint16_t Output_Max = 30000;
    //堵转检测: 输出超过 Stall_Out_Threshold 且转速低于 Stall_Omega_Threshold 持续 1s 判堵转
    //按速度环 Out_Max = 5000 的 90% 取, 改限幅时一起改
    float Stall_Out_Threshold = 4500.0f;
    float Stall_Omega_Threshold = 1.0f;
    //6623 反馈帧周期 1kHz = 1ms, 差分算速度用的
    float Feedback_Dt = 0.001f;
    //速度一阶低通系数, 0~1, 越小越平滑但相位滞后越大
    //0.01 → 截止约 1.6Hz、群延迟约 100ms：只适合“看”，闭环会振荡
    float Omega_Filter_Alpha = 0.01f;
    //速度 5 点滑动平均的环形缓冲（压量化噪声，群延迟 2ms）
    float Omega_Buffer[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t Omega_Buffer_Index = 0;
    float Omega_Sum = 0.0f;
    //连续丢失多少拍(2ms/拍)判掉线, 10 拍 = 20ms
    uint8_t Alive_Miss_Max = 10;


    //内部变量

    //当前时刻的电机接收flag
    uint32_t Flag = 0;
    //前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;
    //上电第一帧只记基准, 不参与差分/换圈判断
    bool First_Feedback = true;
    //连续丢帧计数
    uint8_t Alive_Miss_Count = 0;
    //堵转计时, ms
    uint16_t Stall_Time = 0;

    //接收的编码器位置, 0~8191
    uint16_t Rx_Encoder = 0;
    //接收的扭矩, 目标的扭矩, -30000~30000
    int16_t Rx_Torque = 0;

    //多圈位置
    //之前的编码器位置
    uint16_t Pre_Encoder = 0;
    //总编码器位置
    int32_t Total_Encoder = 0;
    //总圈数
    int32_t Total_Round = 0;

    //读变量

    //速度环的输出 = 电流指令给定
    float Target_Torque = 0.0f;

    //内部函数

    void Output();
};



/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
