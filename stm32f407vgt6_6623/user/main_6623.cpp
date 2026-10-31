/**
  ******************************************************************************
 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <string.h>

#include "__drv_can.h"
#include "__drv_uart.h"
#include "__drv_math.h"
#include "__dvc_motor.h"
#include "__dvc_serialplot.h"

/* Private define ------------------------------------------------------------*/
#define ANGLE_LIMIT_MAX   (2.0f * PI)    // 目标角度限幅 ±360°
#define OMEGA_LIMIT_MAX   (20.0f)        // 目标速度限幅 ±20 rad/s
#define rpm_to_rad (2*pi/60)							//

/* Private variables ---------------------------------------------------------*/

/* 库对象 —— 每个对象对应一个库 */
Class_Motor_GM6623 Motor_1;          // dvc_motor：电机对象，内含 2 个 PID（角度环 + 速度环）
Class_Serialplot Serialplot;         // dvc_serialplot：串口绘图 + 调参对象

/* 喂给库的原料 */
char Serialplot_Dict[][100] = { "kp_angle", "kp_omega", "ki_angle", "ki_omega", "kf_angle", "kf_omega" };
uint8_t UART2_Rx_Buffer[64];         // 串口收货架


enum Enum_Test_Mode
{
    Test_Mode_STOP = 0,      // 0 停
    Test_Mode_JOG,           // 1 开环小电流点动
    Test_Mode_ANGLE_ONLY,    // 2 只闭角度环
    Test_Mode_SPEED_ONLY,    // 3 只闭速度环
    Test_Mode_CASCADE,       // 4 位置-速度串级
    Test_Mode_SINE,          // 5 正弦跟踪
};

static Enum_Test_Mode Test_Mode = Test_Mode_STOP;
static uint32_t Test_Tick = 0;        // 当前模式跑了多少 ms
static float Sine_Phase = 0.0f;       // 正弦相位

/* 各模式参数 */
#define TEST_JOG_CURRENT      200.0f      // counts，约 0.3 A
#define TEST_SPEED_STEP        300.0f      // 300 rpm（生成器里乘 2π/60 换成 rad/s）
//#define TEST_KP_ANGLE_ONLY    2200.0f      // counts/rad
//#define TEST_KD_ANGLE_ONLY     0.0f      // counts/(rad/s)
//#define TEST_KP_ANGLE          12.0f      // (rad/s)/rad
//#define TEST_KP_SPEED         600.0f      // counts/(rad/s)
//#define TEST_KI_SPEED          10.0f
#define TEST_OUT_LIMIT       5000.0f      // counts，
#define TEST_SINE_AMP         (PI / 6.0f) // ±30°
#define TEST_SINE_FREQ         0.5f       // Hz
#define TEST_KF_SINE         1000.0f      // 正弦前馈：f_out = KF × 目标增量/1ms，1000 = 1:1 速度前馈

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void CAN1_Rx_Callback(Struct_CAN_Rx_Buffer *Rx_Buffer);
void UART2_Rx_Callback(uint8_t *Buffer, uint16_t Length);


/**
  * @brief  切换测试模式：先停车、清积分，再按模式配好控制方式和 PID
  */
void Test_Mode_Apply(Enum_Test_Mode mode)
{
    // 不管切到哪个模式，先停车、把积分清零，避免上一次的残留带进来
    Motor_1.Set_Out(0.0f);
    Motor_1.PID_Angle.Set_Integral_Error(0.0f);
    Motor_1.PID_Omega.Set_Integral_Error(0.0f);
    // 目标值也清零：否则 STOP / 速度模式会残留上一次角度模式的目标角
    Motor_1.Set_Target_Angle(0.0f);
    Motor_1.Set_Target_Omega(0.0f);
    Motor_1.Set_Target_Torque(0.0f);
    Test_Tick = 0;
    Sine_Phase = 0.0f;
    Test_Mode = mode;

    switch (mode)
    {
    case Test_Mode_STOP:
        // 回待机：TIM_PID 走 IDLE 分支，输出 0
        System_Status = System_Status_IDLE;
        break;

    case Test_Mode_JOG:
        // 开环：固定电流，不跑 PID
        Motor_1.Set_Control_Method(Control_Method_TORQUE);
        System_Status = System_Status_POSITION;
        break;

    case Test_Mode_ANGLE_ONLY:
        // 只闭角度环：输出是电流，所以限幅用电流量程，Kd 当阻尼
        Motor_1.Set_Control_Method(Control_Method_ANGLE_ONLY);
        Motor_1.PID_Angle.Init( 3200.0f, 600.0f, 5.0f, 0.0f, 0.0f, 5000.0f, 0.001f);
        System_Status = System_Status_POSITION;
        break;

    case Test_Mode_SPEED_ONLY:
        // 只闭速度环：输出是电流
        Motor_1.Set_Control_Method(Control_Method_OMEGA_ONLY);
        Motor_1.PID_Omega.Init(2500.0f, 4800.0f, 5.0f, 0.0f, 5000.0f, TEST_OUT_LIMIT, 0.001f);
        System_Status = System_Status_POSITION;
        break;

    case Test_Mode_CASCADE:
        // 串级：角度环输出速度目标（rad/s），速度环输出电流（counts）
        Motor_1.Set_Control_Method(Control_Method_ANGLE);
        Motor_1.PID_Angle.Init(200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 220.0f, 0.001f);
        Motor_1.PID_Omega.Init(70.0f,  300.0f, 0.0f, 0.0f, 5000.0f, TEST_OUT_LIMIT, 0.001f);
        Motor_1.PID_Angle.Set_K_F(0.0f);
        System_Status = System_Status_POSITION;
        break;
        /*内环的p调的太大时实际速度跟的很快，目标速度是跟90度-实际角度有关，误差掉得快，
        目标速度就掉得快而p小一点，目标速度下降了实际的还在猛猛上冲，目标速度就因为误差小了下降的更凶了*/
    case Test_Mode_SINE:
        // 正弦：串级 速度前馈
        Motor_1.Set_Control_Method(Control_Method_ANGLE);
        Motor_1.PID_Angle.Init(12.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.001f);
        Motor_1.PID_Omega.Init(600.0f, 20.0f, 0.0f, 0.0f, 5000.0f, TEST_OUT_LIMIT, 0.001f);
        Motor_1.PID_Angle.Set_K_F(TEST_KF_SINE);
        System_Status = System_Status_POSITION;
        break;

    default:
        System_Status = System_Status_IDLE;
        break;
    }
}

/**
  * @brief  电机反馈帧回调：电调发给 MCU 的帧落到 FIFO 就调一次
  */
void CAN1_Rx_Callback(Struct_CAN_Rx_Buffer *Rx_Buffer)
{

    if (Rx_Buffer->Header.StdId == 0x205)
    {
        Motor_1.CAN_RxCpltCallback(Rx_Buffer->Data);   // dvc_motor 的拆帧函数
    }
}

/**
  * @brief  上位机串口回调：收到一条指令就调一次
  */
void UART2_Rx_Callback(uint8_t *Buffer, uint16_t Length)
{
    if (Length == 0)
    {
        return;
    }

    Serialplot.UART_RxCpltCallback(Buffer, Length);   // 查字典：命令名 -> 行号 -> 数值

        switch (Buffer[0])                        // 只在“真收到一个字节”这一刻判断
    {
    case '0': case 'r':                       // 停
        Test_Mode_Apply(Test_Mode_STOP);
        break;
    case '1':                                 // 开环小电流点动
        Test_Mode_Apply(Test_Mode_JOG);
        break;
    case '2':                                 // 只闭角度环
        Test_Mode_Apply(Test_Mode_ANGLE_ONLY);
        break;
    case '3': case 's':                       // 只闭速度环
        Test_Mode_Apply(Test_Mode_SPEED_ONLY);
        break;
    case '4': case 'e':                       // 位置-速度串级
        Test_Mode_Apply(Test_Mode_CASCADE);
        break;
    case '5':                                 // 正弦跟踪
        Test_Mode_Apply(Test_Mode_SINE);
        break;
    default:                                  // kp_angle=2.0 这类调参指令，不在这里处理
        break;
    }

    switch (Serialplot.Get_Variable_Index())  // 行号比对，把数值写进对应的 PID
    {
    case 0:  Motor_1.PID_Angle.Set_K_P(Serialplot.Get_Variable_Value());  break;
    case 1:  Motor_1.PID_Omega.Set_K_P(Serialplot.Get_Variable_Value());  break;
    case 2:  Motor_1.PID_Angle.Set_K_I(Serialplot.Get_Variable_Value());  break;
    case 3:  Motor_1.PID_Omega.Set_K_I(Serialplot.Get_Variable_Value());  break;
    case 4:  Motor_1.PID_Angle.Set_K_F(Serialplot.Get_Variable_Value());  break;
    case 5:  Motor_1.PID_Omega.Set_K_F(Serialplot.Get_Variable_Value());  break;
    default: break;                           // -1 = 没命中字典，不理
    }
}

/**
  * @brief  定时器中断：TIM6 每 1ms 调一次，两环 PID / 0x1FF 发帧 / 喂狗都在这
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        static uint16_t count = 0;            // 原来 uint8_t，255 回 0 会让 %10 不均匀
        count++;

        if (count % 2 == 0)
        {
            Motor_1.TIM_Alive_PeriodElapsedCallback();   // 2ms 查一次电机死没死（死了清积分）
        }

				
				Test_Tick++;
        switch (Test_Mode)
        {
        case Test_Mode_JOG:
            if      (Test_Tick <  500) Motor_1.Set_Target_Torque(+TEST_JOG_CURRENT);
            else if (Test_Tick <  750) Motor_1.Set_Target_Torque(0.0f);
            else if (Test_Tick <  1250) Motor_1.Set_Target_Torque(-TEST_JOG_CURRENT);
            else if (Test_Tick <  1500) Motor_1.Set_Target_Torque(0.0f);
            else                        Test_Tick = 0;
            break;

        case Test_Mode_ANGLE_ONLY:
        case Test_Mode_CASCADE:
            if      (Test_Tick <  1000) Motor_1.Set_Target_Angle(0.0f);
            else if (Test_Tick <  2500) Motor_1.Set_Target_Angle(PI / 2.0f);
            else if (Test_Tick <  4000) Motor_1.Set_Target_Angle(0.0f);
            else if (Test_Tick < 5500) Motor_1.Set_Target_Angle(-PI / 2.0f);
            else                        Test_Tick = 0;
            break;

        case Test_Mode_SPEED_ONLY:
            // TEST_SPEED_STEP 是 rpm，这里乘 2π/60 换成 rad/s
            if      (Test_Tick <  500) Motor_1.Set_Target_Omega(0.0f);
            else if (Test_Tick <  1000) Motor_1.Set_Target_Omega(+TEST_SPEED_STEP * (1.0f * PI / 60.0f));
            else if (Test_Tick <  1500) Motor_1.Set_Target_Omega(0.0f);
            else if (Test_Tick < 2000) Motor_1.Set_Target_Omega(-TEST_SPEED_STEP * (1.0f * PI / 60.0f));
            else                        Test_Tick = 0;
            break;

        case Test_Mode_SINE:
            Sine_Phase += 2.0f * PI * TEST_SINE_FREQ * 0.001f;
            if (Sine_Phase > 2.0f * PI)
            {
                Sine_Phase -= 2.0f * PI;
            }
            Motor_1.Set_Target_Angle(TEST_SINE_AMP * sinf(Sine_Phase));
            break;

        default:
            break;
        }
				
				
        Motor_1.TIM_PID_PeriodElapsedCallback();         // 1ms 算一次两环 PID
        TIM_CAN_PeriodElapsedCallback();                 // 1ms 把 0x1FF 货架发上总线

        // 看门狗：每 1ms 喂一次。IWDG 是 4 分频 + 4095 重装 = 约 512ms 超时。
        HAL_IWDG_Refresh(&hiwdg);
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();                 // 必须在 USART 初始化之前
    MX_CAN1_Init();
    MX_USART2_UART_Init();
    MX_TIM6_Init();

    
    CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    // ② 再启动 CAN + 登记接收回调
    CAN_Init(&hcan1, CAN1_Rx_Callback);
    CAN1_0x1ff_Tx_Data[6] = 0x01;              // 6623 电流指令的第一个字节要写 1
    Uart_Init(&huart2, UART2_Rx_Buffer, 64, UART2_Rx_Callback);

    Motor_1.Init(&hcan1, CAN_Motor_ID_0x205, Control_Method_ANGLE);
   
    // 角度环：Kp 先给 2.0，Out_Max = 20 rad/s = 角度环能要出的最大速度
    Motor_1.PID_Angle.Init(2000.0f, 300.0f, 200.0f, 0.0f, 10.0f, 10.0f, 0.001f);//40约等于382rpm
    // 速度环：Out_Max = 5000 = 6623 电流指令上限
    Motor_1.PID_Omega.Init(4.0f, 4.05f, 5.0f, 0.0f, 0.0f, 5000.0f, 0.001f);//5000约5.3A

    // 调参起点（按“5000 counts = 5.3A、J≈0.014 kg·m²、Kt=0.38 N·m/A”粗估）：
    //   速度环 Kp ≈ J·ωc/Kt × (5000/5.3) ≈ 3000~6000（ωc=15~30Hz），Ki 从 Kp/100 起
    //   角度环 Kp ≈ ω_outer ≈ 10~20
    
    // 上电校准：必须先 Init 再 Calibrate，电调会空载堵转约 6 秒，确认安全再打开这一行
    // Motor_1.Calibrate();
    

    Serialplot.Init(&huart2, 6, (char **)Serialplot_Dict, Serialplot_Data_Type_FLOAT, 0xAB);
    Serialplot.Set_Data(5, &Motor_1.Target_Angle_Deg, &Motor_1.Now_Angle_Deg,
                           &Motor_1.Now_Omega_RPM, &Motor_1.Target_Omega_RPM, &Motor_1.Out);
    // 五条曲线：目标角度(°) / 实际角度(°) / 实际速度(rpm) / 目标速度(rpm) / 


    MX_IWDG_Init();

    HAL_TIM_Base_Start_IT(&htim6);   // 开 1ms 心跳（定时器里会喂狗，所以狗要在这之前启动）

    uint32_t last_plot = 0;

    while (1)
    {
        
        //各个模式的改动在定时器中断里面
				 
    
        uint32_t now = HAL_GetTick();

        if (now - last_plot >= 5)
        {
            last_plot = now;
            Serialplot.TIM_Add_PeriodElapsedCallback();
        }
    




    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
