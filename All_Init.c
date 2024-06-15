
#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      车车所有模块初始化
//  @param      无
//  @return     无
//  @note       车车所有模块初始化
//-------------------------------------------------------------------------------------------------------------------
void Car_Initall(void)
{
    // 获取时钟频率  务必保留
    clock_init();   // 获取时钟频率<务必保留>
    debug_init();   // 初始化默认调试串口

    // 陀螺仪初始化
    imu660ra_init();
//    gyroOffset_init();

    // 蜂鸣器初始化
//    gpio_init( Beep , GPO , 0 , GPO_PUSH_PULL );
//    gpio_init( P20_11 , GPO , 1 , GPO_PUSH_PULL );
//    gpio_init( Beep , GPO , 1 , GPO_PUSH_PULL );


    // 编码器初始化--左、右两个轮
    encoder_quad_init( TIM2_ENCODER , TIM2_ENCODER_CH1_P33_7 , TIM2_ENCODER_CH2_P33_6 );
    encoder_quad_init( TIM6_ENCODER , TIM6_ENCODER_CH1_P20_3 , TIM6_ENCODER_CH2_P20_0 );

    // 舵机初始化
    pwm_init( ATOM0_CH5_P02_5 , 300 ,4705 );    // 1050左  975中  900右  左右75//4710

    // 电机驱动
    pwm_init( ATOM0_CH2_P21_4 , 17000 , motorpwm_output );    // 左轮
    pwm_init( ATOM0_CH3_P21_5 , 17000 , motorpwm_output );    // 左轮
    pwm_init( ATOM0_CH1_P21_3 , 17000 , motorpwm_output );    // 右轮
    pwm_init( ATOM0_CH0_P21_2 , 17000 , motorpwm_output );    // 右轮

    // 等待所有核心初始化完毕
    cpu_wait_event_ready();
    enableInterrupts();

    // 其他初始化
    gpio_toggle_level( Beep );
    system_delay_ms( 100 );
    gpio_set_level( Beep , 1 );

    // 无线串口初始化
    wireless_uart_init();
//    uart_init( UART_2 , 115200 , UART2_TX_P10_5 , UART2_RX_P10_6 );

    // 设置中断周期
    pit_ms_init( CCU60_CH1 , 1 );   // 1ms——1核
    pit_ms_init( CCU61_CH0 , 10 );  // 10ms——0核

    // PID参数初始化
    Pid_Param_Init( &pid_type.PID_Steer );
//    Pid_Param_Init( &pid_type.PID_Drive );
    Pid_Param_Init( &pid_type.PID_l );
    Pid_Param_Init( &pid_type.PID_r );

    // 其他参数初始化
//    garage_mode = Left_Entry;    // 入库方式
    steermotor_enable = 1;          // 舵机正常控制
//    patch_line = 1;                 // 初始化正常补线
    Speed_strategy_Init( &speed_type );        //速度策略初始化
    Encoder_Param_Init( &encoder );      //编码器参数初始化

//    InitFuzzyMachine( &Fuzzy_Kp , Rule_Kp , -50 , 50 , -25 ,25 , 0.1, 10 );      // 摄像头//8  InitFuzzyMachine( &Fuzzy_Kp , Rule_Kp , -40 , 40 , -20 ,20 , 1 , 8 );

    InitFuzzyMachine( &Fuzzy_Kp , Rule_Kp , -40 , 40 , -10 ,10 , 0.1 , 8.5 );
/*6.501
    round_p.L_Circle_Enable = 1;    // 允许左入环
    round_p.R_Circle_Enable = 1;    // 允许右入环
    garage_p.L_Garage_Enable = 1;   // 允许左侧方入库
    garage_p.R_Garage_Enable = 1;   // 允许右侧方入库
*/

}
