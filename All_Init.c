
#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��������ģ���ʼ��
//  @param      ��
//  @return     ��
//  @note       ��������ģ���ʼ��
//-------------------------------------------------------------------------------------------------------------------
void Car_Initall(void)
{
    // ��ȡʱ��Ƶ��  ��ر���
    clock_init();   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();   // ��ʼ��Ĭ�ϵ��Դ���

    // �����ǳ�ʼ��
    imu660ra_init();
//    gyroOffset_init();

    // ��������ʼ��
//    gpio_init( Beep , GPO , 0 , GPO_PUSH_PULL );
//    gpio_init( P20_11 , GPO , 1 , GPO_PUSH_PULL );
//    gpio_init( Beep , GPO , 1 , GPO_PUSH_PULL );


    // ��������ʼ��--����������
    encoder_quad_init( TIM2_ENCODER , TIM2_ENCODER_CH1_P33_7 , TIM2_ENCODER_CH2_P33_6 );
    encoder_quad_init( TIM6_ENCODER , TIM6_ENCODER_CH1_P20_3 , TIM6_ENCODER_CH2_P20_0 );

    // �����ʼ��
    pwm_init( ATOM0_CH5_P02_5 , 300 ,4705 );    // 1050��  975��  900��  ����75//4710

    // �������
    pwm_init( ATOM0_CH2_P21_4 , 17000 , motorpwm_output );    // ����
    pwm_init( ATOM0_CH3_P21_5 , 17000 , motorpwm_output );    // ����
    pwm_init( ATOM0_CH1_P21_3 , 17000 , motorpwm_output );    // ����
    pwm_init( ATOM0_CH0_P21_2 , 17000 , motorpwm_output );    // ����

    // �ȴ����к��ĳ�ʼ�����
    cpu_wait_event_ready();
    enableInterrupts();

    // ������ʼ��
    gpio_toggle_level( Beep );
    system_delay_ms( 100 );
    gpio_set_level( Beep , 1 );

    // ���ߴ��ڳ�ʼ��
    wireless_uart_init();
//    uart_init( UART_2 , 115200 , UART2_TX_P10_5 , UART2_RX_P10_6 );

    // �����ж�����
    pit_ms_init( CCU60_CH1 , 1 );   // 1ms����1��
    pit_ms_init( CCU61_CH0 , 10 );  // 10ms����0��

    // PID������ʼ��
    Pid_Param_Init( &pid_type.PID_Steer );
//    Pid_Param_Init( &pid_type.PID_Drive );
    Pid_Param_Init( &pid_type.PID_l );
    Pid_Param_Init( &pid_type.PID_r );

    // ����������ʼ��
//    garage_mode = Left_Entry;    // ��ⷽʽ
    steermotor_enable = 1;          // �����������
//    patch_line = 1;                 // ��ʼ����������
    Speed_strategy_Init( &speed_type );        //�ٶȲ��Գ�ʼ��
    Encoder_Param_Init( &encoder );      //������������ʼ��

//    InitFuzzyMachine( &Fuzzy_Kp , Rule_Kp , -50 , 50 , -25 ,25 , 0.1, 10 );      // ����ͷ//8  InitFuzzyMachine( &Fuzzy_Kp , Rule_Kp , -40 , 40 , -20 ,20 , 1 , 8 );

    InitFuzzyMachine( &Fuzzy_Kp , Rule_Kp , -40 , 40 , -10 ,10 , 0.1 , 8.5 );
/*6.501
    round_p.L_Circle_Enable = 1;    // �������뻷
    round_p.R_Circle_Enable = 1;    // �������뻷
    garage_p.L_Garage_Enable = 1;   // ������෽���
    garage_p.R_Garage_Enable = 1;   // �����Ҳ෽���
*/

}
