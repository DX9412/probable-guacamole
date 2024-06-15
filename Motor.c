
#include "zf_common_headfile.h"
#define limit(x, y)     ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))    // 限幅 数据范围是 [-32768,32767]

/********************参数定义********************/
Encoder_Param encoder;
float steerpid_param[4] = { 0.34 , 0 ,0 , 0 };
//float drivepid_param[4] = { 50 , 0 , 0 , 0 };
float l_speedpid_param[3]={160,10,0};
float r_speedpid_param[3]={150,14,0};
uint8 steermotor_enable = 1;
PID_ERECT pid_type;
SPEED_TYPE speed_type;

/********************参数定义********************/
struct PID dpid = {0};

/********************全局变量定义********************/
int16 l_speed_now = 0;           //左轮当前速度
int16 r_speed_now = 0;           //右轮当前速度
int16 l_speed_aim = 0;           //左轮目标速度
int16 r_speed_aim = 0;           //右轮目标速度
int16 L_pid_out=0;
int16 R_pid_out=0;
int16 differential=0;
//int16 differential = 0;          //后轮差速值
float kp_6=0;
float kd_6=0;
/********************全局变量定义********************/



void Speed_strategy_Init( SPEED_TYPE *speed_type )
{
    //速度策略1
    speed_type->straight    = 150;
    speed_type->bend        = 140;
    speed_type->cross       = 140;
    speed_type->fork        = 140;
    speed_type->cirque      = 35;
    speed_type->lean_cross  = 100;
    speed_type->ramp        = 200;
    speed_type->barn        = 140;
    speed_type->stop        = 0;
}



void Encoder_Param_Init( Encoder_Param *encoder )
{
    encoder->l_encoder_pulse=0;
    encoder->r_encoder_pulse=0;
    encoder->encoder_enable=0;
    encoder->encodersum_left=0;
    encoder->encodersum_right=0;
    encoder->encodersum_average=0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PID参数初始化
//  @param      pid_info        PID结构体
//  @return     无
//  @note       PID参数初始化
//-------------------------------------------------------------------------------------------------------------------
void Pid_Param_Init( PID_INFO *pid_info )
{
    pid_info->iError = 0;
    pid_info->SumError = 0;
    pid_info->PrevError = 0;
    pid_info->LastError = 0;
    pid_info->LastData = 0;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      编码器读取积分
//  @param      无
//  @return     无
//  @note       编码器读取积分
//-------------------------------------------------------------------------------------------------------------------
void Encoder_Integral(Encoder_Param *encoder)
{
    encoder->l_encoder_pulse = -encoder_get_count( TIM2_ENCODER );
    encoder->r_encoder_pulse = encoder_get_count( TIM6_ENCODER );
    if( encoder->encoder_enable == 1 )
    {
        encoder->encodersum_left += encoder->l_encoder_pulse;
        encoder->encodersum_right += encoder->r_encoder_pulse;
        encoder->encodersum_average = ( encoder->encodersum_left + encoder->encodersum_right ) /  2;
    }
    else if( encoder->encoder_enable == 0 )
    {
        encoder->encodersum_left = 0;
        encoder->encodersum_right = 0;
        encoder->encodersum_average = 0;
    }
    encoder_clear_count( TIM2_ENCODER );
    encoder_clear_count( TIM6_ENCODER );
    l_speed_now= encoder->l_encoder_pulse;
    r_speed_now= encoder->r_encoder_pulse;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      驱动电机控制
//  @param      pos     驱动轮位置  0 左轮  1 右轮
//  @param      dir     转动方向    1 正转  0 反转
//  @param      pwm     占空比      0-13000
//  @param      stop    停止信号    0 启动  1 停止
//  @return     无
//  @note       驱动电机控制
//-------------------------------------------------------------------------------------------------------------------
void DriveMotor_Ctrl( int16 l_pwm  , int16 r_pwm )
{


            // 正转
            if( l_pwm >0 )
            {
                l_pwm=Limit_Protection( l_pwm , 0 , 6000 );
                pwm_set_duty( ATOM0_CH2_P21_4  , l_pwm );

                pwm_set_duty( ATOM0_CH3_P21_5 , 0 );
            }
            // 反转
            else
            {
                l_pwm= Limit_Protection( -l_pwm , 0 , 6000);
                pwm_set_duty( ATOM0_CH2_P21_4 , 0 );

                pwm_set_duty( ATOM0_CH3_P21_5 , l_pwm );
            }


            // 正转
            if( r_pwm >0 )
            {
                r_pwm=Limit_Protection  ( r_pwm , 0 , 6000 );
                pwm_set_duty( ATOM0_CH1_P21_3 , r_pwm );
                pwm_set_duty( ATOM0_CH0_P21_2 , 0 );
            }
            // 反转
            else
            {
                r_pwm= Limit_Protection( -r_pwm , 0 , 6000 );

                pwm_set_duty( ATOM0_CH1_P21_3 , 0 );

                pwm_set_duty( ATOM0_CH0_P21_2 , r_pwm );
            }




}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      舵机电机控制
//  @param      pos     驱动轮位置  0 左轮  1 右轮
//  @param      dir     转动方向  1 正转  0 反转
//  @param      pwm     占空比   0-13000
//  @return     无
//  @note       舵机电机控制
//-------------------------------------------------------------------------------------------------------------------
void SteerMotor_Ctrl( int pwm )
{
    pwm_set_duty( ATOM0_CH5_P02_5 , Limit_Protection( 4705+pwm , 4265 , 5165 ) );
//    pwm_set_duty( ATOM0_CH5_P02_5 , Limit_Protection( 4740+pwm , 4290 , 5190 ) );

}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      uint32限幅保护
//  @param      current_value   当前值
//  @param      min             最小值
//  @param      max             最大值
//  @return     限幅后的值
//  @note       uint32限幅保护
//-------------------------------------------------------------------------------------------------------------------
int16 Limit_Protection( int16 current_value , int16 min , int16 max )
{
    if( current_value >= max )
    {
        return max;
    }
    if( current_value <= min )
    {
        return min;
    }
    else
    {
        return current_value;
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      舵机位置式PID
//  @param      pid_info        PID结构体
//  @param      PID_Parm        PID参数数组
//  @param      NowPoint        当前值
//  @param      SetPoint        目标值
//  @return     Position        计算所得值
//  @note       舵机位置式PID
//-------------------------------------------------------------------------------------------------------------------
//float PID_Position_Steer( PID_INFO *pid_info , float *PID_Parm , float NowPoint , float SetPoint )
//{
//    // 输出量
//    float Position;
//    kd_6= pid_info->LastError ;
//    // 当前误差
//    pid_info->iError = -( SetPoint - NowPoint );
//
//    // i项积分限幅
//    if( PID_Parm[3] )
//    {
//        pid_info->SumError = limit( pid_info->SumError , PID_Parm[3] );
//    }
//
//    // 位置式PID计算公式
//    Position = PID_Parm[KP] * pid_info->iError +
//               PID_Parm[KI] * pid_info->SumError +
//               PID_Parm[KD] * ( pid_info->iError - pid_info->LastError );
//
//    // 误差传递
//    kp_6=pid_info->iError ;
//
////    pid_info->PrevError = pid_info->LastError;
//    pid_info->LastError = pid_info->iError;
////    pid_info->LastData = NowPoint;
//
//    // 返回输出值
//    return Position;
//}


void Direction_pid(struct PID *sptr)
{
//    float real_kp;

    sptr->error = Error1;      //面积法计算的偏差值
//    real_kp = 1.0*(sptr->error*sptr->error)/sptr->ki+sptr->kp;
    sptr->out = (int16)(sptr->kp*sptr->error+sptr->kd*(sptr->error-sptr->last_error));
//    sptr->out = Filter_first(sptr->out, sptr->last_out, 0.2);//一阶低通滤波
//    kp_6=sptr->error;
//    kd_6=sptr->error-sptr->last_error;
    sptr->last_error = sptr->error;
    sptr->last_out = sptr->out;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机位置式PID
//  @param      pid_info        PID结构体
//  @param      PID_Parm        PID参数数组
//  @param      NowPoint        当前值
//  @param      SetPoint        目标值
//  @return     Position        计算所得值
//  @note       电机位置式PID
//-------------------------------------------------------------------------------------------------------------------
//float PID_Position_Motor( PID_INFO *pid_info , float *PID_Parm , float NowPoint , float SetPoint )
//{
//    // 输出量
//    float Position;
//
//    // 当前误差
//    pid_info->iError = ( NowPoint - SetPoint );
//
//    // i项积分限幅
//    if( PID_Parm[3] )
//    {
//        pid_info->SumError = limit( pid_info->SumError , PID_Parm[3] );
//    }
//
//    // 位置式PID计算公式
//    Position = PID_Parm[KP] * pid_info->iError +
//               PID_Parm[KI] * pid_info->SumError +
//               PID_Parm[KD] * ( pid_info->iError - pid_info->LastError );
//
//    // 误差传递
//    pid_info->PrevError = pid_info->LastError;
//    pid_info->LastError = pid_info->iError;
//    pid_info->LastData = NowPoint;
//
//    // 返回输出值
//    return Position;
//}


/***********************************************
函数名：L_speed_error
功  能：根据赛道类型选择不同的左轮速度并计算速度偏差
参  数：void
返回值：int16
 ***********************************************/

int16 L_speed_error(void)
{
    int16 error = 0;
//    if( encoder.encodersum_average < 7500 && encoder.encodersum_average >500)
//         {
//           l_speed_aim = -50;
//           eulerAngle.pitch=-18;
//           error = l_speed_aim-l_speed_now;
//         }
//
//    else if(eulerAngle.pitch>= 8)
//    {
//        l_speed_aim = 50;
//         error = l_speed_aim-l_speed_now;
//    }

//    //直道速度偏差计算
//    if(eulerAngle.pitch<= -8)
//    {
//        l_speed_aim = speed_type.ramp;
//         error = l_speed_aim-l_speed_now;
//    }
//    else if(road_type.straight)
//    {
//        l_speed_aim = speed_type.straight;
//        error = l_speed_aim-l_speed_now;
//    }
//    //弯道速度偏差计算
//    else if(road_type.R_bend || road_type.L_bend)
//    {
//        l_speed_aim = speed_type.bend;
//        error = l_speed_aim-l_speed_now;
//    }
//    //十字速度偏差计算
//    else if(road_type.Cross)
//    {
//        l_speed_aim = speed_type.cross;
//        error = l_speed_aim-l_speed_now;
//    }
//
//    //环岛速度偏差计算
//    else
//        if(road_type.LeftCirque)
//    {
//        if(annulus_L_memory == 3)
//        {
//            l_speed_aim = speed_type.cirque-2;
//            error = l_speed_aim-l_speed_now;
//        }
//        else if((annulus_L_memory == 4))
//        {
//            l_speed_aim = speed_type.cirque+2 ;
//            error = l_speed_aim-l_speed_now;
//        }
//        else if((annulus_L_memory == 5))
//        {
//           l_speed_aim = speed_type.cirque-2 ;
//           error = l_speed_aim-l_speed_now;
//        }
//        else
//        {
//           l_speed_aim = speed_type.cirque;
//           error = l_speed_aim-l_speed_now;
//        }
//    }
//
//    //斜十字速度偏差计算
//    else if(road_type.L_Cross || road_type.R_Cross)
//    {
//        if(xieshizi_stage == 1)
//        {
//            l_speed_aim = speed_type.lean_cross;
//            error = l_speed_aim-l_speed_now;
//        }
//        else if(xieshizi_stage == 2)
//        {
//            l_speed_aim = speed_type.lean_cross + 70;
//            error = l_speed_aim-l_speed_now;
//        }
//        else if(xieshizi_stage == 3)
//         {
//             l_speed_aim = speed_type.lean_cross - 80;
//             error = l_speed_aim-l_speed_now;
//         }
//    }
            if(zebra_m>=8000)
    {
        DriveMotor_Ctrl( 0  , 0 );
        l_speed_aim = 0;
        error = l_speed_aim-l_speed_now;
    }
    else
    {
                 l_speed_aim = 32;
                 error = l_speed_aim-l_speed_now;
    }
//    //坡度速度偏差计算
//    else if(2 == pass_barn)
//    {
//        l_speed_aim = speed_type.barn;
//        error = l_speed_aim-l_speed_now;
//    }
//    else if(flag.stop)
//    {
//        l_speed_aim = speed_type.stop;
//        error = l_speed_aim-l_speed_now;
//    }
//
    return error;
}

/***********************************************
函数名：R_speed_error
功  能：根据赛道类型选择不同的右轮速度并计算速度偏差
参  数：void
返回值：int16
 ***********************************************/

int16 R_speed_error(void)
{
    int16 error = 0;

//    //直道速度偏差计算

//        if(encoder.encodersum_average  <7500 && encoder.encodersum_average >500)
//        {
//          r_speed_aim = -50;
//          eulerAngle.pitch=-18;
//          error = r_speed_aim-r_speed_now;
//        }
//        else if(eulerAngle.pitch>= 8)
//           {
//               r_speed_aim = 50;
//                error = r_speed_aim-r_speed_now;
//           }



//    else if(road_type.straight)
//    {
//        r_speed_aim = speed_type.straight;
//        error = r_speed_aim-r_speed_now;
//    }
//    //弯道速度偏差计算
//    else if(road_type.R_bend || road_type.L_bend)
//    {
//        r_speed_aim = speed_type.bend;
//        error = r_speed_aim-r_speed_now;
//    }
//    //十字速度偏差计算
//    else if(road_type.Cross)
//    {
//        r_speed_aim = speed_type.cross;
//        error = r_speed_aim-r_speed_now;
//    }
//
//    //环岛速度偏差计算

//    else
//    if(road_type.LeftCirque)
//    {
//        if(annulus_L_memory == 3)
//        {
//            r_speed_aim = speed_type.cirque -2;
//            error = r_speed_aim-r_speed_now;
//        }
//        else if((annulus_L_memory == 4))
//        {
//            r_speed_aim = speed_type.cirque+2 ;
//            error = r_speed_aim-r_speed_now;
//        }
//        else if((annulus_L_memory == 5))
//        {
//           r_speed_aim = speed_type.cirque-2 ;
//           error = r_speed_aim-r_speed_now;
//        }
//        else
//        {
//           r_speed_aim = speed_type.cirque;
//           error = r_speed_aim-r_speed_now;
//        }
//    }
//    //斜十字速度偏差计算
//    else if(road_type.L_Cross || road_type.R_Cross)
//    {
//        if(xieshizi_stage == 1)
//        {
//            r_speed_aim = speed_type.lean_cross;
//            error = r_speed_aim-r_speed_now;
//        }
//        else if(xieshizi_stage == 2)
//        {
//            r_speed_aim = speed_type.lean_cross + 70;
//            error = r_speed_aim-r_speed_now;
//        }
//        else if(xieshizi_stage == 3)
//         {
//             r_speed_aim = speed_type.lean_cross - 80;
//             error = r_speed_aim-r_speed_now;
//         }
//    }
//    //坡度速度偏差计算
//    else if(2 == pass_barn)
//    {
//        r_speed_aim = speed_type.barn;
//        error = r_speed_aim-r_speed_now;
//    }
//    else if(flag.stop)
//    {
//        r_speed_aim = speed_type.stop;
//        error = r_speed_aim-r_speed_now;
//    }
        if(zebra_m>=8000)
        {
            DriveMotor_Ctrl( 0  , 0 );
            r_speed_aim = 0;
            error = r_speed_aim-r_speed_now;
        }
        else
        {
            r_speed_aim = 32;

            error = r_speed_aim-r_speed_now;
        }
    return error;
}

/**********************************************
函数名：L_speed_pid
功  能：运用PID的增量式控制小车左轮的速度
参  数：struct PID *sptr
返回值：void
 **********************************************/

void L_speed_pid(PID_INFO *pid_info,float *PID_Parm)
{

    pid_info->iError = L_speed_error();//左轮速度偏差计算
    L_pid_out += PID_Parm[KP]*(pid_info->iError-pid_info->LastError)+PID_Parm[KI]*pid_info->iError+PID_Parm[KD]*(pid_info->iError-2*pid_info->LastError+pid_info->PrevError);
    L_pid_out=limit(L_pid_out, 5000);
    pid_info->PrevError = pid_info->LastError;
    pid_info->LastError = pid_info->iError;
}

/**********************************************
函数名：R_speed_pid
功  能：运用PID的增量式控制小车右轮的速度
参  数：struct PID *sptr
返回值：void
 **********************************************/

void R_speed_pid(PID_INFO *pid_info,float *PID_Parm)
{

    pid_info->iError = R_speed_error();//右轮速度偏差计算
    R_pid_out += PID_Parm[KP]*(pid_info->iError-pid_info->LastError)+PID_Parm[KI]*pid_info->iError+PID_Parm[KD]*(pid_info->iError-2*pid_info->LastError+pid_info->PrevError);
    R_pid_out=limit(R_pid_out,5000);
    pid_info->PrevError = pid_info->LastError;
    pid_info->LastError = pid_info->iError;
}

/*****差速******/

void Speed_difference_calculate(int16 diff_base)
{
    float k_diff = 5;//差速系数

    differential = k_diff*diff_base;
}
//#pragma section all restore


