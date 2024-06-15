/*
 * interrupt.c
 *
 *  Created on: 2023年11月20日
 *      Author: Richard Qian
 */
#include "zf_common_headfile.h"


/********************参数定义********************/
int16 steerpwm_output = 0;      // PID计算舵机PWM输出
float motorpwm_output = 0;   // 左右电机PWM输出

/********************参数定义********************/


/********************中断函数********************/
void task_1ms(void)
{
//      dpid.kp = 1.8;
//      dpid.ki = 0;
//      dpid.kd =0;
  dpid.kp=fabs( FuzzyInference( &Fuzzy_Kp , E1 , EC1 ) );
  Direction_pid(&dpid);
}


void task_2ms(void)
{

  Encoder_Integral(&encoder); //编码器积分



  switch( steermotor_enable )
                     {
                         // 舵机停车控制
                         case 0:
                             SteerMotor_Ctrl( 0 );
                         break;
                         // 舵机正常控制
                         case 1:
                             // 摄像头误差左偏误差正右偏误差负，舵机PWM左大右小
                             steerpwm_output = limit( dpid.out , 450 );
                             SteerMotor_Ctrl( steerpwm_output );
                         break;
                         // 舵机左打满
                         case 2:
//                             SteerMotor_Ctrl( 450 );
                             SteerMotor_Ctrl( 420 );
                         break;
                         // 舵机右打满
                         case 3:
//                             SteerMotor_Ctrl( -450 );
                             SteerMotor_Ctrl( -420 );
                         break;
                     }


}
void task_4ms(void)
{


//    Speed_difference_calculate(dpid.out);//差速计算
    L_speed_pid(&pid_type.PID_l,l_speedpid_param);
    R_speed_pid(&pid_type.PID_r,r_speedpid_param);
    DriveMotor_Ctrl( L_pid_out-differential  , R_pid_out+differential );

}


void task_8ms(void)
{
//    ICM_getEulerianAngles();
    get_eulerAngle();

}

//
void task_10ms(void)
{
//    Fuzzy_pid(E1, EC1);

    time_count++;
//    Fuzzy_pid(E1, EC1);
//    dpid.kp = delt_p;
//    dpid.ki = 0;
//    dpid.kd =delt_d;

////       Fuzzy_pid(E1, EC1);
//       dpid.kp = 2.2;
//       dpid.ki = 0;
//       dpid.kd =0;
//       Direction_pid(&dpid);

//       steerpid_param[KP]=delt_p;
//       steerpid_param[KD]=delt_d;

}


void task_20ms(void)
{

//    Encoder_Integral(&encoder); //编码器积分
//
//        if(zebra_m>7000)
//        {
//            DriveMotor_Ctrl(-5000,-5000);
//            DriveMotor_Ctrl(-1000,-1000);
//        }
//    else
//        {
//         if(road_type.LeftCirque==1)
//        {
//
//            if(annulus_L_memory==1)
//            {
//                DriveMotor_Ctrl(2500,2500);
//            }
//            else if(annulus_L_memory==2)
//            {
//                DriveMotor_Ctrl(2400,2450);
//            }
//            else if(annulus_L_memory==3)
//            {
//                DriveMotor_Ctrl(2100,2300);
//            }
//            else if(annulus_L_memory==4)
//            {
//                DriveMotor_Ctrl(2600,2600);
//            }
//            else if(annulus_L_memory==5||annulus_L_memory==6)
//            {
//                DriveMotor_Ctrl(2200,2350);
//            }
//            else if(annulus_L_memory==7)
//            {
//                DriveMotor_Ctrl(2500,2500);
//            }
//        }
//            else if(road_type.RightCirque==1)
//        {
//            if(annulus_R_Flag==1)
//            {
//                DriveMotor_Ctrl(2500,2500);
//            }
//            else if(annulus_R_Flag==2)
//            {
//                DriveMotor_Ctrl(2400,2400);
//            }
//            else if(annulus_R_Flag==3)
//            {
//                DriveMotor_Ctrl(2250,2100);
//            }
//            else if(annulus_R_Flag==4)
//            {
//                DriveMotor_Ctrl(2600,2500);
//            }
//            else if(annulus_R_Flag==5||annulus_R_Flag==6)
//            {
//                DriveMotor_Ctrl(2250,2100);
//            }
//            else if(annulus_R_Flag==7)
//            {
//                DriveMotor_Ctrl(2500,2550);
//            }
//        }
//        else if( road_type.Cross == 1)
//        {
//            if(Crossroad_memory==4)
//            {
//                DriveMotor_Ctrl(2300,2300);
//            }
//            else
//                DriveMotor_Ctrl(2500,2500);
//        }
//        else if(road_type.Ramp1==1)
//        {
//            if(Ramp_memory==1)
//            {
//                 DriveMotor_Ctrl(2500*k_spped,2500*k_spped);
//            }
//            if(Ramp_memory==2)
//            {
//                if(Ramp_m<4000)
//                {
//                    DriveMotor_Ctrl(3500,3500);
//                }
//                else if(Ramp_m>4000)
//                {
//                    DriveMotor_Ctrl(-5000,-5000);
//                    DriveMotor_Ctrl(2400,2400);
//                }
//            }
//        }
//        else
//            {
//                DriveMotor_Ctrl(2600,2600);
//            }
//        }
}
/********************中断函数********************/
