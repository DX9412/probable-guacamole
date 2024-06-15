///*
// * Imu.c
// *
// *  Created on: 2023年10月20日
// *      Author: Daydream
// */
//#include "zf_common_headfile.h"
//
//
///*********************************************************************************************************/
///********************************************四元数姿态解算************************************************/
///*********************************************************************************************************/
//#define delta_T     0.005f  //2ms计算一次
//#define M_PI        3.1415926f
//#define alpha    0.3f
//
//float I_ex, I_ey, I_ez;  // 误差积分
//float param_Kp = 0;    // 加速度计的收敛速率比例增益
//float param_Ki = 1;   // 陀螺仪收敛速率的积分增益 0.004
//
//quater_param_t Q_info = {1, 0, 0,0};  // 全局四元数
//euler_param_t eulerAngle;           // 欧拉角
//
//icm_param_t icm_data;
//gyro_param_t GyroOffset;            //零飘
//
//bool GyroOffset_init = 0 ;
//
//float fast_sqrt(float x)
//{
//    float halfx = 0.5f * x;
//    float y = x;
//    long i = *(long *) &y;
//    i = 0x5f3759df - (i >> 1);
//    y = *(float *) &i;
//    y = y * (1.5f - (halfx * y * y));
//    return y;
//}
//
//void gyroOffset_init(void)      /////////陀螺仪零飘初始化
//{
//    GyroOffset.Xdata = 0;
//    GyroOffset.Ydata = 0;
//    GyroOffset.Zdata = 0;
//    for (uint16_t i = 0; i < 100; ++i)
//    {
//        imu660ra_get_acc();
//        imu660ra_get_gyro();
//        GyroOffset.Xdata += imu660ra_gyro_x;
//        GyroOffset.Ydata += imu660ra_gyro_y;
//        GyroOffset.Zdata += imu660ra_gyro_z;
//        system_delay_ms(10);
//    }
//    GyroOffset.Xdata /= 100;
//    GyroOffset.Ydata /= 100;
//    GyroOffset.Zdata /= 100;
//    GyroOffset_init = 1;
//}
//
//
////转化为实际物理值
//void ICM_getValues()
//{
//    //一阶低通滤波，单位g/s
//    icm_data.acc_x = (((float) imu660ra_acc_x) * alpha) * 8 / 4096 + icm_data.acc_x * (1 - alpha);
//    icm_data.acc_y = (((float) imu660ra_acc_y) * alpha) * 8 / 4096 + icm_data.acc_y * (1 - alpha);
//    icm_data.acc_z = (((float) imu660ra_acc_z) * alpha) * 8 / 4096 + icm_data.acc_z * (1 - alpha);
//
//
//    //陀螺仪角度转弧度
//    icm_data.gyro_x = ((float) imu660ra_gyro_x - GyroOffset.Xdata) * M_PI / 180 / 16.4f;
//    icm_data.gyro_y = ((float) imu660ra_gyro_y - GyroOffset.Ydata) * M_PI / 180 / 16.4f;
//    icm_data.gyro_z = ((float) imu660ra_gyro_z - GyroOffset.Zdata) * M_PI / 180 / 16.4f;
//}
//
//
////互补滤波
//void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az)
//{
//    float halfT = 0.5 * delta_T;
//    float vx, vy, vz;    //当前的机体坐标系上的重力单位向量
//    float ex, ey, ez;    //四元数计算值与加速度计测量值的误差
//    float q0 = Q_info.q0;
//    float q1 = Q_info.q1;
//    float q2 = Q_info.q2;
//    float q3 = Q_info.q3;
//    float q0q0 = q0 * q0;
//    float q0q1 = q0 * q1;
//    float q0q2 = q0 * q2;
////    float q0q3 = q0 * q3;     //未使用 消警告
//    float q1q1 = q1 * q1;
////    float q1q2 = q1 * q2;     //未使用 消警告
//    float q1q3 = q1 * q3;
//    float q2q2 = q2 * q2;
//    float q2q3 = q2 * q3;
//    float q3q3 = q3 * q3;
//    // float delta_2 = 0;
//
//    //对加速度数据进行归一化 得到单位加速度
//    float norm = fast_sqrt(ax * ax + ay * ay + az * az);
//    ax = ax * norm;
//    ay = ay * norm;
//    az = az * norm;
//
//    //根据当前四元数的姿态值来估算出各重力分量。用于和加速计实际测量出来的各重力分量进行对比，从而实现对四轴姿态的修正
//    vx = 2 * (q1q3 - q0q2);
//    vy = 2 * (q0q1 + q2q3);
//    vz = q0q0 - q1q1 - q2q2 + q3q3;
//    //vz = (q0*q0-0.5f+q3 * q3) * 2;
//
//    //叉积来计算估算的重力和实际测量的重力这两个重力向量之间的误差。
//    ex = ay * vz - az * vy;
//    ey = az * vx - ax * vz;
//    ez = ax * vy - ay * vx;
//
//    //用叉乘误差来做PI修正陀螺零偏，
//    //通过调节 param_Kp，param_Ki 两个参数，
//    //可以控制加速度计修正陀螺仪积分姿态的速度。
//    I_ex += halfT * ex;   // integral error scaled by Ki
//    I_ey += halfT * ey;
//    I_ez += halfT * ez;
//
//    gx = gx + param_Kp * ex + param_Ki * I_ex;
//    gy = gy + param_Kp * ey + param_Ki * I_ey;
//    gz = gz + param_Kp * ez + param_Ki * I_ez;
//
//
//    /*数据修正完成，下面是四元数微分方程*/
//
//
//    //四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为陀螺仪角速度，以下都是已知量，这里使用了一阶龙哥库塔求解四元数微分方程
//    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
//    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
//    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
//    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
//    //    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
//    // 整合四元数率    四元数微分方程  四元数更新算法，二阶毕卡法
//    //    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
//    //    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
//    //    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
//    //    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT
//
//
//    // normalise quaternion
//    norm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//    Q_info.q0 = q0 * norm;
//    Q_info.q1 = q1 * norm;
//    Q_info.q2 = q2 * norm;
//    Q_info.q3 = q3 * norm;
//}
//
//
///*把四元数转换成欧拉角*/
//void ICM_getEulerianAngles(void)
//{
//    //采集陀螺仪数据
//    imu660ra_get_acc();
//    imu660ra_get_gyro();
//
//    ICM_getValues();
//    ICM_AHRSupdate(icm_data.gyro_x, icm_data.gyro_y, icm_data.gyro_z, icm_data.acc_x, icm_data.acc_y, icm_data.acc_z);
//    float q0 = Q_info.q0;
//    float q1 = Q_info.q1;
//    float q2 = Q_info.q2;
//    float q3 = Q_info.q3;
//
//    //四元数计算欧拉角
//    eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / M_PI; // pitch
//    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / M_PI; // roll
//    eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI; // yaw
//    eulerAngle.pitch=Limit_Min_Max(eulerAngle.pitch,-20,20);
///*   姿态限制*/
////    if (eulerAngle.roll > 90 || eulerAngle.roll < -90)
////    {
////        if (eulerAngle.pitch > 0)
////            {
////            eulerAngle.pitch = 180 - eulerAngle.pitch;
////            }
////        if (eulerAngle.pitch < 0)
////            {
////            eulerAngle.pitch = -(180 + eulerAngle.pitch);
////            }
////    }
////
////    if (eulerAngle.yaw > 360)
////    {
////        eulerAngle.yaw -= 360;
////    }
////    else if (eulerAngle.yaw < 0)
////    {
////        eulerAngle.yaw += 360;
////    }
//}
//
//
///*********************************************************************************************************/
///***********************************         角速度滤波           *****************************************/
///*********************************************************************************************************/
//float dt = 0.008;
//
//#define I_PI 3.14159265
//
//float acc_ratio  =     2;
//float gyro_ratio = -0.97;
//
//float Q_angle=0.001;//0.001   过程噪声的协方差
//float Q_gyro=0.003; //0.003   过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
//float R_angle=0.5;  //0.5     测量噪声的协方差 既测量偏差
//
//GYRO_VAR gyroscope;
//
////IMU数据采集   与ICM_getEulerianAngles();搭配使用
//void Get_Imu_data(void)
//{
//    imu660ra_get_acc();
//    imu660ra_get_gyro();
//
//    gyroscope.Acc_fiter_X = gyroscope.Acceleration_X =imu660ra_acc_x;   //赋值
//    gyroscope.Acc_fiter_Y = gyroscope.Acceleration_Y =-imu660ra_acc_y;
//    gyroscope.Acc_fiter_Z = gyroscope.Acceleration_Z =imu660ra_acc_z;
//
//    gyroscope.Gyro_Balance = imu660ra_gyro_x/16.4-0.5;                  //翻滚角
//    //给串级PID 角速度
//    gyroscope.Gyro_fiter_Balance = gyroscope.Gyro_Balance;              //
//
//    gyroscope.Gyro_Turn=imu660ra_gyro_z/16.4;                           //偏航角
//    gyroscope.Gyro_Up=imu660ra_gyro_y/16.4;                             //俯仰角
//
//}
//
////未完
////float angle_calc(float angle_m,float gyro_m,uint8 _switch)
////{
////    float temp_angle;
////    float gyro_now;
////    float error_angle;
////
////    static float last_angle_1,last_angle_2,last_angle_3;
////    static unsigned char first_angle_1,first_angle_2,first_angle_3;
////
////
////
////
////}
//
//
//// *  name:  Kalman_Filter_x
//// *  para:   Accel: 加速度算出的角度
////            Gyro: 陀螺仪的角速度
//// *  return: NONE
//// *  writer: LYF
//// *  function:获取x轴角度简易卡尔曼滤波
//// *  time:   2023/10/27
//float angle, angle_dot;
//char  C_0 = 1;
//float Q_bias, Angle_err;
//float PCt_0, PCt_1, E;
//float K_0, K_1, t_0, t_1;
//float Pdot[4] ={0,0,0,0};
//float PP[2][2] = { { 1, 0 },{ 0, 1 } };
//float Kalman_Filter(float Accel,float Gyro)
//{
//    angle+=(Gyro - Q_bias) * dt; //先验估计
//    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; //  Pk-先验估计误差协方差的微分
//    Pdot[1]=-PP[1][1];
//    Pdot[2]=-PP[1][1];
//    Pdot[3]=Q_gyro;
//    PP[0][0] += Pdot[0] * dt;   //Pk-先验估计误差协方差微分的积分
//    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
//    PP[1][0] += Pdot[2] * dt;
//    PP[1][1] += Pdot[3] * dt;
//
//    Angle_err = Accel - angle;  //zk-先验估计
//
//    PCt_0 = C_0 * PP[0][0];
//    PCt_1 = C_0 * PP[1][0];
//
//    E = R_angle + C_0 * PCt_0;
//
//    K_0 = PCt_0 / E;
//    K_1 = PCt_1 / E;
//
//    t_0 = PCt_0;
//    t_1 = C_0 * PP[0][1];
//
//    PP[0][0] -= K_0 * t_0;       //后验估计误差协方差
//    PP[0][1] -= K_0 * t_1;
//    PP[1][0] -= K_1 * t_0;
//    PP[1][1] -= K_1 * t_1;
//
//    angle   += K_0 * Angle_err;  //后验估计
//    Q_bias  += K_1 * Angle_err;  //后验估计
//    angle_dot   = Gyro - Q_bias;     //输出值(后验估计)的微分=角速度
//    return -angle;
//}
//
//
//// 姿态解算
//void Get_Angle(void)
//{
//    gyroscope.Acceleration_Angle=
//                    atan2(gyroscope.Acc_fiter_Y,sqrt(gyroscope.Acc_fiter_Y*gyroscope.Acc_fiter_Y+gyroscope.Acc_fiter_Z*gyroscope.Acc_fiter_Z))*180/I_PI;
//
//    gyroscope.Acceleration_Angle_up=
//                    atan2(gyroscope.Acc_fiter_X,sqrt(gyroscope.Acc_fiter_Z*gyroscope.Acc_fiter_Z+gyroscope.Acc_fiter_X*gyroscope.Acc_fiter_X))*180/I_PI;
//
//
////    if(way==1)
////    {
////        gyroscope.Balance_Angle = angle_calc(gyroscope.Acceleration_Angle           ,gyroscope.Gyro_fiter_Balance,1);
////    }
////    else
////    {
////        gyroscope.Protect_Angle = angle_calc(gyroscope.Acceleration_Angle,gyroscope.Gyro_fiter_Balance,1);
////        gyroscope.Balance_Angle = Kalman_Filter(gyroscope.Acceleration_Angle,-gyroscope.Gyro_fiter_Balance);
////        gyroscope.Up_Angle      = angle_calc(gyroscope.Acceleration_Angle_up ,gyroscope.Gyro_Up,3);
////    }
//
//}
//
//
//
//
////-------------------------------------------------------------------------------------------------------------------
////  @brief      角速度均值滤波
////  @param      滤波次数
////  @return     角速度
////  @note       用于角速度积分
////-------------------------------------------------------------------------------------------------------------------
//float gyro_z_speed(uint8 count)
//{
//    uint8 i;
//    float gyro_speed;
//    float medium=0;
//    for(i=0;i<count;i++)
//    {
//        imu660ra_get_gyro();
//        medium+=(float)imu660ra_gyro_z;
//    }
//    gyro_speed=-((float)medium/((float)count*16.40)); //16.4
//    if(gyro_speed<0)
//        gyro_speed*=1.1; //????????
//    if(gyro_speed>0)
//        gyro_speed*=1.1;
//    if(gyro_speed<1 && gyro_speed>-1)
//        gyro_speed=0;
////    gyro_speed=-(float)((int16)((gyro_speed+0.005)*100))/100.00;
//    return gyro_speed;
//}
//
//
//
//
//
//
