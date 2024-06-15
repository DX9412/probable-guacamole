/*
 * imu660.c
 *
 *  Created on: 2024年1月29日
 *      Author: LateRain
 */
#include "zf_common_headfile.h"
//-------------------------------------------------------------------------------------------------------------------
//  @brief      单位转换，数据预处理
//  @param      六轴数据输入
//-------------------------------------------------------------------------------------------------------------------
#define alpha 0.3f
#define M_PI        3.1415926f

imu_param imu660ra_date;

void date_handle()
{
    imu660ra_get_gyro();
    imu660ra_get_acc();
    //RC低通滤波,单位m/s2
    imu660ra_date.acc_x = (((float) imu660ra_acc_x) * alpha) * 9.79 / 4096 + imu660ra_date.acc_x * (1 - alpha);
    imu660ra_date.acc_y = (((float) imu660ra_acc_y) * alpha) * 9.79 / 4096 + imu660ra_date.acc_y * (1 - alpha);
    imu660ra_date.acc_z = (((float) imu660ra_acc_z) * alpha) * 9.79 / 4096 + imu660ra_date.acc_z * (1 - alpha);
    //单位rps
    imu660ra_date.gyro_x = ((float) imu660ra_gyro_x /*- GyroOffset.Xdata*/) * M_PI / 180 / 16.4f;
    imu660ra_date.gyro_y = ((float) imu660ra_gyro_y /*- GyroOffset.Ydata*/) * M_PI / 180 / 16.4f;
    imu660ra_date.gyro_z = ((float) imu660ra_gyro_z /*- GyroOffset.Zdata*/) * M_PI / 180 / 16.4f;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      单位转换，用于控制
//  @param      六轴数据输入
//-------------------------------------------------------------------------------------------------------------------
imu_param imu660ra_date_raw;

void date_raw()
{
    //单位g
    imu660ra_date_raw.acc_x = (float) imu660ra_acc_x;
    imu660ra_date_raw.acc_y = (float) imu660ra_acc_y;
    imu660ra_date_raw.acc_z = (float) imu660ra_acc_z;
    //单位dps
    imu660ra_date_raw.gyro_x = ((float) imu660ra_gyro_x) / 16.4f;
    imu660ra_date_raw.gyro_y = ((float) imu660ra_gyro_y) / 16.4f;
    imu660ra_date_raw.gyro_z = ((float) imu660ra_gyro_z) / 16.4f;
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      四元数滤波
//  @return     float       数据融合后的角度
//-------------------------------------------------------------------------------------------------------------------
#define pi      3.14159265f

float Kp   =   18;//2   0.8
float Ki    =  0.3;//0.01
float halfT =  0.001;

float  q0=1,q1=0,q2=0,q3=0;
float  exInt=0,eyInt=0,ezInt=0;

euler_param offset_angle;
euler_param eulerAngle;

void get_eulerAngle()
{
    date_handle();
    float ax = imu660ra_date.acc_x;
    float ay = imu660ra_date.acc_y;
    float az = imu660ra_date.acc_z;
    float gx = imu660ra_date.gyro_x;
    float gy = imu660ra_date.gyro_y;
    float gz = imu660ra_date.gyro_z;

    float  norm;
    float  vx, vy, vz;
    float  ex, ey, ez;

    float  q00, q11, q22;
    float  q0q0 = q0*q0;
    float  q0q1 = q0*q1;
    float  q0q2 = q0*q2;
    float  q1q1 = q1*q1;
    float  q1q3 = q1*q3;
    float  q2q2 = q2*q2;
    float  q2q3 = q2*q3;
    float  q3q3 = q3*q3;

    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;

    ex = (ay*vz - az*vy) ;
    ey = (az*vx - ax*vz) ;
    ez = (ax*vy - ay*vx) ;

    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;

    q00 = q0;
    q11 = q1;
    q22 = q2;
    q0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
    q1 = q1 + ( q00*gx + q2*gz - q3*gy) * halfT;
    q2 = q2 + ( q00*gy - q11*gz + q3*gx) * halfT;
    q3 = q3 + ( q00*gz + q11*gy - q22*gx) * halfT;

    norm = invSqrt(q0q0 + q1q1 + q2q2 + q3q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    eulerAngle.pitch = asin(2*(q0q2 - q1q3 )) * 57.2957795f + offset_angle.pitch;
    eulerAngle.roll  = asin(2*(q0q1 + q2q3 )) * 57.2957795f + offset_angle.roll;
    eulerAngle.yaw   = Kalman_Filter( eulerAngle.yaw , imu660ra_date_raw.gyro_z );

}

//----------------------------------------------------------------
//  @brief      卡尔曼滤波
//  @param      angle       角度
//  @param      angle_dot   角速度
//  @param      Q_angle     角度数据置信度
//  @param      Q_gyro      角速度数据置信度
//  @return     float       数据融合后的角度
//----------------------------------------------------------------
float dt=0.001;//注意：dt的取值为kalman滤波器采样时间
float angle, angle_dot;//角度和角速度
float P[2][2] = {{ 1, 0 },
                 { 0, 1 }};
float Pdot[4] ={ 0,0,0,0};
float Q_angle=0.00001, Q_gyro=0.000005; //角度数据置信度,角速度数据置信度Q_angle=0.00001, Q_gyro=0.000005
float R_angle=0.5 ,C_0 = 1;
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//angleAx=atan2((ACCEL_XOUT_H / 16384),(ACCEL_ZOUT_H / 16384))*180/3.14;
//卡尔曼滤波
float Kalman_Filter(float angle_m, float gyro_m)//angleAx 和 gyroGy
{
        angle+=(gyro_m-q_bias) * dt;
        angle_err = angle_m - angle;
        Pdot[0]=Q_angle - P[0][1] - P[1][0];
        Pdot[1]=- P[1][1];
        Pdot[2]=- P[1][1];
        Pdot[3]=Q_gyro;
        P[0][0] += Pdot[0] * dt;
        P[0][1] += Pdot[1] * dt;
        P[1][0] += Pdot[2] * dt;
        P[1][1] += Pdot[3] * dt;
        PCt_0 = C_0 * P[0][0];
        PCt_1 = C_0 * P[1][0];
        E = R_angle + C_0 * PCt_0;
        K_0 = PCt_0 / E;
        K_1 = PCt_1 / E;
        t_0 = PCt_0;
        t_1 = C_0 * P[0][1];
        P[0][0] -= K_0 * t_0;
        P[0][1] -= K_0 * t_1;
        P[1][0] -= K_1 * t_0;
        P[1][1] -= K_1 * t_1;
        angle += K_0 * angle_err; //最优角度
        q_bias += K_1 * angle_err;
        angle_dot = gyro_m-q_bias;//最优角速度

        return angle;
}
