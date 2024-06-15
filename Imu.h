///*
// * Imu.h
// *
// *  Created on: 2023年10月20日
// *      Author: Daydream
// */
//
//#ifndef CODE_CONTROL_IMU_H_
//#define CODE_CONTROL_IMU_H_
//#define Limit_Min_Max(data, min, max) (((data) > (max)) ? (max):(((data) < (min)) ? (min):(data)))
//
//#include "zf_common_headfile.h"
//
//
//typedef struct{                 //计算角度，角加速度 供PID使用
//     float Up_Angle;
//     float Turn_Angle;
//     float Balance_Angle;       // 翻滚角度 Roll
//     float Protect_Angle;
//     float Gyro_Balance;        // 翻滚角度速度
//     float Gyro_fiter_Balance;  // 翻滚角度速度滤波值
//     float Gyro_Turn;           // 航向角速度
//     float Gyro_Up;             // 俯仰角速度
//     float Acceleration_Z;      // Z轴加速度
//     float Acceleration_X;      // X轴加速度
//     float Acceleration_Y;      // Y轴加速度
//     float Acc_fiter_X;         // X轴加速度计滤波值
//     float Acc_fiter_Y;         // Y轴加速度计滤波值
//     float Acc_fiter_Z;         // Z轴加速度计滤波值
//     float Acceleration_Angle;  // 角度解算过程值
//     float Acceleration_Angle_turn;
//     float Acceleration_Angle_up;
//} GYRO_VAR;
//
//typedef struct{
//    float acc_x;    //x轴加速度
//    float acc_y;
//    float acc_z;
//
//    float gyro_x;   //x轴角速度
//    float gyro_y;
//    float gyro_z;
//} icm_param_t;
//
//typedef struct{
//    float Xdata;    //零飘参数X
//    float Ydata;
//    float Zdata;
//} gyro_param_t;
//
//typedef struct {
//    float pitch;    //俯仰角
//    float roll;     //偏航角
//    float yaw;      //翻滚角
//} euler_param_t;
//
//typedef struct {    //四元数
//    float q0;
//    float q1;
//    float q2;
//    float q3;
//} quater_param_t;
//
//extern bool GyroOffset_init;
//
//extern euler_param_t eulerAngle;
//
//void gyroOffset_init(void);
//
//float fast_sqrt(float x);
//
//void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);
//
//void ICM_getValues();
//
//void ICM_getEulerianAngles(void);
//
//
//
//float gyro_z_speed(uint8 count);
//
//
//
//
//#endif /* CODE_CONTROL_IMU_H_ */
