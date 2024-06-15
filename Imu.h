///*
// * Imu.h
// *
// *  Created on: 2023��10��20��
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
//typedef struct{                 //����Ƕȣ��Ǽ��ٶ� ��PIDʹ��
//     float Up_Angle;
//     float Turn_Angle;
//     float Balance_Angle;       // �����Ƕ� Roll
//     float Protect_Angle;
//     float Gyro_Balance;        // �����Ƕ��ٶ�
//     float Gyro_fiter_Balance;  // �����Ƕ��ٶ��˲�ֵ
//     float Gyro_Turn;           // ������ٶ�
//     float Gyro_Up;             // �������ٶ�
//     float Acceleration_Z;      // Z����ٶ�
//     float Acceleration_X;      // X����ٶ�
//     float Acceleration_Y;      // Y����ٶ�
//     float Acc_fiter_X;         // X����ٶȼ��˲�ֵ
//     float Acc_fiter_Y;         // Y����ٶȼ��˲�ֵ
//     float Acc_fiter_Z;         // Z����ٶȼ��˲�ֵ
//     float Acceleration_Angle;  // �ǶȽ������ֵ
//     float Acceleration_Angle_turn;
//     float Acceleration_Angle_up;
//} GYRO_VAR;
//
//typedef struct{
//    float acc_x;    //x����ٶ�
//    float acc_y;
//    float acc_z;
//
//    float gyro_x;   //x����ٶ�
//    float gyro_y;
//    float gyro_z;
//} icm_param_t;
//
//typedef struct{
//    float Xdata;    //��Ʈ����X
//    float Ydata;
//    float Zdata;
//} gyro_param_t;
//
//typedef struct {
//    float pitch;    //������
//    float roll;     //ƫ����
//    float yaw;      //������
//} euler_param_t;
//
//typedef struct {    //��Ԫ��
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
