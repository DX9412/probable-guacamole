/*
 * imu660.h
 *
 *  Created on: 2024��1��29��
 *      Author: LateRain
 */

#ifndef CODE_BALANCE_IMU660_H_
#define CODE_BALANCE_IMU660_H_

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} imu_param;
typedef struct
{
    float pitch;
    float roll;
    float yaw;
} euler_param;

void date_handle();
void date_raw();
void get_eulerAngle();
float Kalman_Filter(float angle_m, float gyro_m);

extern euler_param offset_angle;
extern imu_param imu660ra_date_raw;  //�ɼ�������
extern imu_param imu660ra_date;      //������������
extern euler_param eulerAngle;       //��λ�ǽǶ�

#endif /* CODE_BALANCE_IMU660_H_ */
