/*
 * Math_Advanced.h
 *
 *  Created on: 2023��10��19��
 *      Author: Daydream
 */

#ifndef CODE_COMMUNALITY_MATH_ADVANCED_H_
#define CODE_COMMUNALITY_MATH_ADVANCED_H_



#include "zf_common_headfile.h"



#define limit(x, y)     ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))    // �޷� ���ݷ�Χ�� [-32768,32767]
#define max(a,b)        ((a)>(b)?(a):(b))
#define min(a,b)        ((a)<(b)?(a):(b))


// ��������
float invSqrt(float x);                                     // ���ټ��� 1/Sqrt(x)
float fast_atan(float v);                                   // ������
float constrain_float(float amt, float low, float high);    // Լ��ֵ  float
int16 constrain_int16(int16 amt, int16 low, int16 high);    // Լ��ֵ  int16
int32 constrain_int32(int32 amt, int32 low, int32 high);    // Լ��ֵ  in32
int HalfAdjust_Number( float f );                           // �������뺯��
int16 Filter_first(int16 data, int16 last_data, float k);

#endif /* CODE_COMMUNALITY_MATH_ADVANCED_H_ */



