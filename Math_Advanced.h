/*
 * Math_Advanced.h
 *
 *  Created on: 2023年10月19日
 *      Author: Daydream
 */

#ifndef CODE_COMMUNALITY_MATH_ADVANCED_H_
#define CODE_COMMUNALITY_MATH_ADVANCED_H_



#include "zf_common_headfile.h"



#define limit(x, y)     ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))    // 限幅 数据范围是 [-32768,32767]
#define max(a,b)        ((a)>(b)?(a):(b))
#define min(a,b)        ((a)<(b)?(a):(b))


// 函数声明
float invSqrt(float x);                                     // 快速计算 1/Sqrt(x)
float fast_atan(float v);                                   // 求反正切
float constrain_float(float amt, float low, float high);    // 约束值  float
int16 constrain_int16(int16 amt, int16 low, int16 high);    // 约束值  int16
int32 constrain_int32(int32 amt, int32 low, int32 high);    // 约束值  in32
int HalfAdjust_Number( float f );                           // 四舍五入函数
int16 Filter_first(int16 data, int16 last_data, float k);

#endif /* CODE_COMMUNALITY_MATH_ADVANCED_H_ */



