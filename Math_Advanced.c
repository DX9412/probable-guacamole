/*
 * Math_Advanced.c
 *
 *  Created on: 2023��10��19��
 *      Author: Daydream
 */

#include "zf_common_headfile.h"
#include "math.h"



int16 Filter_first(int16 data, int16 last_data, float k)
{
    int16 result;

    result = (int16)((1-k)*data+k*last_data);

    return result;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ټ��� 1/Sqrt(x)
//  @param      x       ��Ҫ�������������������
//  @return     y       ���
//  @note       ���ټ��� 1/Sqrt(x)  �����ͨSqrt()����Ҫ���ı�See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
//-------------------------------------------------------------------------------------------------------------------
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      a faster varient of atan.  accurate to 6 decimal places for values between -1 ~ 1 but then diverges quickly
//  @param      v       ��Ҫ�����������
//  @return     ���
//  @note       a faster varient of atan.  accurate to 6 decimal places for values between -1 ~ 1 but then diverges quickly
//-------------------------------------------------------------------------------------------------------------------
float fast_atan(float v)
{
    float v2 = v*v;
    return (v*(1.6867629106f + v2*0.4378497304f)/(1.6867633134f + v2));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Լ��ֵ     constrain a value
//  @param      amt
//  @param      low         ���ֵ
//  @param      high        ���ֵ
//  @return     ���
//  @note       Լ��ֵ     constrain a value
//-------------------------------------------------------------------------------------------------------------------
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      Լ��ֵ     constrain a int16_t value
//  @param      amt
//  @param      low         ���ֵ
//  @param      high        ���ֵ
//  @return     ���
//  @note       Լ��ֵ     constrain a int16_t value
//-------------------------------------------------------------------------------------------------------------------
int16 constrain_int16(int16 amt, int16 low, int16 high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      Լ��ֵ     constrain a int32_t value
//  @param      amt
//  @param      low         ���ֵ
//  @param      high        ���ֵ
//  @return     ���
//  @note       Լ��ֵ     constrain a int32_t value
//-------------------------------------------------------------------------------------------------------------------
int32 constrain_int32(int32 amt, int32 low, int32 high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �������뺯��
//  @param      f       ���븡����
//  @return     ȡ��[f]
//  @note       [1.5] = 1  [-1.5] = -2
//-------------------------------------------------------------------------------------------------------------------
int HalfAdjust_Number( float f )
{
    int intPart = (int)f;
    float fractionPart = f - intPart;

    if( fractionPart >= 0.5 )
    {
        if( f >= 0.0 )
        {
            intPart++;
        }
        else
        {
            intPart--;
        }
    }
    return intPart;
}

