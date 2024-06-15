/*
 * FUZZY.h
 *
 *  Created on: 2022楠烇拷11閺堬拷5閺冿拷
 *      Author: 2879058780
 */
#ifndef FUZZY_H
#define FUZZY_H

#include "zf_common_headfile.h"

/****************************************************************************
                    鍗ゆ埉鑴涙嫝娼炲獟鑴ヨ劃鑴岄搯绂勭叅鑴㈤箍鑴剻闇查晛鑴﹀崲鑴涙嫝娼炲獟鍗ら搯闄嗛叾鑴ㄨ劏鑴ヨ劃鑴岄搯
                          鑴涙嫝娼炲獟绂勭倝鑴㈤箍鑴剻鑴犲獟闄嗚劅鑴曟ゼ鑴箞娼炵倝鑴㈠獟
                         鑴熸皳鑴︾叅绂勭倝鑴剻鑴拌劥鑴ㄨ剾璺瘬闄嗛叾鑴ㄨ劏鑴熸皳鑴︾叅绂勭倝
*****************************************************************************/

//#include "common.h"
//#include "stdlib.h"
#define InputMod 0 //输入模式(-1:只映射到NB-ZO部分，0:映射到全部论域，1：只映射到ZO-PB部分)
#define OutputMod  //输出模式(同上)
//宏定义部分
//1.模糊词汇的定义(这里按照7个模糊词给出)
#define NS -1 //负小
#define NM -2 //负中
#define NB -3 //负大
#define ZO 0  //刚好(0)
#define PS 1  //正小
#define PM 2  //正中
#define PB 3  //正大
//2.模糊表的大小(此版本暂不支持修改,还没想到怎么做出一个普适的效果)
#define OUTPUTNUM 7
#if InputMod==-1||InputMod==1
    #define INPUTNUM    (4)
    #define DIFINPUTNUM (4)
#else
    #define INPUTNUM    (7)
    #define DIFINPUTNUM (7)
#endif
/*
   模糊表请用如下二维数组方式给出
    THIS IS FUZZY TABLE EXAMPLE：

   FuzzyTable[INPUTNUM][DIFINPUTNUM]
          D I F I N P U T
       — — — — — — —  Num
      |A1 A2 A3 A4 A5 A6 A7
    I |B1 B2 B3 B4 B5 B6 B7
    N |C1 C2 C3 C4 C5 C6 C7
    P |D1 D2 D3 D4 D5 D6 D7
    U |E1 E2 E3 E4 E5 E6 E7
    T |F1 F2 F3 F4 F5 F6 F7
      |G1 G2 G3 G4 G5 G6 G7
    Num
     其中(A-G)(1-7)表示的模糊规则
*/
//结构体创建部分
//模糊推理机结构体
typedef struct
{
    //输入部分
    float InputMax;                  //输入最大
    float InputMin;                  //输入最小
    float DifInputMax;               //输入差分最大
    float DifInputMin;               //输入差分最小
    float Ui[INPUTNUM];              //输入量对每个模糊集的隶属函数值数组(顺序是NB-PB)
    float Udifi[DIFINPUTNUM];        //输入差分对每个模糊集的隶属函数数组(顺序同上)
    float Kei;                       //输入的量化因子   物理论域->模糊论域
    float Kedifi;                    //输入差分的量化因子
    float InputOffset;               //输入偏置
    float DifInputOffset;            //输入差分的偏置
    //输出部分
    float OutputMax;         //输出最大
    float OutputMin;         //输出最小
    float Kuo;               //输出的比例因子 模糊论域->物理论域
    float OutputOffset;     //输出的偏置
    float Uo[OUTPUTNUM];    //输出模糊集的隶属度(顺序是NB-PB)
    //模糊表
    char (*FuzzyTable)[DIFINPUTNUM];
}FuzzyMachine,*FuzzyMachinePtr;
//函数部分
/**********************************************************************
函数名: InitFuzzyMachine
功能:   初始化创建出来的模糊推理机
参数:   创建出的模糊推理机的指针,规则表,控制值的范围，控制值的差分的范围
返回值: 无
***********************************************************************/
void InitFuzzyMachine(FuzzyMachinePtr Fuzzy,char (*Table)[DIFINPUTNUM] ,
                      float InputMin, float InputMax,
                      float DifInputMin,float DifInputMax,
                      float OutputMin,float OutputMax);
/**********************************************************************
函数名: FuzzyInference
功能:   对初始化过的结构体进行模糊推理
参数:   模糊推理机指针,控制量当前值，控制量
返回值: 清晰化后的值
***********************************************************************/
float FuzzyInference(FuzzyMachinePtr Fuzzy,float Val,float DifVal);

/**********************************************************************
函数名: Fuzzy_Triangle
功能:   使用三角隶属函数计算某一个点的隶属度
参数:   三角底边左,右点横坐标,顶点横坐标,和计算的那个点横坐标
返回值: 隶属度(0-1)
***********************************************************************/
static float Fuzzy_Trianle(float Left,float Top,float Right,float Input);


/**********************************************************************
函数名: Fuzzy_Ramp   (左斜坡是从左到右下降趋势,右斜坡是从左到右上升趋势)
功能:   使用斜坡形隶属函数计算某一个点的隶属度
参数:   斜坡的左拐点,右拐点横坐标,使用左斜坡还是右斜坡(0是左,1是右),计算坐标的那个点的横坐标
返回值: 隶属度(0-1)
***********************************************************************/
static float Fuzzy_Ramp(float Left, float Right, bool Mode,float Input);

//TEST
extern char Rule_Kp[7][7];
extern char Rule_Ki[7][7];
extern char Rule_Kd[7][7];
extern char Rule_Speed[7][7];
extern FuzzyMachine Fuzzy_Kp;
extern FuzzyMachine Fuzzy_Kd;
extern FuzzyMachine Fuzzy_Motor_Kp;
extern FuzzyMachine Fuzzy_Motor_Ki;
extern FuzzyMachine Fuzzy_Elc_Kp;
extern FuzzyMachine Fuzzy_Speed;

#endif
