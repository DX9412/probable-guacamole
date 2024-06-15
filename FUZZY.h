/*
 * FUZZY.h
 *
 *  Created on: 2022骞�11鏈�5鏃�
 *      Author: 2879058780
 */
#ifndef FUZZY_H
#define FUZZY_H

#include "zf_common_headfile.h"

/****************************************************************************
                    卤戮脛拢潞媒脥脝脌铆禄煤脢鹿脫脙露镁脦卢脛拢潞媒卤铆陆酶脨脨脥脝脌铆
                          脛拢潞媒禄炉脢鹿脫脙脠媒陆脟脕楼脢么潞炉脢媒
                         脟氓脦煤禄炉脫脙脰脴脨脛路篓陆酶脨脨脟氓脦煤禄炉
*****************************************************************************/

//#include "common.h"
//#include "stdlib.h"
#define InputMod 0 //����ģʽ(-1:ֻӳ�䵽NB-ZO���֣�0:ӳ�䵽ȫ������1��ֻӳ�䵽ZO-PB����)
#define OutputMod  //���ģʽ(ͬ��)
//�궨�岿��
//1.ģ���ʻ�Ķ���(���ﰴ��7��ģ���ʸ���)
#define NS -1 //��С
#define NM -2 //����
#define NB -3 //����
#define ZO 0  //�պ�(0)
#define PS 1  //��С
#define PM 2  //����
#define PB 3  //����
//2.ģ����Ĵ�С(�˰汾�ݲ�֧���޸�,��û�뵽��ô����һ�����ʵ�Ч��)
#define OUTPUTNUM 7
#if InputMod==-1||InputMod==1
    #define INPUTNUM    (4)
    #define DIFINPUTNUM (4)
#else
    #define INPUTNUM    (7)
    #define DIFINPUTNUM (7)
#endif
/*
   ģ�����������¶�ά���鷽ʽ����
    THIS IS FUZZY TABLE EXAMPLE��

   FuzzyTable[INPUTNUM][DIFINPUTNUM]
          D I F I N P U T
       �� �� �� �� �� �� ��  Num
      |A1 A2 A3 A4 A5 A6 A7
    I |B1 B2 B3 B4 B5 B6 B7
    N |C1 C2 C3 C4 C5 C6 C7
    P |D1 D2 D3 D4 D5 D6 D7
    U |E1 E2 E3 E4 E5 E6 E7
    T |F1 F2 F3 F4 F5 F6 F7
      |G1 G2 G3 G4 G5 G6 G7
    Num
     ����(A-G)(1-7)��ʾ��ģ������
*/
//�ṹ�崴������
//ģ��������ṹ��
typedef struct
{
    //���벿��
    float InputMax;                  //�������
    float InputMin;                  //������С
    float DifInputMax;               //���������
    float DifInputMin;               //��������С
    float Ui[INPUTNUM];              //��������ÿ��ģ��������������ֵ����(˳����NB-PB)
    float Udifi[DIFINPUTNUM];        //�����ֶ�ÿ��ģ������������������(˳��ͬ��)
    float Kei;                       //�������������   ��������->ģ������
    float Kedifi;                    //�����ֵ���������
    float InputOffset;               //����ƫ��
    float DifInputOffset;            //�����ֵ�ƫ��
    //�������
    float OutputMax;         //������
    float OutputMin;         //�����С
    float Kuo;               //����ı������� ģ������->��������
    float OutputOffset;     //�����ƫ��
    float Uo[OUTPUTNUM];    //���ģ������������(˳����NB-PB)
    //ģ����
    char (*FuzzyTable)[DIFINPUTNUM];
}FuzzyMachine,*FuzzyMachinePtr;
//��������
/**********************************************************************
������: InitFuzzyMachine
����:   ��ʼ������������ģ�������
����:   ��������ģ���������ָ��,�����,����ֵ�ķ�Χ������ֵ�Ĳ�ֵķ�Χ
����ֵ: ��
***********************************************************************/
void InitFuzzyMachine(FuzzyMachinePtr Fuzzy,char (*Table)[DIFINPUTNUM] ,
                      float InputMin, float InputMax,
                      float DifInputMin,float DifInputMax,
                      float OutputMin,float OutputMax);
/**********************************************************************
������: FuzzyInference
����:   �Գ�ʼ�����Ľṹ�����ģ������
����:   ģ�������ָ��,��������ǰֵ��������
����ֵ: ���������ֵ
***********************************************************************/
float FuzzyInference(FuzzyMachinePtr Fuzzy,float Val,float DifVal);

/**********************************************************************
������: Fuzzy_Triangle
����:   ʹ������������������ĳһ�����������
����:   ���ǵױ���,�ҵ������,���������,�ͼ�����Ǹ��������
����ֵ: ������(0-1)
***********************************************************************/
static float Fuzzy_Trianle(float Left,float Top,float Right,float Input);


/**********************************************************************
������: Fuzzy_Ramp   (��б���Ǵ������½�����,��б���Ǵ�������������)
����:   ʹ��б����������������ĳһ�����������
����:   б�µ���յ�,�ҹյ������,ʹ����б�»�����б��(0����,1����),����������Ǹ���ĺ�����
����ֵ: ������(0-1)
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
