#include "Fuzzy.h"
#include <stdio.h>
////////////example////////////
/*
         InitFuzzyMachine(&Fuzzy_Kp, Rule_Kp, -30, 30, -1, 1, 0, 30);
        InitFuzzyMachine(&Fuzzy_Kd, Rule_Kd, -30, 30, -0.5, 0.5, -1500,1500);
          Kp=FuzzyInference(&Fuzzy_Kp, SteerPID_Cam.Error_Now, SteerPID_Cam.Error_Now - SteerPID_Cam.Error_Last);
        Kp=SteerKp_Cam + fabs(Kp);

        Kd=FuzzyInference(&Fuzzy_Kd, SteerPID_Cam.Error_Now, SteerPID_Cam.Error_Now - SteerPID_Cam.Error_Last);
        Kd=SteerKd_Cam+fabs(Kd);

*/
//char Rule_Kp[7][7] ={
//        {4,4,2,2,1,ZO,ZO},  //表1
//        {4,4,2,1,1,ZO,1},
//        {2,2,2,1,ZO,1,1},
//        {2,2,1,ZO,1,2,2},
//        {1,1,ZO,1,1,2,2},
//        {1,ZO,1,2,2,2,4},
//        {ZO,ZO,2,2,2,4,4},
//};
char Rule_Speed[7][7] ={
            //-6 -4 -2  0 2 4 6 EC  E
              {0, 0, 1,1,2,2,3}, //-6
              {0, 1, 1,2,2,3,4}, //-4
              {1, 1, 2,3,4,5,5}, //-2
              {3, 4, 5,5,5,4,3}, //0
              {5, 5, 4,3,2,1,1}, //2
              {4, 3, 2,2,1,1,0}, //4
              {3, 2, 2,1,1,0,0}  //6
};
char Rule_Kp[7][7] = {
    //-6 -4 -2  0 2 4 6 EC  E
      {5, 5, 4,4,3,3,2}, //-6
      {5, 4, 4,3,3,2,1}, //-4
      {4, 4, 3,2,1,0,0}, //-2
      {2, 1, 0,0,0,1,2}, //0
      {0, 0, 1,2,3,4,4}, //2
      {1, 2, 3,3,4,4,5}, //4
      {2, 3, 3,4,4,5,5}  //6
};
char Rule_Ki[7][7]={
        {NB,NB,NM,NM,NS,ZO,ZO},
        {NB,NB,NM,NS,NS,ZO,ZO},
        {NB,NM,NS,NS,ZO,PS,PS},
        {NM,NM,NS,ZO,PS,PM,PM},
        {NM,NS,ZO,PS,PS,PM,PB},
        {ZO,ZO,PS,PS,PM,PB,PB},
        {ZO,ZO,PS,PM,PM,PB,PB},
};
char Rule_Kd[7][7]={
        {PS,NS,NB,NB,NB,NM,PS},
        {PS,NS,NB,NM,NM,NS,ZO},
        {ZO,NS,NM,NM,NS,NS,ZO},
        {ZO,NS,NS,NS,NS,NS,ZO},
        {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
        {PB,NS,PS,PS,PS,PS,PB},
        {PB,PM,PM,PM,PS,PS,PB},
};
FuzzyMachine Fuzzy_Elc_Kp;
FuzzyMachine Fuzzy_Kp;
FuzzyMachine Fuzzy_Kd;
FuzzyMachine Fuzzy_Motor_Kp;
FuzzyMachine Fuzzy_Motor_Ki;
FuzzyMachine Fuzzy_Speed;
void InitFuzzyMachine(FuzzyMachinePtr Fuzzy, char(*Table)[DIFINPUTNUM],
    float InputMin, float InputMax,
    float DifInputMin, float DifInputMax,
    float OutputMin, float OutputMax)
{
    char i=0;
    char j=0;
    int Table_Min=65536;
    int Table_Max=-65536;
    //物理论域赋值
    Fuzzy->InputMin = InputMin;
    Fuzzy->InputMax = InputMax;
    Fuzzy->DifInputMin = DifInputMin;
    Fuzzy->DifInputMax = DifInputMax;
    Fuzzy->OutputMin = OutputMin;
    Fuzzy->OutputMax = OutputMax;
    //初始化各个隶属度函数数组
    for (i = 0;i < INPUTNUM;i++)   //输入量
        Fuzzy->Ui[i] = 0;
    for (i = 0;i < DIFINPUTNUM;i++)//输入量差分
        Fuzzy->Udifi[i] = 0;
    for (i = 0;i < OUTPUTNUM;i++)  //输出量
        Fuzzy->Uo[i] = 0;
    //1.
    //载入模糊表
    Fuzzy->FuzzyTable = Table;
    //2.
    //计算量化因子
    //(2.1)输入量化因子
    Fuzzy->Kei = (PB - NB) /
                 (Fuzzy->InputMax - Fuzzy->InputMin);
    //(2.2)输入差分的量化因子
    Fuzzy->Kedifi = (PB - NB) /
                 (Fuzzy->DifInputMax - Fuzzy->DifInputMin);
    //3.
    //计算比例因子
    //(3.1)先找表中的最大最小量
    for (i = 0;i < INPUTNUM;i++)//输入量
    {
        for (j = 0;j < DIFINPUTNUM;j++)//输入量差分
        {
            if(Fuzzy->FuzzyTable[i][j]>=Table_Max)Table_Max=Fuzzy->FuzzyTable[i][j];
            if(Fuzzy->FuzzyTable[i][j]<=Table_Min)Table_Min=Fuzzy->FuzzyTable[i][j];
        }
    }
    //(3.2)计算
    Fuzzy->Kuo = (Fuzzy->OutputMax - Fuzzy->OutputMin) /
                 (Table_Max-Table_Min);
    //计算偏置
    Fuzzy->InputOffset= (Fuzzy->InputMin + Fuzzy->InputMax) / 2;
    Fuzzy->DifInputOffset= (Fuzzy->DifInputMin + Fuzzy->DifInputMax) / 2;

    //Fuzzy->OutputOffset = (Fuzzy->OutputMin + Fuzzy->OutputMax) / 2;
    //6.10修改
    Fuzzy->OutputOffset =Fuzzy->OutputMin-Table_Min*Fuzzy->Kuo;
}
float FuzzyInference(FuzzyMachinePtr Fuzzy, float Val, float DifVal)
{
    //返回的模糊结果值
    float Result = 0;
    //利用偏置归一化各个量
    float Yi = Val - Fuzzy->InputOffset;
    float Ydifi = DifVal - Fuzzy->DifInputOffset;
    //利用量化因子量化映射到模糊论域
    float Ni = Fuzzy->Kei*Yi;
    float Ndifi = Fuzzy->Kedifi*Ydifi;
    //循环变量
    char i = 0;
    char j = 0;
    //临时变量
    float temp = 0;//重心法的分母
    //计算输入以及其偏差的隶属度

    //输入隶属度
    Fuzzy->Ui[0] = Fuzzy_Ramp(NB, NM, 0, Ni);
    Fuzzy->Ui[1] = Fuzzy_Trianle(NB, NM, NS, Ni);
    Fuzzy->Ui[2] = Fuzzy_Trianle(NM, NS, ZO, Ni);
    Fuzzy->Ui[3] = Fuzzy_Trianle(NS, ZO, PS, Ni);
    Fuzzy->Ui[4] = Fuzzy_Trianle(ZO, PS, PM, Ni);
    Fuzzy->Ui[5] = Fuzzy_Trianle(PS, PM, PB, Ni);
    Fuzzy->Ui[6] = Fuzzy_Ramp(PM, PB, 1, Ni);

    //输入差分隶属度
    Fuzzy->Udifi[0] = Fuzzy_Ramp(NB, NM, 0, Ndifi);
    Fuzzy->Udifi[1] = Fuzzy_Trianle(NB, NM, NS, Ndifi);
    Fuzzy->Udifi[2] = Fuzzy_Trianle(NM, NS, ZO, Ndifi);
    Fuzzy->Udifi[3] = Fuzzy_Trianle(NS, ZO, PS, Ndifi);
    Fuzzy->Udifi[4] = Fuzzy_Trianle(ZO, PS, PM, Ndifi);
    Fuzzy->Udifi[5] = Fuzzy_Trianle(PS, PM, PB, Ndifi);
    Fuzzy->Udifi[6] = Fuzzy_Ramp(PM, PB, 1, Ndifi);

    //开始计算结果
    //方案一:使用伪重心法
    for (i = 0;i < INPUTNUM;i++)
    {
        for (j = 0;j < DIFINPUTNUM;j++)
        {
            if (Fuzzy->Ui[i] != 0 && Fuzzy->Udifi[j] != 0)//如果发现节点被激活
            {
                //重心法计算(此结果在输出模糊论域上)
                Result += Fuzzy->FuzzyTable[i][j] * Fuzzy->Ui[i] * Fuzzy->Udifi[j];
                temp += Fuzzy->Ui[i] * Fuzzy->Udifi[j];
                //测试,记录下输出在不同模糊集上的隶属度
//              Fuzzy->Uo[Fuzzy->FuzzyTable[i][j] + PB - ZO]+= Fuzzy->Ui[i] * Fuzzy->Udifi[j];//PB-ZO是一个偏置,为了让数组下标符合规则
            }
        }
    }
    //方案二:


    //模糊论域->物理论域
    Result /= temp;
    Result = Result * Fuzzy->Kuo + Fuzzy->OutputOffset;
    return Result;
}


static float Fuzzy_Trianle(float Left, float Top, float Right, float Input)
{
    float Value=0;//隶属度
    if (Input < Left)Value = 0;
    else if (Left <= Input && Input < Top)Value = (Input - Left) / (Top - Left);
    else if (Top <= Input && Input < Right)Value = (Right - Input) / (Right - Top);
    else Value = 0;
    return Value;
}
static float Fuzzy_Ramp(float Left, float Right, bool Mode, float Input)
{
    float Value = 0;//隶属度
    if (!Mode)//左斜坡
    {
        if (Input < Left)Value = 1;
        else if (Left <= Input && Input < Right)Value = (Right - Input) / (Right - Left);
        else Value = 0;
    }else//右斜坡
    {
        if (Input < Left)Value = 0;
        else if (Left <= Input && Input < Right)Value = (Input-Left) / (Right - Left);
        else Value = 1;
    }
    return Value;
}
