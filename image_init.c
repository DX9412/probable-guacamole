#include "zf_common_headfile.h"

double Mat1[3][3]=       { { 0.728070175438597, -0.455912533831805, 8.84641626263216},
                 { 3.44951982839683E-17, 0.134139455186246, 8.13887103997508},
                 { 2.43952476359639E-19, -0.00517164164573476, 0.686212200868431}, };


double Mat2[3][3]= { { 1.37349397590361, 2.73493975903614, -50.1445783132529},
                 { -2.22044604925031E-16, 5.1156626506024, -60.6746987951806},
                 { -2.1617254558436E-18, 0.0385542168674698, 1}, };

//输出的结果
double Tx=0;
double Ty=0;


void Transform_Point1(int x, int y) {
  // 将二维坐标转换为三维齐次坐标
        double w = 1;

        // 与变换矩阵相乘
        double transformedX = Mat1[0][ 0] * x + Mat1[0][ 1] * y + Mat1[0][ 2] * w;
        double transformedY = Mat1[1][ 0] * x + Mat1[1][ 1] * y + Mat1[1][ 2] * w;
        double transformedW = Mat1[2][ 0] * x + Mat1[2][ 1] * y + Mat1[2][2] * w;

        // 将得到的三维齐次坐标转换回二维坐标
        if (transformedW != 0)
        {
            transformedX /= transformedW;
            transformedY /= transformedW;
        }
   Tx=transformedX;
   Ty=transformedY;
}

// 函数用于坐标转换
void Transform_Point2(int x, int y) {
  // 将二维坐标转换为三维齐次坐标
        double w = 1;

        // 与变换矩阵相乘
        double transformedX = Mat2[0][ 0] * x + Mat2[0][1] * y + Mat2[0][2] * w;
        double transformedY = Mat2[1][ 0] * x + Mat2[1][ 1] * y + Mat2[1][ 2] * w;
        double transformedW = Mat2[2][ 0] * x + Mat2[2][ 1] * y + Mat2[2][2] * w;

        // 将得到的三维齐次坐标转换回二维坐标
        if (transformedW != 0)
        {
            transformedX /= transformedW;
            transformedY /= transformedW;
        }
   Tx=transformedX;
   Ty=transformedY;
}
static int src_w=140;//输出图像的宽
static int src_h=100;//输出图像的高

//边界数组

//边线逆透视
void Transform_Line(int in[][2],float out[][2],int *in_num,int *out_num)
{
    int step=0;
    for(int i=0;i< *in_num;i++){
        Transform_Point2(in[i][0],in[i][1]);
        if(Tx>=0&&Tx<src_w&&Ty>=0&&Ty<src_h)
        {

            out[step][0]=Tx;
            out[step][1]=Ty;
            step++;
//            ips200_draw_point               (Tx, Ty, RGB565_BLACK);
        }
    }
    *out_num = step;

}


