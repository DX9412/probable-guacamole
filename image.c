/*
 * image.c
 *
 *  Created on: 2023年8月25日
 *      Author: Charlie
 */
//-------------------------------------------------------------------------------------------------------------------

#include "zf_common_headfile.h"




#define White 255
#define Black 0

#define Left 1
#define Right 2
uint8 Thres=150;//增大白色变多 140
uint8 huandao_stage=0;
uint8 xieshizi_stage=0;



struct ROAD_TYPE road_type = {
        .straight      = 0,
        .L_bend        = 0,
        .R_bend        = 0,
        .Ramp1          = 0,
        .Cross         = 0,
        .L_Cross       = 0,
        .R_Cross       = 0,
        .LeftCirque    = 0,
        .RightCirque   = 0,
        .L_bihznag     = 0,
        .R_bihznag     = 0,
        .zebra_flag    = 0,   //斑马线
        .Barn_l_out    = 0,
        .Barn_r_out    = 0,
        .Barn_l_in     = 0,
        .Barn_r_in     = 0,
        .smalls        = 0,
};


uint8 kuandu_saidao [MT9V03X_H-1] = {
        106, 105, 104, 103, 102, 101, 100, 99, 98, 97,
        96,  95,  94,  93,  92,  91,  90,  89,  88,  87,
        86,  85,  84,  83,  82,  81,  80,  79,  78,  77,
        76,  75,  74,  73,  72,  71,  70,  69,  68,  67,
        66,  65,  64,  63,  62,  61,  60,  59,  58,  57,
        56,  55,  54,  53,  52,  51,  50,  49,  48,  47,
        46,  45,  44,  43,  42,  41,  40,  39,  38,  37,
        35,  33,  31,  29,  27,   25,  23,  21,  19,  17 }; //前80行的赛道宽度

int x,y;
float parameterB,parameterA;   //y=parameterB*x+parameterA
uint8 left[120]={2};
uint8 right[120]={185};
uint8 middle[120]={93};

uint8 left_dispose[120]={2};
uint8 right_dispose[120]={185};


uint8 calkuan[120]={114};
uint8 Endline=1;
int WhiteNum=0;
int X1,Y1;//左下补线点（圆环）

uint8 left_num=0;
uint8 right_num=0;
uint8 right_lost_num=0;
uint8 left_lost_num=0;
uint8 imag[120][188];// 二值化后的图像
uint8 threshold_value=175;
uint32 image_process_time=0;   //图像处理时间

uint8 Right_straight_flag=0; //右直线
uint8 Left_straight_flag=0; //左直线

uint8 annulus_L_Flag=0;       //左圆环
uint8 annulus_R_Flag=0;       //右圆环
uint8 annulus_L_memory=0;     //左圆环计步
uint8 annulus_R_memory=0;     //右圆环计步

uint8 zebra_crossing_flag=0;//斑马线
uint8 Left_garage_flag=0; //左车库
uint8 Left_garage_memory=0;//左车库计步
uint8 zebra_crossing_memory=0;//距离斑马线位置 标志位

uint8 car_stop=0;

uint8 Ramp_memory=0;     //坡道计步

//圆环凸起点
uint8 roundabout_X=0;
uint8 roundabout_Y=0;
uint8 roundabout_Flag=0;

//出环识别点
uint8 Exit_loop_X=0;
uint8 Exit_loop_Y=0;
uint8 Exit_loop_Flag=0;

uint8 Crossroad_Flag=0;      //十字
uint8 Crossroad_memory=0;     //十字计步
uint8 Finish_Flag=0; //处理完成标识位

uint8 L_Crossroad_Flag=0;
uint8 L_Crossroad_memory=0;

//丢线
uint8 Lost_left_Flag=0;
uint8 Lost_right_Flag=0;
uint8 Lost_point_L_scan_line=0;
uint8 Lost_point_R_scan_line=0;

//左下拐点
uint8 Lower_left_inflection_X =0;
uint8 Lower_left_inflection_Y =0;
uint8 Lower_left_inflection_Flag=0;

//右下拐点
uint8 Lower_right_inflection_X =0;
uint8 Lower_right_inflection_Y =0;
uint8 Lower_right_inflection_Flag=0;

//左上拐点
uint8 Upper_left_inflection_X =0;
uint8 Upper_left_inflection_Y =0;
uint8 Upper_left_inflection_Flag=0;

//右上拐点
uint8 Upper_right_inflection_X =0;
uint8 Upper_right_inflection_Y =0;
uint8 Upper_right_inflection_Flag=0;

//左环左下拐点
uint8 Lower_left_huan_X =0;
uint8 Lower_left_huan_Y =0;
uint8 Lower_left_huan_Flag=0;

//坡道斜率119-40      119-80          拟合误差
float LK0end=0,LK=0,RK0end=0,RK=0,kerrorL=0,kerrorR=0;

float absolute(float z)
{
    z = z< 0 ? (-z) : z;
    return z;
}

int16 limit_a_b(int16 x, int a, int b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}


int16 limit1(int16 x, int16 y)
{
    if (x > y)             return y;
    else if (x < -y)       return -y;
    else                return x;
}


uint8 original_image[image_h][image_w];
void Get_image(uint8(*mt9v03x_image)[image_w])
{
#define use_num     1   //1 不压缩，2 压缩一倍
    uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < image_h; i += use_num)          //
    {
        for (j = 0; j <image_w; j += use_num)     //
        {
            original_image[row][line] = mt9v03x_image[i][j];//这里的参数填写你的摄像头采集到的图像
            line++;
        }
        line = 0;
        row++;
    }
}


uint8 OtsuThreshold(uint8 *image, uint16 col, uint16 row)
{

#define GrayScale 256
    uint16 Image_Width  = col;
    uint16 Image_Height = row;
    int X; uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};

    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差;
    uint8 MinValue=0, MaxValue=0;
    uint8 Threshold = 0;


    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; //统计每个灰度值的个数信息
        }
    }




    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //前景像素点数
          PixelFore = Amount - PixelBack;         //背景像素点数
          OmegaBack = (double)PixelBack / Amount;//前景像素百分比
          OmegaFore = (double)PixelFore / Amount;//背景像素百分比
          PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
          MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
          MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//遍历最大的类间方差g
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }

   return Threshold;
}

void binaryzation(void)
{
  uint8 i,j;

threshold_value = OtsuThreshold(original_image[0], image_w, image_h)+5;
  for(i = 0;i<image_h;i++)
  {

      for(j = 0;j<image_w;j++)
      {
          if(original_image[i][j]>threshold_value)imag[i][j] = white_pixel;
          else imag[i][j] = black_pixel;

      }

  }

}


//大津法计算阈值
unsigned char Image_Threshold(unsigned char *image, unsigned short width, unsigned short height, unsigned int image_max_threshold)
{
    int pixelCount[256];
    float pixelPro[256];
    int i, j, pixelSum = width * height;
    unsigned char threshold = 0;
    unsigned char *data = image;    // 指向像素数据的指针
    for (i = 0; i < 256; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    unsigned long gray_sum = 0;
    // 统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i += 1)
    {
        for (j = 0; j < width; j += 1)
        {
            pixelCount[(int)data[i * width + j]]++; // 将当前的点的像素值作为计数数组的下标
            gray_sum += (int)data[i * width + j];   // 灰度值总和
        }
    }
    // 计算每个像素值的点在整幅图像中的比例
    for (i = 0; i < 256; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    // 遍历灰度级[0,255],[0,pixel_threshold]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < image_max_threshold; j++)
    {
        w0 += pixelPro[j];         // 背景部分每个灰度值的像素点所占比例之和 即背景部分的比例
        u0tmp += j * pixelPro[j];  // 背景部分每个灰度值的点的比例*灰度值

        w1 = 1 - w0;
        u1tmp = gray_sum / pixelSum - u0tmp;

        u0 = u0tmp / w0;    // 背景平均灰度
        u1 = u1tmp / w1;    // 前景平均灰度
        u = u0tmp + u1tmp;  // 全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = (unsigned char)j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
    return threshold;
}


uint8 Threshold;  //阈值
uint8 Threshold_static = 160;   //阈值静态下限
uint16 Threshold_detach = 300;  //阳光算法分割阈值(光强越强,该值越大)
//   大津法图像二值化
void Get01change_Dajin(unsigned char *in_image,unsigned char *out_image, unsigned int image_max_threshold)
{
    Threshold = Image_Threshold(in_image, image_w, image_h, image_max_threshold);

    if (Threshold < Threshold_static)
    {
        Threshold = Threshold_static;
    }
    uint8 thre;
    for(uint8 y = 0; y < image_h; y++)
    {
        for(uint8 x = 0; x < image_w; x++)
        {
            if (x <= 30)
                thre = Threshold + 20;
            else if (x >= image_w-35)
                thre = Threshold + 20;
            else
                thre = Threshold;

            if (mt9v03x_image[y][x] >thre)         //数值越大，显示的内容越多，较浅的图像也能显示出来
                imag[y][x] = 255;  //白
            else
                imag[y][x] = 0;  //黑
        }
    }
//    for(uint8 x = 0; x < image_w; x++)
//        {
//        for(uint8 y = 0; y < image_h; y++)
//        {
//            if (y <= 30)
//                thre = Threshold + 20;
//            else
//                thre = Threshold;
//            if (mt9v03x_image[y][x] >thre)         //数值越大，显示的内容越多，较浅的图像也能显示出来
//                imag[y][x] = 255;  //白
//            else
//                imag[y][x] = 0;  //黑
//        }
//    }

}

#define Sobel_Gx(addr, y, x)    (addr[UP][RIGHT]+2*addr[y][RIGHT]+addr[DOWN][RIGHT]-(addr[UP][LEFT]+2*addr[y][LEFT]+addr[DOWN][LEFT]))
#define Sobel_Gy(addr, y, x)    (addr[UP][LEFT]+2*addr[UP][x]+addr[UP][RIGHT]-(addr[DOWN][LEFT]+2*addr[DOWN][x]+addr[DOWN][RIGHT]))
#define Sobel_G(addr, y, x)     (abs(Sobel_Gx(addr, y, x)) + abs(Sobel_Gy(addr, y, x)))
//soble阳光算法
void Sobel_Adpt_origin(uint8 (*in_IMG)[image_w], uint8 (*out_IMG)[image_w], uint16 Threshold)
{
    uint8 i, j;
    uint8 UP, DOWN, LEFT, RIGHT;
    if (Threshold == 0)
    {
        for (i = 1; i < image_h-1; i++)
        {
            DOWN = i + 1;     UP = i - 1;
            for (j = 1; j < image_w-1; j++)
            {
                RIGHT = j + 1;     LEFT = j - 1;
                out_IMG[i][j] = Sobel_G(in_IMG, i, j);
            }
        }
    }
    else
    {
        for (i = 1; i <  image_h-1; i++)
        {
            DOWN = i + 1;     UP = i - 1;
            for (j = 1; j < image_w -1; j++)
            {
                RIGHT = j + 1;     LEFT = j - 1;
                out_IMG[i][j] = (Sobel_G(in_IMG, i, j) >= Threshold ? 0 : 255);
            }
        }
    }

}


uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0,l_found = 0,r_found = 0;
    //清零
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

        //从中间往左边，先找起点
    for (i = image_w / 2; i > border_min; i--)
    {
        start_point_l[0] = i;//x
        start_point_l[1] = start_row;//y
        if (imag[start_row][i] == 255 && imag[start_row][i - 1] == 0)
        {
            //printf("找到左边起点image[%d][%d]\n", start_row,i);
            l_found = 1;
            break;
        }
    }

    for (i = image_w / 2; i < border_max; i++)
    {
        start_point_r[0] = i;//x
        start_point_r[1] = start_row;//y
        if (imag[start_row][i] == 255 && imag[start_row][i + 1] == 0)
        {
            //printf("找到右边起点image[%d][%d]\n",start_row, i);
            r_found = 1;
            break;
        }
    }

    if(l_found&&r_found)return 1;
    else {
        //printf("未找到起点\n");
        return 0;
    }
}


#define USE_num image_h*3
 //存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*Endline)
{

    uint8 i = 0, j = 0;
    //左边变量
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics;//统计左边
    //定义八个邻域
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是顺时针

    //右边变量
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//中心坐标点
    uint8 index_r = 0;//索引下标
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//统计右边
    //定义八个邻域
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是逆时针

    l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

    //第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y

        //开启邻域循环
    while (break_flag--)
    {

        //左边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//索引加一

        //右边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y

        index_l = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//先清零，后使用
            temp_l[i][1] = 0;//先清零，后使用
        }

        //左边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i);//记录生长方向
            }

            if (index_l)
            {
                //更新坐标点
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }

        }
        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            //printf("三次进入同一个点，退出\n");
            break;
        }
        if (absolute(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && absolute(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n左右相遇退出\n");
            *Endline = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
            //printf("\n在y=%d处退出\n",*Endline);
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
           // printf("\n如果左边比右边高了，左边等待右边\n");
            continue;//如果左边比右边高了，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
        {
            //printf("\n左边开始向下了，等待右边，等待中... \n");
            center_point_l[0] = points_l[l_data_statics - 1][0];//x
            center_point_l[1] = points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//索引加一

        index_r = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//先清零，后使用
            temp_r[i][1] = 0;//先清零，后使用
        }

        //右边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;//索引加一
                dir_r[r_data_statics - 1] = (i);//记录生长方向
                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            if (index_r)
            {

                //更新坐标点
                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }

            }
        }


    }

    //取出循环次数
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;


}


void get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    left_num=0;
    //初始化
    for (i = 0;i<image_h;i++)
    {
        left[i] = border_min;
        left_dispose[i] = border_min;
    }
    h = image_h - 2;
    //左边
    for (j = 0; j < total_L; j++)
    {
        //printf("%d\n", j);
        if (points_l[j][1] == h)
        {
            left_num++;
            left[h] = points_l[j][0]+1;
            left_dispose[h]= points_l[j][0]+1;
        }
        else continue; //每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)
        {
            break;//到最后一行退出
        }
    }

}

void lost_left(void){
    uint8 i=0;
    left_lost_num=0;
    Lost_left_Flag=0;
    for(i=image_h-5;i>35;i--){
        if(left[i]==2){
            left_lost_num++;
            Lost_point_L_scan_line=i+4;
        }
        if(left_lost_num>15){
            Lost_left_Flag=1; //判断左边下方是否丢线
            return;
        }
    }
}

void get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    right_num=0;
    for (i = 0; i < image_h; i++)
    {
        right[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
        right_dispose[i] = border_max;
    }
    h = image_h - 2;
    //右边
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            right_num++;
            right[h] = points_r[j][0] - 1;
            right_dispose[h] = points_r[j][0] - 1;
        }
        else continue;//每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)break;//到最后一行退出
    }
}

void lost_right(void){
    uint8 i=0;
    right_lost_num=0;
    Lost_right_Flag=0;
    for(i=image_h-5;i>35;i--){
        if(right[i]==185){
            right_lost_num++;
            Lost_point_R_scan_line=i-4;
        }
        if(right_lost_num>15){
            Lost_right_Flag=1;  //判断右边下方是否丢线
            return;
        }
    }
}
//寻找中线
void middle_line(void){

    for(y=119;y>Endline;y--){
        middle[y]=93;
    }
    for(y=119;y>Endline;y--){
        middle[y]=(right[y]+left[y])/2;
    }
}


//腐蚀（像素滤波,去除偶尔出现的噪点,效果有限）二值化时用1表示的白
void Pixle_Filter(uint8(*image)[image_w])
{
    for (uint8 height = 10; height < MT9V03X_H-10; height++)
    {
        for (uint8 width = 10; width < MT9V03X_W -10; width = width + 1)
        {
            if ((image[height][width] == 0) && (image[height - 1][width] + image[height + 1][width] +image[height][width + 1] + image[height][width - 1] >=3*255))
            { //一个黑点的上下左右的白点大于等于三个，令这个点为白
                image[height][width] = 1;
            }
            else if((image[height][width] !=0)&&(image[height-1][width]+image[height+1][width]+image[height][width+1]+image[height][width-1]<2*255))
            {
                image[height][width] =0;
            }
        }
    }
}
//定义膨胀和腐蚀的阈值区间
#define threshold_max   255*5
#define threshold_min   255*2
void image_filter(uint8(*imag)[image_w])//形态学滤波，膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;


    for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < (image_w - 1); j++)
        {
            //统计八个方向的像素值
            num =
                imag[i - 1][j - 1] + imag[i - 1][j] + imag[i - 1][j + 1]
                + imag[i][j - 1] + imag[i][j + 1]
                + imag[i + 1][j - 1] + imag[i + 1][j] + imag[i + 1][j + 1];


            if (num >= threshold_max && imag[i][j] == 0)
            {

                imag[i][j] = 255;//白  可以搞成宏定义，方便更改

            }
            if (num <= threshold_min && imag[i][j] == 255)
            {

                imag[i][j] = 0;//黑

            }

        }
    }

}


void image_draw_rectan(uint8(*image)[image_w])
{

    uint8 i = 0;
    for (i = 0; i < image_h; i++)
    {
        image[i][0] = 0;
        image[i][1] = 0;
        image[i][image_w - 1] = 0;
        image[i][image_w - 2] = 0;

    }
    for (i = 0; i < image_w; i++)
    {
        image[0][i] = 0;
        image[1][i] = 0;
        //image[image_h-1][i] = 0;

    }
}

void get_down_point(void)
{

    Lower_left_inflection_Flag=0;
    Lower_left_inflection_X =0;
    Lower_left_inflection_Y =0;

    Lower_right_inflection_Flag=0;
    Lower_right_inflection_X =0;
    Lower_right_inflection_Y =0;

    for(y=image_h-3;y>Endline+10;y--)
    {
        if(y>30)
        {
            if((left[y]-left[y+4])*(left[y-4]-left[y])-16>=0&&left[y-4]<left[y])//&&left[y-5]==2&&left[y+2]!=2&&left[y]>20&&y>40
            {

                Lower_left_inflection_Flag=1;
                Lower_left_inflection_X =left[y];
                Lower_left_inflection_Y =y;
                break;
            }
            if((right[y]-right[y+4])*(right[y-4]-right[y])-16>=0&&right[y-4]>right[y]&&right[y-5]==185&&right[y+2]!=185&&right[y]<160&&y>40)
            {
                Lower_right_inflection_Flag=1;
                Lower_right_inflection_X =right[y];
                Lower_right_inflection_Y =y;
                break;
            }
        }

    }
}
void get_up_point(void)
{

    Upper_left_inflection_Flag=0;
    Upper_left_inflection_X=0;
    Upper_left_inflection_Y=0;

    Upper_right_inflection_Flag=0;
    Upper_right_inflection_X =0;
    Upper_right_inflection_Y =0;

    for(y=image_h-3;y>Endline+10;y--)
    {
        if(y>30)
        {
            if((left[y]-left[y+4])*(left[y-4]-left[y])-16>=0&&left[y-4]>left[y])//&&left[y+5]==2&&left[y-2]!=2&&left[y]>30&&y>30
            {
//                Process_Curvity

                Upper_left_inflection_Flag=1;
                Upper_left_inflection_X=left[y];
                Upper_left_inflection_Y=y;
                break;
            }
            if((right[y]-right[y+4])*(right[y-4]-right[y])-16>=0&&right[y-4]<right[y]&&right[y+5]==185&&right[y-2]!=185&&right[y]<160&&y>30)
            {

                Upper_right_inflection_Flag=1;
                Upper_right_inflection_X =right[y];
                Upper_right_inflection_Y =y;
                break;
            }
        }

    }
}

//==========================================================拐点识别====================================================

//---------------------------------左下拐点--------------------------------
//-----------------------------第二版：用断点判断------------------------
//void Lower_left(void){
//    Lower_left_inflection_Flag=0;
//    Lower_left_inflection_X =0;
//    Lower_left_inflection_Y =0;
//    uint8 h=image_h-3;
//        for(y=image_h-3;y>Endline+10;y--)
//        {
//        if(annulus_L_Flag==1)
//        {
//          for(h=Lost_point_L_scan_line+10;h>(Endline+10);h--)
//          {
//              if((left[h]-left[h-8])>3&&left[h-8]==2&&left[h]!=2&&(left[h+4]-left[h])<5&&h<100&&left[h]>10)
//              {
//               if(left[h]>93)
//               {            //针对圆环状态1补线出现的左拐点问题进行的优化
//                   Lower_left_inflection_Flag=1;
//                   Lower_left_inflection_X =left[h];
//                   Lower_left_inflection_Y =h;
//                   //-----------第一版：用扫线方法判断-----------
//                   uint8 Find_Flag=0;
//                   for(y=h;y>Endline;y--){
//                       Find_Flag=0;
//                       for(x=left[h]+10;x<50;x--){
//                             if(imag[y][x-1]==Black&&imag[y][x]==Black&&imag[y][x+1]==White){
//                                 Find_Flag=1;
//                                break;
//                             }
//
//                           }
//                       if(Find_Flag==0){
//                           Lower_left_inflection_Y =y-1;
//                           return;
//                           }
//
//                       }
//               }
//               else
//               {
//                   Lower_left_inflection_Flag=1;
//                   Lower_left_inflection_X =left[h];
//                   Lower_left_inflection_Y =h;
//                   return;
//               }
//            }
//         }
//      }
//         else
//            {
//                if((left[y]-left[y-4])>5&&left[y-4]==2&&(left[y]-left[y+2])<5&&left[y]>3)
//                {
//                    Lower_left_inflection_Flag=1;
//                    Lower_left_inflection_X =left[y];
//                    Lower_left_inflection_Y =y;
//                    return;
//                }
//            }
//
//        }
//    //----------------------------第一版：用扫线方法判断--------------------------------
//    //    for(y=110;y>(Endline+10);y--){
//    //        for(x=left[y+1];x<186&&x>1;x++){
//    //              if(imag[y][x-1]==Black&&imag[y][x]==Black&&imag[y][x+1]==White){
//    //                 Lower_left_inflection_X =x;
//    //                 Lower_left_inflection_Y =y;
//    //              //  ips200_draw_point(Lower_left_inflection_X, Lower_left_inflection_Y , RGB565_RED);
//    //                 break;
//    //              }
//    //              if(x>(left[y-1]+3)&&imag[y][x]==White){
//    //                  Lower_left_inflection_Flag=1;
//    //                  X1=Lower_left_inflection_X;
//    //                  Y1=Lower_left_inflection_Y;
//    //                  break;
//    //                  }
//    //            }
//    //
//    //
//    //        }
//}
float LK1=0,LK2=0;
void Lower_left(void){
    Lower_left_inflection_Flag=0;
    Lower_left_inflection_X =0;
    Lower_left_inflection_Y =0;

        for(y=image_h-3;y>Endline+10;y--)
        {
            if(y>30)
            {//&&left[y-4]==2
                if((left[y]-left[y-4])>5&&(left[y]-left[y-10])>10&&left[y+2]!=2&&(left[y]-left[y+2])<3&&y>=60&&y<115)//
                {

//                    regression(1,y+2,y+5);
//                    LK1 = parameterB;
//                    regression(1,y+2,y+8);
//                    LK2= parameterB;
//
//                    if(fabs(LK1-LK2)<0.2)
//                    {
                    Lower_left_inflection_Flag=1;
                    Lower_left_inflection_X =left[y];
                    Lower_left_inflection_Y =y;
                    return;
//                    }

                }
            }

         }
}

void f_Lower_left(void)//左下拐点
{
    uint8 i=0;
    Lower_left_inflection_Flag=0;
    Lower_left_inflection_X =0;
    Lower_left_inflection_Y =0;
    for(i=110;i>=50;i--)
      {
        if(
           abs(left[i]-left[i+1])<=3&&//阈值可以更改
           abs(left[i+1]-left[i+2])<=3&&
           abs(left[i+2]-left[i+3])<=3&&
           abs(left[i]-left[i-6])>=20)
                {
            Lower_left_inflection_Flag=1;
            Lower_left_inflection_X =left[i];
            Lower_left_inflection_Y =i;
                    break;
                }
      }
}
void f_Lower_right(void)
{
    uint8 i=0;
    Lower_right_inflection_Flag=0;
    Lower_right_inflection_X =0;
    Lower_right_inflection_Y =0;
     for(i=110;i>=50;i--)
     {
         if(
         abs(right[i]-right[i+1])<=3&&
         abs(right[i+1]-right[i+2])<=3&&
         abs(right[i+2]-right[i+3])<=3&&
         abs(right[i]-right[i-6])>=20)
         {
             Lower_right_inflection_Flag=1;
             Lower_right_inflection_X =right[i];
             Lower_right_inflection_Y =i;
             break;
         }
     }
}


void flup(void)//左上拐点
{
    uint8 i=0;
    Upper_left_inflection_Flag=0;
    Upper_left_inflection_X =0;
    Upper_left_inflection_Y =0;
    for(i=20;i<=110;i++)
    {
      if(
       abs(left[i]-left[i-1])<=3&&
       abs(left[i-1]-left[i-2])<=3&&
       abs(left[i-2]-left[i-3])<=3&&
       abs(left[i]-left[i+6])>=20)
        {
          Upper_left_inflection_Flag=1;
          Upper_left_inflection_X =left[i];
          Upper_left_inflection_Y =i;
            break;
        }
    }
}
void frup(void)
{
    uint8 i=0;
    Upper_right_inflection_Flag=0;
    Upper_right_inflection_X =0;
    Upper_right_inflection_Y =0;
     for(i=20;i<=110;i++)
     {
         if(
         abs(right[i]-right[i-1])<=3&&
         abs(right[i-1]-right[i-2])<=3&&
         abs(right[i-2]-right[i-3])<=3&&
         abs(right[i]-right[i+6])>=20)
         {
             Upper_right_inflection_Flag=1;
             Upper_right_inflection_X =right[i];
             Upper_right_inflection_Y =i;
             break;
         }
     }
}

//-------------------------------------右下拐点---------------------------------
//-----------------------------第二版：用断点判断------------------------
void Lower_right(void){
    Lower_right_inflection_Flag=0;
    Lower_right_inflection_X =0;
    Lower_right_inflection_Y =0;
    for(y=image_h-3;y>(Endline+10);y--){
        if(y>30){//&&right[y-4]==185
            if((right[y-4]-right[y])>5&&(right[y-10]-right[y])>8&&right[y+2]!=185&&(right[y+2]-right[y])<5&&y>60&&y<115){//
                Lower_right_inflection_Flag=1;
                Lower_right_inflection_X =right[y];
                Lower_right_inflection_Y =y;
                return;
            }
        }

     }
    //----------------------------第一版：用扫线方法判断--------------------------------
    //（可对右下拐点识别，但是右转弯时的拐角也会被判定）
    //        for(y=115;y>(Endline+15);y--){
    //            for(x=right[y+1];x<186&&x>1;x--){
    //                  if(imag[y][x-1]==White&&imag[y][x]==Black&&imag[y][x+1]==Black){
    //                     Lower_right_inflection_X =x;
    //                     Lower_right_inflection_Y =y;
    //                    // ips200_draw_point(Lower_right_inflection_X, Lower_right_inflection_Y , RGB565_RED);
    //                     break;
    //                  }
    //                  if(x<(right[y-1]-3)&&imag[y][x]==White){
    //                      Lower_right_inflection_Flag=1;
    //                      break;
    //                      }
    //                }
    //
    //            }

}

//-------------------------------------左上拐点------------------------------------
//-----------------------------第二版：用断点判断------------------------
void Upper_left(void)
{
    uint8 h=image_h-3;
    Upper_left_inflection_Flag=0;
    Upper_left_inflection_X =0;
    Upper_left_inflection_Y =0;
    if(Lost_left_Flag==1){
        //针对圆环写的找点方式
          if(road_type.LeftCirque==1){
              for(h=image_h-3;h>(Endline+10);h--){
                if((left[h]-left[h+4])>3&&left[h+4]==2&&left[h]!=2&&(left[h-4]-left[h])<3&&left[h]>20&&h<80&&h>=30){
                   if(left[h]>93) {            //针对圆环状态4补线出现的断层问题进行的尝试优化
                       Upper_left_inflection_Flag=1;
                       Upper_left_inflection_X =left[h];
                       Upper_left_inflection_Y =h;
                       //-----------第一版：用扫线方法判断-----------
                       uint8 Find_Flag=0;
                       for(y=h;y<115;y++){
                           Find_Flag=0;
                           for(x=left[h]+10;x>70;x--){
                                 if(imag[y][x-1]==Black&&imag[y][x]==Black&&imag[y][x+1]==White){
                                     Find_Flag=1;
                                    break;
                                 }

                               }
                           if(Find_Flag==0){
                               Upper_left_inflection_Y =y-1;
                               return;
                               }

                           }
                   }
                   else{
                       if(left[h]>30&&left[h]<93)
                       Upper_left_inflection_Flag=1;
                       Upper_left_inflection_X =left[h];
                       Upper_left_inflection_Y =h;
                       return;
                   }
                }
             }
          }
          else{
              for(h=image_h-3;h>(Endline+10);h--){
                if((left[h]-left[h+4])>3&&left[h+10]==2&&left[h]!=2&&(left[h-1]-left[h])<3&&left[h]>20&&h>=15&&h<100){
                       Upper_left_inflection_Flag=1;
                       Upper_left_inflection_X =left[h];
                       Upper_left_inflection_Y =h;
                       return;

                }
             }
          }

    }
}
//-----------------------------------右上拐点-----------------------------------
//-----------------------------第二版：用断点判断------------------------
void Upper_right(void){
    uint8 h=image_h-3;
    Upper_right_inflection_Flag=0;
    Upper_right_inflection_X =0;
    Upper_right_inflection_Y =0;
    if(Lost_right_Flag==1){
        //针对圆环写的找点方式
          if(annulus_R_Flag==1){
              for(h=image_h-3;h>(Endline+10);h--){
                if((right[h+4]-right[h])>3&&right[h+4]==185&&right[h]!=185&&(right[h]-right[h-4])<3&&right[h]<150&&right[h]>40&&h>=30&&h<80){
                   if(right[h]>93) {            //针对圆环状态4补线出现的断层问题进行的尝试优化
                       Upper_right_inflection_Flag=1;
                       Upper_right_inflection_X =right[h];
                       Upper_right_inflection_Y =h;
                       //-----------第一版：用扫线方法判断-----------
                       uint8 Find_Flag=0;
                       for(y=h;y<115;y++){
                           Find_Flag=0;
                           for(x=right[h]+10;x>70;x--){
                                 if(imag[y][x-1]==White&&imag[y][x]==Black&&imag[y][x+1]==Black){
                                     Find_Flag=1;
                                    break;
                                 }

                               }
                           if(Find_Flag==0){
                               Upper_right_inflection_Y =y-1;
                               return;
                               }

                           }

                   }
                   else{
                       Upper_right_inflection_Flag=1;
                       Upper_right_inflection_X =right[h];
                       Upper_right_inflection_Y =h;
                       return;
                   }
                }
             }
          }
          else{
              for(h=image_h-3;h>(Endline+10);h--){
                  if((right[h+4]-right[h])>3&&right[h+10]==185&&right[h]!=185&&(right[h]-right[h-1])<3&&right[h]<160&&h>15&&h<100)
                  {
                          Upper_right_inflection_Flag=1;
                          Upper_right_inflection_X =right[h];
                          Upper_right_inflection_Y =h;
                          return;

                  }

              }
          }

    }
    //----------------------------第一版：用扫线方法判断--------------------------------
    //        for(y=Endline+15;y<110;y++){
    //            for(x=right[y-1];x<186&&x>1;x++){
    //                  if(imag[y][x-1]==White&&imag[y][x]==Black&&imag[y][x+1]==Black){
    //                     Upper_right_inflection_X =x;
    //                     Upper_right_inflection_Y =y;
    //                    // ips200_draw_point(Upper_right_inflection_X, Upper_right_inflection_Y , RGB565_BLUE );
    //                     break;
    //                  }
    //                  if(x>(right[y-1]+3)){
    //                      Upper_right_inflection_Flag=1;
    //                      break;
    //                      }
    //                }
    //            }
}

//--------拐点总判断-----------
void inflection_point(void){

    Upper_left();
    Upper_right();
    Lower_left();
    Lower_right();
}

uint8 wandao_flag=0,zhidao_flag=0;
//弯道和直道判别  利用中线
void wandaopanduan(void)
{
    int chazhi=fabs(middle[60]-94);
    if(chazhi>10)
    {
        wandao_flag=1;
    }
    if(chazhi<10)
    {
        zhidao_flag=1;
    }
}


//==================================================右直线识别===========================================================
void right_straight(void){
//    float k1,k2;
//    Right_straight_flag=0;
//    k1=((float)right[90]-(float)right[60])/30;
//    k2=((float)right[110]-(float)right[30])/80;
//    if(fabs(k1-k2)<0.1)
//    {
//        Right_straight_flag=1;
//
//    }
    Right_straight_flag=0;
    regression(2, 30,60);
    RK0end = parameterB;
    //10-100右线斜率
    advanced_regression(2, 30 , 40,100, 110);
    RK = parameterB;
    //斜率偏差
    kerrorR =RK - RK0end;
    if(Lost_right_Flag==0&&kerrorR<0.11&&kerrorR>-0.28&&RK0end>0.3&&RK>0.3&&RK0end<1.1&&RK<1.1)
    {
        Right_straight_flag=1;
    }
//        ips200_show_float(0, 260,RK0end,3,3);
//        ips200_show_float(70, 260,RK ,3,3);
//        ips200_show_float(140, 260,kerrorR ,3,3);
//        printf("right:%d,30\n %d,110\n",right[30],right[110]);

}
//==================================================左直线识别===========================================================
void left_straight(void)
{
//    float k1,k2;
//    Left_straight_flag=0;
//    k1=((float)left[90]-(float)left[60])/30;
//    k2=((float)left[110]-(float)left[30])/80;
//
//    if(k1>-0.7&&k2>-0.7&&fabs(k1-k2)<0.1)
//    {
//
//        Left_straight_flag=1;
//    }
    Left_straight_flag=0;
    regression(1,30,60);
    LK0end = parameterB;
    //10-100左线斜率
    advanced_regression(1,30 , 40,100, 110);
    LK = parameterB;
    //10-60右线斜率
    regression(2, 30,60);
    kerrorL =LK - LK0end;

    if(Lost_left_Flag==0&&kerrorL<0.1&&kerrorL>-0.3&&LK0end<-0.3&&LK<0.3&&LK0end>-1.3&&LK>-1.3)
    {

        Left_straight_flag=1;

    }
//    ips200_show_float(0, 240,LK0end,3,3);
//    ips200_show_float(70, 240,LK ,3,3);
//    ips200_show_float(140, 240,kerrorL ,3,3);
}

//=================================================十字识别=======================================================
void crossroad111(void)
{
    uint8 i=0;
    uint8 left_up_break0 = 0,left_up_break1 = 0;//左上拐点坐标及其标志位0x.1y
    uint8 left_down_break0 = 0,left_down_break1=0;//左下0x.1y
    uint8 right_up_break0 = 0,right_up_break1 = 0;//右上0x.1y
    uint8 right_down_break0 = 0,right_down_break1 = 0;;//右下0x.1y

    uint8 left_up_break_flage=0,left_down_break_flag=0,right_up_break_flag=0,right_down_break_flag=0;


    //找左上拐点
    for (i=data_stastics_l-10; i> 20; i--)
    {
        if(dir_l[i + 1] == 5 && dir_l[i] == 4 && dir_l[i - 3] == 6
                && dir_l[i - 5] == 6&& dir_l[i - 7] == 6)//4---->6
        {
//               if(points_l[i][0]>=10&&points_l[i][1]>20)
            left_up_break0 = points_l[i][0];//传递x坐标
            left_up_break1 = points_l[i][1];//传递y坐标
            left_up_break_flage=1;
            printf("找到左上拐点:%d,%d\n", left_up_break0,left_up_break1);
            break;
        }
    }

    //找右上拐点
    for (i=data_stastics_r-10; i> 20; i--)
    {
        if (dir_r[i + 1] == 5 && dir_r[i] == 4 && dir_r[i - 3] == 6
                && dir_r[i - 5] == 6  && dir_r[i - 7] == 6)//4-->6
        {
//               if(points_r[i][0]<170&&points_r[i][1]>110)
            right_up_break0 = points_r[i][0];//传递x坐标
            right_up_break1 = points_r[i][1];//传递y坐标
            right_up_break_flag=1;
            printf("找到右上拐点:%d,%d\n", right_up_break0,right_up_break1);
            break;
        }
    }

    if(road_type.Cross==1)
    {
       if(Upper_left_inflection_Flag==1&&Upper_right_inflection_Flag==1&&Upper_left_inflection_Y>25&&Upper_right_inflection_Y>25)
        {
            Addingline2(1,Upper_left_inflection_X,Upper_left_inflection_Y);
            Addingline2(2,Upper_right_inflection_X,Upper_right_inflection_Y);
        }
        else
        {
            Addingline( 1, 50,30, 3, 117);
            Addingline( 2, 119,30, 186, 117);
        }
       if(Upper_left_inflection_Flag==0&&Upper_right_inflection_Flag==0)
       {
          Crossroad_Flag=0;
          Crossroad_memory=0;
          return;
       }

    }

}


void cross_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2])
{
    uint8 i=0;
    uint8 l_start=0, l_end=0;
    uint8 r_start=0, r_end=0;
    float slope_l_rate = 0, intercept_l = 0;//左线斜率，截距
    float slope_r_rate = 0, intercept_r = 0;//右线斜率，截距
    uint8 break_num_l = 0;
    uint8 break_num_r = 0;
//    uint8 start, end;
//    float slope_l_rate = 0, intercept_l = 0;

    uint8 left_up_break0 = 0,left_up_break1 = 0;//左上拐点坐标及其标志位0x.1y
    uint8 left_down_break0 = 0,left_down_break1=0;//左下0x.1y
    uint8 right_up_break0 = 0,right_up_break1 = 0;//右上0x.1y
    uint8 right_down_break0 = 0,right_down_break1 = 0;;//右下0x.1y

    uint8 left_up_break_flage=0,left_down_break_flag=0,right_up_break_flag=0,right_down_break_flag=0;

//    l_shizi_down_break();
//    l_shizi_up_break();
//    r_shizi_up_break();
//    r_shizi_down_break();
    //先找左下拐点
       for (i=total_num_l-10; i> 0; i--)
       {
           if (dir_l[i - 1] == 4 && dir_l[i] == 3 && dir_l[i + 3] == 2
                   && dir_l[i + 5] == 2&& dir_l[i + 7] == 2)//4--
           {
               left_down_break0 = points_l[i][0];//传递x坐标
               left_down_break1 = points_l[i][1];//传递y坐标
               left_down_break_flag=1;
               printf("找到左下拐点:%d\n,%d\n", left_down_break0,left_down_break1);
               break;
           }
       }
       //找左上拐点
       for (i=total_num_l-10; i> 20; i--)
       {
           if(dir_l[i + 1] == 5 && dir_l[i] == 4 && dir_l[i - 3] == 6
                   && dir_l[i - 5] == 6&& dir_l[i - 7] == 6)//4---->6
           {
//               if(points_l[i][0]>=10&&points_l[i][1]>20)
               left_up_break0 = points_l[i][0];//传递x坐标
               left_up_break1 = points_l[i][1];//传递y坐标
               left_up_break_flage=1;
               printf("找到左上拐点:%d,%d\n", left_up_break0,left_up_break1);
               break;
           }
       }
          //找右下拐点
       for (i=total_num_l-10; i> 0; i--)
        {
            if (dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] == 2
                    && dir_r[i + 5] == 2  && dir_r[i + 7] == 2)//从上往下找2-->4
            {
//                if(points_r[i][0]<180&&points_r[i][1]>118)
                right_up_break0 = points_r[i][0];//传递x坐标
                right_up_break1 = points_r[i][1];//传递y坐标
                right_down_break_flag=1;
                printf("找到右下拐点:%d,%d\n", right_up_break0,right_up_break1);
                break;
            }
        }
       //找右上拐点
       for (i=total_num_l-10; i> 20; i--)
       {
           if (dir_r[i + 1] == 5 && dir_r[i] == 4 && dir_r[i - 3] == 6
                   && dir_r[i - 5] == 6  && dir_r[i - 7] == 6)//4-->6
           {
//               if(points_r[i][0]<170&&points_r[i][1]>110)
               right_up_break0 = points_r[i][0];//传递x坐标
               right_up_break1 = points_r[i][1];//传递y坐标
               right_up_break_flag=1;
               printf("找到右上拐点:%d,%d\n", right_up_break0,right_up_break1);
               break;
           }
       }
    if (left_up_break_flage && left_down_break_flag&&right_down_break_flag && right_up_break_flag)
    {
        Addingline( 1,left_up_break0 ,left_up_break1, left_down_break0,  left_down_break1);  //输入两个点的横纵坐标
        printf("补线左\n");
        Addingline(2,right_up_break0 , right_up_break1,right_down_break0, right_down_break1);   //输入两个点的横纵坐标
        printf("补线右1\n");
    }
    else if(left_down_break_flag==0&&left_up_break_flage&&right_down_break_flag==0 && right_up_break_flag)
    {
        Addingline2(1,left_up_break0 ,left_up_break1);
        Addingline2(2,right_up_break0 , right_up_break1);
        printf("斜率补线右2\n");
    }
}
void crossroad222(void)
{
    if(road_type.Cross==1)
    {
        if(Lower_left_inflection_Flag==1&&Upper_left_inflection_Flag==1)//左边两个拐点拉线
       {
           Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, Lower_left_inflection_X,Lower_left_inflection_Y);
       }
       if(Lower_right_inflection_Flag==1&&Upper_right_inflection_Flag==1)//右边两个拐点拉线
       {
           Addingline(2 ,Upper_right_inflection_X, Upper_right_inflection_Y,Lower_right_inflection_X,Lower_right_inflection_Y);
       }
       if(Lower_left_inflection_Flag==1&&Upper_left_inflection_Flag==1&&Upper_right_inflection_Flag==1)//左上，左下，右上三个个拐点拉线
       {
           Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, Lower_left_inflection_X,Lower_left_inflection_Y);
           for(uint8 y =MT9V03X_H -30; y >=MT9V03X_H -40; y--)
            {
               right[y] = left[y] + kuandu_saidao[MT9V03X_H-1 -y] + 5;
               left[y] = left[y] + 5;
            }
       }
       if(Lower_right_inflection_Flag==1&&Upper_right_inflection_Flag==1)//左上，右下，右上三个个拐点拉线
       {
           Addingline(2 ,Upper_right_inflection_X, Upper_right_inflection_Y,Lower_right_inflection_X,Lower_right_inflection_Y);
           for(uint8 y =MT9V03X_H -30; y >=MT9V03X_H -40; y--)
            {
                left[y] = right[y] - kuandu_saidao[MT9V03X_H-1 -y] - 5;
                right[y] = right[y] - 5;
            }
       }
       if(Lower_left_inflection_Flag==0&&Lower_right_inflection_Flag==0&&Upper_left_inflection_Flag==1&&Upper_right_inflection_Flag==1&&Upper_right_inflection_Y<100&&Upper_left_inflection_Y<100){
          Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, 5, 118);
          Addingline( 2,Upper_right_inflection_X, Upper_right_inflection_Y, 180,118);
      }
      else{
          Addingline( 1, 45,55, 3, 117);
          Addingline( 2, 119,55, 180, 117);
      }
      //退出
      if(Upper_left_inflection_Flag==0&&Upper_right_inflection_Flag==0){
          Crossroad_Flag=0;
          Crossroad_memory=0;
          road_type.Cross=0;
          return;
      }
    }
}

void crossroad(void){

//    if(Lost_left_Flag==1&&Lost_right_Flag==1&&Upper_left_inflection_Flag==1&&Upper_right_inflection_Flag==1){
//        Crossroad_Flag=1;
//    }
    if(road_type.Cross==1){
//    if(Crossroad_Flag==1){ // 原代码
        //状态1 看到十字，和左右下面两个拐点
       if(Crossroad_memory==1){
          //方案一：强行补线
//           Beep_Action(1);
//           Addingline( 1, 45,55, 3, 117);
//           Addingline( 2, 119,55, 186, 117);
           if(Lower_left_inflection_Flag==1){
               Addingline1(1,Lower_left_inflection_X,Lower_left_inflection_Y);
           }
           else{
//               Addingline( 1, 45,55, 3, 117);
               Addingline( 1, 50,30, left[117] , 117);
           }

           if(Lower_right_inflection_Flag==1){
               Addingline1(2,Lower_right_inflection_X,Lower_right_inflection_Y);
           }
           else{
               Addingline( 2, 119,30, right[117] , 117);
//               Addingline( 2, 119,55, 186, 117);
           }

        if ( Upper_left_inflection_Flag == 1 || Upper_right_inflection_Flag == 1)
            {
            Crossroad_memory=2;
            }
         }
       //状态2 看到左上拐点和右上拐点，向下拉线
       if(Crossroad_memory==2){
//           Beep_Action(1);
           if(Upper_left_inflection_Flag==1&&Upper_right_inflection_Flag==1){
               Addingline( 1, Upper_left_inflection_X+1, Upper_left_inflection_Y-1, left[117] , 117 );
               Addingline( 2,Upper_right_inflection_X-1, Upper_right_inflection_Y-1, right[118] , 118 );
           }
//           else{
//                Addingline( 1, 50,30, 3, 117);
//                Addingline( 2, 119,30, 186, 117);
               // 原代码
//               Addingline( 1, 45,55, 3, 117);
//               Addingline( 2, 119,55, 186, 117);
           }
           if(Upper_left_inflection_Flag==1&&Upper_right_inflection_Flag==1)
           {
               Crossroad_memory=3;
           }
       }
//       状态3
       if(Crossroad_memory==3)
       {
//           Beep_Action(1);
        if(Lower_left_inflection_Flag==1&&Upper_left_inflection_Flag==1)//左边两个拐点拉线
        {
        Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, Lower_left_inflection_X+5,Lower_left_inflection_Y);
        }
        else if(Lower_left_inflection_Flag==1&&Upper_left_inflection_Flag==0)
        {
        Addingline1(1, Lower_right_inflection_X, Lower_right_inflection_Y) ;
        }


        if(Lower_right_inflection_Flag==1&&Upper_right_inflection_Flag==1)//右边两个拐点拉线
        {
        Addingline(2 ,Upper_right_inflection_X, Upper_right_inflection_Y,Lower_right_inflection_X,Lower_right_inflection_Y);
        }
        else if(Lower_right_inflection_Flag==1&&Upper_right_inflection_Flag==0)
        {
        Addingline1(2, Lower_right_inflection_X, Lower_right_inflection_Y) ;
        }

        if(Lower_left_inflection_Flag==0&&Lower_right_inflection_Flag==0)
        {
        Crossroad_memory=4;
        }
       }
       //状态4 补线出十字
       if(Crossroad_memory==4){
//           Beep_Action(1);
           if(Upper_left_inflection_Flag==1&&Upper_right_inflection_Flag==1&&Upper_right_inflection_Y<110&&Upper_left_inflection_Y<110){
               Addingline( 1, Upper_left_inflection_X+1, Upper_left_inflection_Y, 35, 118);
               Addingline( 2,Upper_right_inflection_X-1, Upper_right_inflection_Y, 165,118);
           }
//           else{
//               Addingline( 1, 50,65, 35, 117);
//               Addingline( 2, 150,75, 165, 117);
//           }
           //退出
           if(Upper_left_inflection_Flag==0&&Upper_right_inflection_Flag==0){
//               if(Lost_right_Flag==0&&Lost_left_Flag==0)
               Crossroad_Flag=0;
               Crossroad_memory=0;
               road_type.Cross=0;
               Beep_Action(0);
               return;
           }
       }
}

void crossroad2(void){

//    if(Lost_left_Flag==1&&Lost_right_Flag==1&&Upper_left_inflection_Flag==1&&Upper_right_inflection_Flag==1){
//        Crossroad_Flag=1;
//    }
    if(road_type.Cross==1){
//    if(Crossroad_Flag==1){ // 原代码
       if(Crossroad_memory==1)
       {
//           Beep_Action(1);
        if(Lower_left_inflection_Flag==1&&Upper_left_inflection_Flag==1)//左边两个拐点拉线
        {
        Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, Lower_left_inflection_X+5,Lower_left_inflection_Y);
        }
        else if(Lower_left_inflection_Flag==1&&Upper_left_inflection_Flag==0)
        {
        Addingline1(1, Lower_left_inflection_X+5,Lower_left_inflection_Y);
        }
        else if(Lower_left_inflection_Flag==0&&Upper_left_inflection_Flag==1)
        {
        Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, 35, 117);
//        Addingline1(1, Lower_left_inflection_X+5,Lower_left_inflection_Y);
        }
        else
        {
            Addingline( 1, 94,30, 35, 117);
        }

//        else if(Upper_left_inflection_Flag==0)//Lower_left_inflection_Flag==0&&
//        {
//            Addingline( 1, 94,30, 35, 117);
//        }
        if(Lower_right_inflection_Flag==1&&Upper_right_inflection_Flag==1)//右边两个拐点拉线
        {
        Addingline(2 ,Upper_right_inflection_X, Upper_right_inflection_Y,Lower_right_inflection_X,Lower_right_inflection_Y);
        }
        else if(Lower_right_inflection_Flag==1&&Upper_right_inflection_Flag==0)
       {
//            Addingline( 2, 150,75, 165, 117);
            Addingline(2 ,110, 30,Lower_right_inflection_X,Lower_right_inflection_Y);
//        Addingline1(2, Lower_right_inflection_X, Lower_right_inflection_Y) ;
       }
         else if(Lower_right_inflection_Flag==0&&Upper_right_inflection_Flag==1)
        {
        Addingline(2 ,Upper_right_inflection_X, Upper_right_inflection_Y,165, 117);
//        Addingline1(2, Lower_right_inflection_X, Lower_right_inflection_Y) ;
        }
        else
        {
                Addingline( 2, 94,30, 165, 117);
        }
//        else if(Upper_right_inflection_Flag==0)//Lower_right_inflection_Flag==0&&
//        {
//            Addingline( 2, 94,30, 165, 117);
//        }
        if(Upper_left_inflection_Flag==1&&Upper_right_inflection_Flag==1&&Upper_right_inflection_Y<110&&Upper_left_inflection_Y<110){
            Addingline( 1, Upper_left_inflection_X+1, Upper_left_inflection_Y, 35, 118);
            Addingline( 2,Upper_right_inflection_X-1, Upper_right_inflection_Y, 165,118);
        }

//        if(Lower_left_inflection_Flag==0&&Lower_right_inflection_Flag==0)
//        {
//        Crossroad_memory=2;
//        }
        if(Upper_left_inflection_Flag==0&&Upper_right_inflection_Flag==0){
//               if(Lost_right_Flag==0&&Lost_left_Flag==0)
            Crossroad_Flag=0;
            Crossroad_memory=0;
            road_type.Cross=0;
            Beep_Action(0);
            return;
        }
       }
       //状态4 补线出十字
//       if(Crossroad_memory==2)
//       {
////           Beep_Action(1);
//           if(Upper_left_inflection_Flag==1&&Upper_right_inflection_Flag==1&&Upper_right_inflection_Y<110&&Upper_left_inflection_Y<110){
//               Addingline( 1, Upper_left_inflection_X+1, Upper_left_inflection_Y, 35, 118);
//               Addingline( 2,Upper_right_inflection_X-1, Upper_right_inflection_Y, 165,118);
//           }
////           else{
////               Addingline( 1, 50,65, 35, 117);
////               Addingline( 2, 150,75, 165, 117);
////           }
//           //退出
//           if(Upper_left_inflection_Flag==0&&Upper_right_inflection_Flag==0){
////               if(Lost_right_Flag==0&&Lost_left_Flag==0)
//               Crossroad_Flag=0;
//               Crossroad_memory=0;
//               road_type.Cross=0;
//               Beep_Action(0);
//               return;
//           }
//       }
    }
}

//=============================================圆环===================================================
//================环岛识别=====================

void roundabout_L(void){

    roundabout_X=0;
    roundabout_Y=0;
    roundabout_Flag=0;
    for(y=image_h-3;y>10;y--){
        if((left[y]-left[y-8])>=5&&(left[y]-left[y+2])<=5&&left[115]==2&&Lost_left_Flag==1){
            y+=4;
            roundabout_Flag=1;
            roundabout_X =left[y];
            roundabout_Y =y;
            return;
        }
     }
}

void roundabout_R(void){
    roundabout_X=0;
    roundabout_Y=0;
    roundabout_Flag=0;
    for(y=image_h-3;y>10;y--){
//        if((right[y-8]-right[y])>5&&right[y-8]==185&&(right[y+2]-right[y])<5&&Lost_right_Flag==1){
        if((right[y-8]-right[y])>=5&&(right[y+2]-right[y])<=5&&right[115]==185&&Lost_right_Flag==1){
            y+=4;
            roundabout_Flag=1;
            roundabout_X =right[y];
            roundabout_Y =y;
            return;
        }
     }

}

//================出环拐点识别=====================
void Exit_loop_L_inflection(void){
   uint8 i;
   Exit_loop_Flag=0;
   Exit_loop_X=0;
   Exit_loop_Y=0;
  // 左圆环，识别右边拐点
    for(i=115;i>Endline+10;i--){
//        if(right[i]<right[i+4]&&right[i]<right[i-4]&&right[i]<right[i+3]&&right[i]<right[i-3]&&right[i-8]==185){
        if(right[i]<right[i+4]&&right[i]<right[i-4]&&right[i]<right[i+3]&&right[i]<right[i-3])
        {
            Exit_loop_Flag=1;
            Exit_loop_X=right[i];
            Exit_loop_Y=i;
            return;
        }
    }
}

void Exit_loop_R_inflection(void){
   uint8 i;
   Exit_loop_Flag=0;
   Exit_loop_X=0;
   Exit_loop_Y=0;
  //右圆环，识别左边拐点
    for(i=115;i>Endline+10;i--){
//        &&right[i+8]==185
        if(left[i+4]<left[i]&&left[i-4]<left[i]&&left[i+3]<left[i]&&left[i-3]<left[i]&&left[i-8]==2){
            Exit_loop_Flag=1;
            Exit_loop_X=left[i];
            Exit_loop_Y=i;
            return;
        }
    }
}

// 右线=赛道宽度/2+左边线  或
void track_line( uint8 type , uint16 start , uint16 end )
{
    if(type==0)
    {
      for( int i=start ; i>end ; i-- )
        {
            left[i] = right[i] - kuandu_saidao[i];    //范围100行
            left[i] = right[i];
        }
    }
    if(type==1)
    {
      for( int i=start ; i>end ; i-- )
        {
            right[i] = left[i] + kuandu_saidao[i];    //范围100行
            right[i] = left[i];
        }
    }

}

//================左圆环识别=====================
void annulus_L(void){
//     if(annulus_L_Flag==1){// 原代码
    if(road_type.LeftCirque==1){
         //状态 1  识别到圆环，未识别到环岛（可强行补线，或不补线，待测试）
         if (annulus_L_memory == 1)
         {
             Beep_Action(1);
             if(Lower_left_inflection_Flag==1&&Lower_left_inflection_Y<117)
             {
              Addingline( 1, 74,20,Lower_left_inflection_X, Lower_left_inflection_Y);
//                roundabout_L();
             }
//             else if(Lower_left_inflection_Flag==0)
//              {
//                  if(Lost_left_Flag==1)
//                  {
//                      Addingline( 1, 64,30, 10, 117);//强行补线  额外加的
//                  }
//             }
             else if(Lower_left_inflection_Flag==0)
             {
                   annulus_L_memory = 2;
             }
//             else
//                 Addingline( 1, 64,30, 10, 117);//强行补线  额外加的
//                 annulus_L_memory = 2;
         }
         //状态2 识别到圆环，环岛，并对左边进行补线
          if (annulus_L_memory == 2 )
         {
              Beep_Action(0);
              roundabout_L();          //环岛
            if(roundabout_Flag==1&&roundabout_Y<110)
            {
                Addingline( 1, roundabout_X, roundabout_Y,left[117] , 117 );
             }
//            else {
////                left[119], 119
//                Addingline( 1, 50,50, left[117] , 117 );//强行补线  额外加的
//                //Addingline( 1, 56, 38,2 , 118 );
//            }
//            if(Upper_left_inflection_Flag==1&&roundabout_Y>40)
                if(Lost_left_Flag==1&&left[115]!=2)
            {   //Upper_left_inflection_Flag==1&&&&roundabout_Flag==0改变roundabout_Y限制间接改变入环位置，待验证
                annulus_L_memory = 3;
            }
            else return;
         }
         //状态3 到达圆环入口，封住前路，补线入环
          if (annulus_L_memory == 3 )
          {
//              Beep_Action(1);
//              if(Upper_left_inflection_Flag==1)
//              {
//                  //Addingline( 2, Upper_left_inflection_X,Upper_left_inflection_Y,right[119] , 119);
//                  if(Upper_left_inflection_X>35&&Upper_left_inflection_Y<70){
//                      Addingline( 2, Upper_left_inflection_X-30,Upper_left_inflection_Y-5,right[119], 119);
////                      Addingline( 2, Upper_left_inflection_X,Upper_left_inflection_Y,170, 119);
//                  }
//                  else if(Lost_left_Flag==1&&Lost_right_Flag==1)
//                  {
//                      Addingline( 2, 30,25,right[118], 118);
////                      Addingline( 2, 5,25,right[118], 118);//小环
//
//                  }
//                  return;
//              }

              //小环
              if(Upper_left_inflection_Flag==1)
            {
                //Addingline( 2, Upper_left_inflection_X,Upper_left_inflection_Y,right[119] , 119);
                if(Upper_left_inflection_X>35&&Upper_left_inflection_Y>50&&Upper_left_inflection_Y<60){
                    Addingline( 2, Upper_left_inflection_X-35,Upper_left_inflection_Y-1,right[119]+10, 119);
                }
                else{
                    Addingline( 2, 5,25,right[118], 118);
                    }
                return;
            }
             if(Upper_left_inflection_Flag==0&&Lost_right_Flag==0&&Endline>30)
                  annulus_L_memory = 4;
              else return;

//              if(Lost_left_Flag==1)
//              {
//                       if(left[60]==2)
//                   {
//                       Addingline( 2, 35,25,right[117], 117);
//                   }
//                  else if(Lost_right_Flag==1)
//                  {
//                      Addingline( 2, 15,25,right[110], 110);
//                          return;
//                  }
//              }
//              if(Upper_left_inflection_Flag==0&&Lost_left_Flag==1&&Lost_right_Flag==0&&Endline>30)
//                  annulus_L_memory = 4;
//              else return;
          }
         //状态4 在圆环中行驶，当看到右下拐点时进入下一状态
         if (annulus_L_memory == 4)
         {
             Beep_Action(0);
            Exit_loop_L_inflection();
            if(Exit_loop_Flag==0){
                return;
            }
//            if(Exit_loop_Flag==1&&Lost_left_Flag==1&&Lost_right_Flag==1&&right[Exit_loop_Y-10]==185&&right[Exit_loop_Y+4]!=185){
            if(Exit_loop_Flag==1&&Lost_right_Flag==1){
                annulus_L_memory = 5;
            }
         }
         //状态5 出环时看到右下拐点,对其补线处理
         if (annulus_L_memory == 5 )
          {
//             Beep_Action(1);
             Exit_loop_L_inflection();
             if(Lost_right_Flag==1)
             {
                 if(Exit_loop_Flag==1&&Exit_loop_Y<110)
                 {
                     Addingline( 2, 50, Endline, Exit_loop_X, Exit_loop_Y);//此处补线结束点待测试
                     return;
                 }
                 else  if(right[70]==185)
                 {
                     Addingline( 2,50, Endline, right[117], 117);
                     return;
                 }

             }
             if(Lost_left_Flag==1&&Lost_right_Flag==1&&Exit_loop_Flag==0){
                 annulus_L_memory = 6;
             }
             else return;
          }

         //状态6 出环时右下拐点消失，但是车还没完全出环，此时还需要补线处理
         if (annulus_L_memory == 6 )
          {

             Beep_Action(0);
             if(Lost_left_Flag==1&&Lost_right_Flag==1){
                 Addingline( 2,40, Endline, right[117], 117);
//                 return;
             }
//             else if(Lost_left_Flag==1&&Lost_right_Flag==0&&Right_straight_flag==1){
             else if(Lost_right_Flag==0){//Lost_left_Flag==1&&
                 annulus_L_memory = 7;
             }
             else return;
          }
         //状态7 出环补线
         if (annulus_L_memory == 7)
      {
             Beep_Action(1);
                if(Upper_left_inflection_Flag==1&&Upper_left_inflection_Y<=115&&Upper_left_inflection_Y>30&&bianxian_tubian(1)==0)
             {
                 Addingline( 1, Upper_left_inflection_X+10, Upper_left_inflection_Y-1, 10, 118);
                 return;
             }
            else
            {
                Addingline( 1, 30, 30, 10, 118); // 要改
            }
         if(Lost_left_Flag==0)//Lost_right_Flag==0&&
         {
             annulus_L_memory =0;
             annulus_L_Flag=0;
             road_type.LeftCirque =0;
             Beep_Action(0);
             return;
         }
      }
     }
}

//================右圆环识别=====================
void annulus_R(void)
{
     if(annulus_R_Flag==1){
         //状态 1  识别到圆环，未识别到环岛（可强行补线，或不补线，待测试）
         if (annulus_R_memory == 1)
         {
             Beep_Action(1);
             if(Lower_right_inflection_Flag==1)
             {
                 Addingline( 2,130,20,Lower_right_inflection_X, Lower_right_inflection_Y);

                //roundabout();
             }
             else if(Lower_right_inflection_Flag==0){
                 annulus_R_memory = 2;
             }
         }
         //状态2 识别到圆环，环岛，并对右边进行补线
          if (annulus_R_memory == 2 )
         {
              Beep_Action(0);
              roundabout_R();          //环岛
            if(roundabout_Flag==1){
                Addingline( 2, roundabout_X, roundabout_Y,right[118] , 118 );
             }
//            else {
//                Addingline( 2, 120, 50,right[118] , 118 );  //要改
//            }
            if(Lost_right_Flag==1&&right[115]!=185){   //&&roundabout_Flag==0改变Upper_right_inflection_Y限制间接改变入环位置，待验证
                annulus_R_memory = 3;
            }
            else return;
         }
         //状态3 到达圆环入口，封住前路，补线入环
          if (annulus_R_memory == 3 )
          {
//              Beep_Action(1);
//              if(Upper_right_inflection_Flag==1&&Upper_right_inflection_Y<40&&Upper_right_inflection_X>65){
                 if(Lost_right_Flag==1)
                 {
                     if(Upper_right_inflection_Flag==1)
                     {
                         Addingline( 1, Upper_right_inflection_X+20,Upper_right_inflection_Y-1,left[118]+10, 118);
                     }
//                  return;
                     else if(Lost_left_Flag==1&&Lost_right_Flag==1)
                      {
                          Addingline( 1, 160,20,left[118], 118);
//                          Addingline( 1, 180,20,left[118], 118);//小环
                          return;
                      }
              }

//                 if(Lost_right_Flag==1)//小环
//                 {
//                     if(Upper_right_inflection_Flag==1)
//                     {
//                         Addingline( 1, Upper_right_inflection_X+35,Upper_right_inflection_Y-1,left[118]+10, 118);
//                     }
////                  return;
//                     else if(Lost_left_Flag==1&&Lost_right_Flag==1)
//                      {
//                          Addingline( 1, 180,20,left[118], 118);
//                          return;
//                      }
//              }
              if(Upper_right_inflection_Flag==0&&Lost_right_Flag==1&&Endline>30)
              {
                  annulus_R_memory = 4;
              }
              else return;

          }
         //状态4 在圆环中行驶，当看到左下拐点时进入下一状态
         if (annulus_R_memory == 4)
         {
             Beep_Action(0);
            Exit_loop_R_inflection();
            if(Exit_loop_Flag==0){
                return;
            }
            if(Exit_loop_Flag==1&&Lost_left_Flag==1)//&&left[Exit_loop_Y+4]!=2
            {
                annulus_R_memory = 5;
            }
         }
         //状态5 出环时看到左下拐点,对其补线处理
         if (annulus_R_memory == 5 )
          {
//             Beep_Action(1);
             Exit_loop_R_inflection();
             if(Lost_left_Flag==1)
             {
                 if(Exit_loop_Flag==1&&Exit_loop_Y<90)
                 {
                     Addingline( 1, 170, Endline+2, Exit_loop_X, Exit_loop_Y);
                     return;
                 }
                 else  if (left[70] == 2)
                 {
                     Addingline(1, 170, Endline+2, left[117], 117);
                     return;
                 }
             }
             if(Lost_right_Flag==1&&Lost_left_Flag==1&&Exit_loop_Flag==0){
                 annulus_R_memory = 6;
             }
             else return;
          }

         //状态6 出环时左下拐点消失，但是车还没完全出环，此时还需要补线处理
         if (annulus_R_memory == 6 )
          {
             Beep_Action(0);
             if(Lost_right_Flag==1&&Lost_left_Flag==1){

                 Addingline( 1, 150, Endline+5, left[118], 118);
//                 return;
             }
             else if(Lost_left_Flag==0){//&&Lost_right_Flag==1
                 annulus_R_memory = 7;
             }
             else return;
          }
         //状态7 出环补线
         if (annulus_R_memory == 7)
          {
             Beep_Action(1);
             if(Upper_right_inflection_Flag==1)
             {
                 Addingline( 2, Upper_right_inflection_X, Upper_right_inflection_Y, 170, 118);
                 return;
             }
//             else
//             {
//                 Addingline( 2, 165, 50, 185, 118); // 要改
//             }
         if(Lost_right_Flag==0&&Lost_left_Flag==0)
             {
                 annulus_R_memory =0;
                 annulus_R_Flag=0;
                 road_type.RightCirque=0;
                 Beep_Action(0);
                 return;
             }
          }
     }
}

//===================================================左车库===================================================
//
//void check_starting_line(byte start_point, byte end_point)
//{
//    byte times = 0;
//    for (byte y = start_point; y <= end_point; y++)
//    {
//        byte black_blocks = 0;
//        byte cursor = 0;    //指向栈顶的游标
//        for (byte x = 0; x <= 185; x++)
//        {
//            if (Pixels[y, x] == 0)
//            {
//                if (cursor >= 20)
//                {
//                    break;          //当黑色元素超过栈长度的操作   break;
//                }
//                else
//                {
//                    cursor++;
//                }
//            }
//            else
//            {
//                if (cursor >= 4 && cursor <= 8)
//                {
//                    black_blocks++;
//                    cursor = 0;
//                }
//                else
//                {
//                    cursor = 0;
//                }
//            }
//        }
//        if (black_blocks >= 6 && black_blocks <= 9) times++;
//    }
//    if (times >= 1)
//    {
//        flag_starting_line = 1;
//        flag_ruku = 1;
//        flag_find_cheku = 0;
//    }
//    else
//    {
//        flag_starting_line = 0;
//    }
////}
//判断斑马线函数

void banmaxian(int start_point, int end_point)
{
        road_type.zebra_flag=0;
        zebra_crossing_flag=0;
        //变量标志位
        int zebra_kuandu;//斑马线宽度
        int zebra_hangshu;//斑马线行数
        int zebra_geshu;//斑马线个数（块）
        int cursor0=0,cursor1=0;
        //从下往上扫描
        for (int y = end_point; y >= start_point; y--)
        {
            zebra_kuandu=0;
            zebra_hangshu=0;
            zebra_geshu=0;
             //从左往右扫描
             for(int i=3;i<185;i++)
             {
                 if(imag[Endline+15][i-1]==White&&imag[Endline+15][i]==Black&&imag[Endline+15][i+1]==Black)
                 {
                     cursor0=i;
                     zebra_kuandu++;
//                     if(imag[Endline+15][i-1]==Black&&imag[Endline+15][i]==Black&&imag[Endline+15][i+1]==White)
//                     {
//                         cursor1=i;
//                         break;
//                     }
//                     banmaxian_kuandu=cursor1-cursor0;
                 }
                 if (zebra_kuandu >= 4 && zebra_kuandu <= 8)
                 {
                     cursor0=cursor1=0;
                     zebra_kuandu = 0;
                     zebra_geshu++;
                 }
                 else
                 {
                     cursor0=cursor1=0;
                     zebra_kuandu = 0;
                 }
             }

             //如果色块的个数在6~9之间则认为这一行的斑马线满足要求，在去扫下一行
            if (zebra_geshu >= 6 && zebra_geshu <= 9)
            {
                zebra_hangshu++;
            }
        }
        //如果有大于等于4行的有效斑马线
        if(zebra_hangshu>=4)
        {
            //斑马线标准位置1
            road_type.zebra_flag=1;
            zebra_crossing_flag=1;
        }
        else
        {
            road_type.zebra_flag=0;
            zebra_crossing_flag=0;
        }

}



uint8 stop=0;
uint16 time_count=0;
void car_start(void)
{
    if(time_count>=1000&&stop==0)
    {
        stop=1;
    }
}



//=========斑马线===========


int16 zebra_m=0;
void zebra_crossing(uint8 zebra_start)
{
    uint8 i;
    uint8 num;
    uint8 hangshu;
    hangshu=0;
    num=0;
    Beep_Action(0);

    for(i=left[zebra_start]+1;i<right[zebra_start]-1;i++)
    {
        if(imag[zebra_start][i-1]==White&&imag[zebra_start][i]==Black&&imag[zebra_start][i+1]==Black){
            num++;
        }
        if(num>=5&&num<= 9)
        {
            hangshu++;
        }
        if(hangshu>=2)
        {
            zebra_crossing_memory=1;
            zebra_crossing_flag=1;
            road_type.zebra_flag=1;
//            Beep_Action(1);
        }
    }
    for(i=left[zebra_start+10]+1;i<right[zebra_start+10]-1;i++)
    {
        if(imag[zebra_start][i-1]==White&&imag[zebra_start][i]==Black&&imag[zebra_start][i+1]==Black){
            num++;
        }
        if(num>=5&&num<= 9)
        {
            hangshu++;
        }
        if(hangshu>=2)
        {
            zebra_crossing_memory=2;
            zebra_crossing_flag=1;
            road_type.zebra_flag=1;
//            Beep_Action(1);
        }
    }
    for(i=left[zebra_start+10]+1;i<right[zebra_start+10]-1;i++)
   {
       if(imag[zebra_start][i-1]==White&&imag[zebra_start][i]==Black&&imag[zebra_start][i+1]==Black){
           num++;
       }
       if(num>=5&&num<= 9)
       {
           hangshu++;
       }
       if(hangshu>=2)
       {
           zebra_crossing_memory=3;
           zebra_crossing_flag=1;
           road_type.zebra_flag=1;
//            Beep_Action(1);
       }
   }

    if(road_type.zebra_flag==1)
    {
        Beep_Action(1);
        Beep_Action(0);
        road_type.LeftCirque =0;
        road_type.RightCirque =0;
        road_type.Cross = 0;
        road_type.Ramp1=0;
        Ramp_memory=0;
        road_type.straight = 0;
        road_type.bend = 0;

        encoder.encoder_enable=1;
        zebra_m=encoder.encodersum_average;
//        if(<15000)
//        {
//            road_type.zebra_flag=1;
//        }
//        else
//        {
//            road_type.zebra_flag=0;
//            zebra_crossing_memory=0;
//        }
    }

}

//int16 zebra_m=0;
//void zebra_crossing(uint8 zebra_start)
//{
//    uint8 i,j;
//    uint8 num;
//    uint8 hangshu;
//    uint8 a=zebra_start;
//    hangshu=0;
//    num=0;
//    Beep_Action(0);
//
//    for(j=70;j<=75;j++)
//    {
//        for(i=left[j]+1;i<right[j]-1;i++)
//        {
//            if(imag[j][i-1]==White&&imag[j][i]==Black&&imag[j][i+1]==Black)
//            {
//                num++;
//            }
//            if(num>=5&&num<=9)//
//            {
//                hangshu++;
//            }
//        }
//    }
//    if(hangshu>=2)
//        {
//            zebra_crossing_memory=5;
//            zebra_crossing_flag=1;
//            road_type.zebra_flag=1;
////            Beep_Action(1);
//        }
//    if(road_type.zebra_flag==1)
//    {
//        Beep_Action(1);
//        Beep_Action(0);
//        road_type.LeftCirque =0;
//        road_type.RightCirque =0;
//        road_type.Cross = 0;
//        road_type.Ramp1=0;
//        Ramp_memory=0;
//        road_type.straight = 0;
//        road_type.bend = 0;
//
//        encoder.encoder_enable=1;
//        zebra_m=encoder.encodersum_average;
////        if(<15000)
////        {
////            road_type.zebra_flag=1;
////        }
////        else
////        {
////            road_type.zebra_flag=0;
////            zebra_crossing_memory=0;
////        }
//    }
//
//}

 uint8 car_state=0;//0 停车  1开始计数    计时到状态2 发车标志位
 uint8 start_count=0;
 uint8 Enable_Element=0;
void car_start111(void)
{
    start_count=0;//计数
     if(road_type.zebra_flag==1&&car_state==0&&start_count==0)
     {
         Enable_Element=0;//元素使能
         car_state=0;//0 停车 开始计数    计时到状态1 发车
         start_count++;
//         Beep_Action(1);
         if(start_count>=30)
         {
             road_type.zebra_flag=0;
             car_state=1;
             start_count=0;
             Enable_Element=1;
             Beep_Action(0);
         }
     }

     if(road_type.zebra_flag==1&&car_state==1)
     {
         car_state=0;
         Enable_Element=0;

         encoder.encoder_enable=1;
         zebra_m=encoder.encodersum_average;
         if(zebra_m<7000)
         {
             road_type.zebra_flag=1;

         }
     }

     if(road_type.zebra_flag==1)
     {

         encoder.encoder_enable=1;
         if(encoder.encodersum_average<7000)
         {
             road_type.zebra_flag=1;
         }
     }

}

void zebra_crossing1(void){
      uint8 i;
      uint8 num1;
      uint8 num2;
      uint8 num3;
      uint8 num4;

      zebra_crossing_memory=0;
      zebra_crossing_flag=0;
      road_type.zebra_flag=0;

      num1=0;
      num2=0;
      num3=0;
      num4=0;

      for(i=3;i<185;i++){
          if(imag[Endline+5][i-1]==White&&imag[Endline+5][i]==Black&&imag[Endline+5][i+1]==Black){
              num4++;
          }
          if(num4>5){
              zebra_crossing_memory=1;
//              zebra_crossing_flag=1;
//              road_type.zebra_flag=1;
          }
      }

      for(i=3;i<185;i++){
          if(imag[Endline+15][i-1]==White&&imag[Endline+15][i]==Black&&imag[Endline+15][i+1]==Black){
              num1++;
          }
          if(num1>5){
              zebra_crossing_memory=2;
//              zebra_crossing_flag=1;
//              road_type.zebra_flag=1;
          }
      }
      for(i=3;i<185;i++){
          if(imag[60][i-1]==White&&imag[60][i]==Black&&imag[60][i+1]==Black){
              num2++;
          }
          if(num2>5){
              zebra_crossing_memory=3;
//              zebra_crossing_flag=1;
//              road_type.zebra_flag=1;
          }
      }
      for(i=3;i<185;i++)
      {
          if(imag[80][i-1]==White&&imag[80][i]==Black&&imag[80][i+1]==Black){
              num3++;
          }
          if(num3>5){
              zebra_crossing_memory=4;
//              zebra_crossing_flag=1;
//              road_type.zebra_flag=1;
          }
      }
      for(i=3;i<185;i++)
      {
          if(imag[100][i-1]==White&&imag[100][i]==Black&&imag[100][i+1]==Black){
              num3++;
          }
          if(num3>5){
              zebra_crossing_memory=5;
              zebra_crossing_flag=1;
              road_type.zebra_flag=1;
          }
      }
      if(road_type.zebra_flag==1)
      {
          road_type.LeftCirque =0;
          road_type.RightCirque =0;
          road_type.Cross = 0;
          road_type.Ramp1=0;
          Ramp_memory=0;
          road_type.straight = 0;
          road_type.bend = 0;

          encoder.encoder_enable=1;
          if(encoder.encodersum_average<7000)
          {
              road_type.zebra_flag=1;
          }

      }
}


uint8 speed1=0.5;
uint8 speed2=0.5;
uint8 Encoder_speed1;
//车库识别
void Garage(void){
    zebra_crossing(60);
    Left_garage_flag=0;
    if(zebra_crossing_flag==1){
        annulus_L_Flag=0;
        annulus_L_memory=0;
        Left_garage_flag=1;
    }
    switch(Left_garage_memory){
        case 0:  //出库
            if(Lower_left_inflection_Flag==0&&Lower_right_inflection_Flag==0){
                Addingline( 2, 3, Endline+3, 184, 117);   //调节此处可调节舵机拐角
            }
            if(Lost_left_Flag==0&&Lost_right_Flag==0&&Lower_left_inflection_Flag==0&&Lower_right_inflection_Flag==0&&Endline<5){
                Left_garage_memory=1;
            }
            break;
        case 1:  //出库后正常行驶
            if(Left_garage_flag==1){
                Left_garage_memory=4;
            }
            break;
        case 2:  //第一次看到车库，补线过库
            speed1=0.5;
            speed2=0.5;
            if(Upper_left_inflection_Flag==1){
                Addingline2(1,Upper_left_inflection_X,Upper_left_inflection_Y);
            }
            else if(Lower_left_inflection_Flag==1){
                Addingline1(1,Lower_left_inflection_X,Lower_left_inflection_Y);
            }
            else{
                Addingline( 1, 29,67, 55, 25);  //根据实际调整
            }
            if(Lost_left_Flag==0&&Left_garage_flag==0&&Upper_left_inflection_Flag==0&&zebra_crossing_flag==0&&Lower_left_inflection_Flag==0){
                Left_garage_memory=3;
            }

            break;
        case 3:  //正常行驶
            speed1=1.5;
            speed2=1.5;
            if(Left_garage_flag==1){
                Left_garage_memory=4;

            }
            break;
        case 4: //再次看到车库时补线入库
            speed1=-1.7;
            speed2=-1.7;
            Addingline( 2, Upper_left_inflection_X,Upper_left_inflection_Y,right[119] , 119);
            if((Lost_left_Flag==0&&Lost_right_Flag==0)||Endline>50){
                Left_garage_memory =5;
            }
            break;
        case 5: //刹车
            speed1=-2.2;
            speed2=-2;
            if(Encoder_speed1<-5){
                Left_garage_memory=7;
            }
            break;
        case 6://刹车
            speed1=-0.4;
            speed2=-0.2;
            if(Encoder_speed1<0){
                Left_garage_memory=7;
            }
            break;
        case 7:
            speed1=-0.01;
            speed2=0;
            break;
    }

}
//---------------------------------------线性拟合---------------------------------
void regression(int type, int startline, int endline)//最小二乘法拟合曲线，分别拟合中线，左线，右线,type表示拟合哪几条线
 {
     int i = 0;
     int sumlines = endline - startline;
     int sumX = 0;
     int sumY = 0;
     float averageX = 0;
     float averageY = 0;
     float sumUp = 0;
     float sumDown = 0;
     if (type == 0)      //拟合中线
     {
         for (i = startline; i < endline; i++)
         {
             sumX += i;
             sumY += middle[i];
         }
         if (sumlines != 0)
         {
             averageX = (float)(sumX / sumlines);     //x的平均值
             averageY = (float)(sumY / sumlines);     //y的平均值
         }
         else
         {
             averageX = 0;     //x的平均值
             averageY = 0;     //y的平均值
         }
         for (i = startline; i < endline; i++)
         {
             sumUp += (middle[i] - averageY) * (i - averageX);
             sumDown += (i - averageX) * (i - averageX);
         }
         if (sumDown == 0) parameterB = 0;
         else parameterB = sumUp / sumDown;
         parameterA = averageY - parameterB * averageX;
     }
     else if (type == 1)//拟合左线
     {
         for (i = startline; i < endline; i++)
         {
             sumX += i;
             sumY += left[i];
         }
         if (sumlines == 0) sumlines = 1;
         averageX = (float)(sumX / sumlines);     //x的平均值
         averageY = (float)(sumY / sumlines);     //y的平均值
         for (i = startline; i < endline; i++)
         {
             sumUp += (left[i] - averageY) * (i - averageX);
             sumDown += (i - averageX) * (i - averageX);
         }
         if (sumDown == 0) parameterB = 0;
         else parameterB = sumUp / sumDown;
         parameterA = averageY - parameterB * averageX;
     }
     else if (type == 2)//拟合右线
     {
         for (i = startline; i < endline; i++)
         {
             sumX += i;
             sumY += right[i];
         }
         if (sumlines == 0) sumlines = 1;
         averageX = (float)(sumX / sumlines);     //x的平均值
         averageY = (float)(sumY / sumlines);     //y的平均值
         for (i = startline; i < endline; i++)
         {
             sumUp += (right[i] - averageY) * (i - averageX);
             sumDown += (i - averageX) * (i - averageX);
         }
         if (sumDown == 0) parameterB = 0;
         else parameterB = sumUp / sumDown;
         parameterA = averageY - parameterB * averageX;

     }
 }

void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2)//0拟合中线  //1拟合左线  //2拟合右线
 {
    int i = 0;
    int sumlines1 = endline1 - startline1;
    int sumlines2 = endline2 - startline2;
    int sumX = 0;
    int sumY = 0;
    float averageX = 0;
    float averageY = 0;
    float sumUp = 0;
    float sumDown = 0;
    if (type == 0)  //拟合中线
    {
        /**计算sumX sumY**/
        for (i = startline1; i <=endline1; i++)
        {
            sumX += i;
            sumY += middle[i];
        }
        for (i = startline2; i <=endline2; i++)
        {
            sumX += i;
            sumY += middle[i];
        }
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x的平均值
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y的平均值
        for (i = startline1; i <= endline1; i++)
        {
            sumUp += (middle[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i <= endline2; i++)
        {
            sumUp += (middle[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;

    }
    else if (type == 1)     //拟合左线
    {
        /**计算sumX sumY**/
        for (i = startline1; i <= endline1; i++)
        {
            sumX += i;
            sumY += left[i];
        }
        for (i = startline2; i <= endline2; i++)
        {
            sumX += i;
            sumY += left[i];
        }
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x的平均值
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y的平均值
        for (i = startline1; i <= endline1; i++)
        {
            sumUp += (left[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i <= endline2; i++)
        {
            sumUp += (left[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 2)         //拟合右线
    {
        /**计算sumX sumY**/
        for (i = startline1; i <= endline1; i++)
        {
            sumX += i;
            sumY += right[i];
        }
        for (i = startline2; i <= endline2; i++)
        {
            sumX += i;
            sumY += right[i];
        }
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x的平均值
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y的平均值
        for (i = startline1; i <= endline1; i++)
        {
            sumUp += (right[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i <= endline2; i++)
        {
            sumUp += (right[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
}
//------------------------------判断直线斜率是否相同------------------------------
int Judgment_symbol(float x, float y)
{
    int a;
    a = 0;
    if (x < 0 && y < 0) a = 1;
    if (x >= 0 && y >= 0) a = 1;
    return a;
}
//-----------------------------------------补线-----------------------------------

//有起点和终点的补线函数  1左   2右
void Addingline( uint8 choice, uint8 startX, uint8 startY, uint8 endX, uint8 endY)
{
    y = 0;

    // 直线 x = ky + b
    float k = 0;
    float b = 0;
    switch(choice)
    {
      case 1://左补线
        {
            k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
            b = (float)startX - (float)startY * k;

            for(y = startY; y < endY; y++)
            {
                left_dispose[y] = (uint8)(k * y + b);
            }
            break;
        }

      case 2://右补线
        {
            k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
            b = (float)startX - (float)startY * k;

            for(y = startY; y < endY; y++)
            {
                right_dispose[y]= (uint8)(k * y + b);

            }
            break;
        }

    }
}


void Addingline1( uint8 choice, uint8 startX, uint8 startY)    //看到拐点延斜率向上延长
{

    // 直线 x = ky + b
    float k = 0;
    float b = 0;
    int temp=0;
    switch(choice)
    {
      case 1://左补线
        {

            k = (float)(((float)left[Lower_left_inflection_Y+10] - (float)left[Lower_left_inflection_Y+15]) /(-10));
            b = (float)((float)left[Lower_left_inflection_Y+10]- (float)(Lower_left_inflection_Y+15) * k);

            for(y = startY; y >(Endline+20); y--)
            {
             temp = (int)(k* y + b);
             if(temp<180&&temp>10){
                 left_dispose[y]=temp;
             }
            }
            break;
        }

      case 2://右补线  待测试
      {

           k = (float)(((float)right[Lower_right_inflection_Y+10] - (float)right[Lower_right_inflection_Y+15]) /(-10));
           b = (float)((float)right[Lower_right_inflection_Y+10]- (float)(Lower_right_inflection_Y+15) * k);

           for(y = startY; y >(Endline+20); y--)
           {

            temp = (int)(k* y + b);
            if(temp<180&&temp>10){
                right_dispose[y]=temp;
            }
           }
           break;
       }

    }
}

void Addingline2( uint8 choice, uint8 startX, uint8 startY)   //找到上拐点延斜率向下拉线
{

    // 直线 x = k*y + b
    float k = 0;
    float b = 0;
    int temp=0;
    switch(choice)
    {
      case 1://左补线
        {

            k = (float)(((float)left[Upper_left_inflection_Y-5] - (float)left[Upper_left_inflection_Y-15]) /10);
            b = (float)((float)left[Upper_left_inflection_Y-5]- (float)(Upper_left_inflection_Y-15) * k);

            for(y = startY-1; y<(image_h-3); y++)
            {

             temp = (int)(k* y + b);
             if(temp<185&&temp>2){
                 left_dispose[y]=temp;
             }

            }
            break;
        }

     case 2://右补线  待测试
        {

            k = (float)(((float)right[Upper_right_inflection_Y-2] - (float)right[Upper_right_inflection_Y-5]) /3);
            b = (float)((float)right[Upper_right_inflection_Y-5]- (float)(Upper_right_inflection_Y-5) * k);
            for(y = startY; y<(image_h-3); y++)
            {

             temp = (int)(k* y + b);
             if(temp<185&&temp>2){
                 right_dispose[y]=temp;
             }

            }
            break;
        }

    }
}



//===================================================显  示===================================================
void IPS_show(void){
   int i;
  //********************图像显示*****************************
ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);

  //********************边线显示*****************************
        //根据最终循环次数画出边界点
        for (i = 0; i < data_stastics_l; i++)
        {
            ips200_draw_point(0, 0, RGB565_RED);
            ips200_draw_point(points_l[i][0]+2, points_l[i][1],  RGB565_GREEN);
        }
        for (i = 0; i < data_stastics_r; i++)
        {
            ips200_draw_point(points_r[i][0]-2, points_r[i][1],  RGB565_GREEN);
        }

        for (i = Endline; i < image_h-1; i++)
        {
          //  middle[i] = (left[i] + right[i]) >> 1;//求中线

            //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
            //当然也有多组边线的找法，但是个人感觉很繁琐，不建议
            ips200_draw_point(middle[i], i,  RGB565_GREEN);
            ips200_draw_point(left[i], i, RGB565_RED);
            ips200_draw_point(right[i], i, RGB565_BLUE);

        }
          if(Endline<115){
              for(int i=119;i>Endline;i--)
               {
                   //ips200_draw_line(middle[scanning_line], 1, middle[scanning_line], 119, RGB565_RED); //中线
                   //ips200_draw_line(left[scanning_line], 1, left[scanning_line], 119, RGB565_CYAN); //左控制线
                   //ips200_draw_line(right[scanning_line], 1, right[scanning_line], 119, RGB565_YELLOW);  //右控制线
                   if(Endline>1&&Endline<120)
                     {
                       ips200_draw_line(1, Endline, 187, Endline,RGB565_GREEN); //截止行
                     }
                }
          }



  //********************拐点显示*****************************
  //---------------左下拐点--------------------
  if(Lower_left_inflection_Flag==1){
      ips200_show_string(0,125,"low left");
      ips200_show_int(0, 140, Lower_left_inflection_X,3);
      ips200_show_int(0, 155, Lower_left_inflection_Y,3);
      ips200_draw_point(Lower_left_inflection_X, Lower_left_inflection_Y,  RGB565_BLUE);
  }
  //---------------右下拐点-------------------
  if(Lower_right_inflection_Flag==1){
      ips200_show_string(0,170,"low right");
      ips200_show_int(0, 185, Lower_right_inflection_X,3);
      ips200_show_int(0, 200, Lower_right_inflection_Y,3);
      ips200_draw_point(Lower_right_inflection_X, Lower_right_inflection_Y,  RGB565_BLUE);
  }
  //---------------左上拐点-------------------
  if(Upper_left_inflection_Flag==1){
      ips200_show_string(100,125,"upper left");
      ips200_show_int(100,140, Upper_left_inflection_X,3);
      ips200_show_int(100,155, Upper_left_inflection_Y,3);
      ips200_draw_point(Upper_left_inflection_X, Upper_left_inflection_Y,  RGB565_RED);
  }
  //---------------右上拐点--------------------
  if(Upper_right_inflection_Flag==1){
      ips200_show_string(100,170,"upper right1");
      ips200_show_int(100, 185,Upper_right_inflection_X,3);
      ips200_show_int(100, 200, Upper_right_inflection_Y,3);
      ips200_draw_point(Upper_right_inflection_X, Upper_right_inflection_Y,  RGB565_RED);
  }

  //********************环岛*****************************
//  if(roundabout_Flag==1){
//      ips200_show_string(130,215,"roundabout");
//      ips200_show_int(130, 230,roundabout_X,3);
//      ips200_show_int(130, 245, roundabout_Y,3);
//      ips200_draw_point(roundabout_X, roundabout_Y,  RGB565_RED);
//  }
  //********************出环岛口*****************************
//  if(Exit_loop_Flag==1){
//      ips200_show_string(130,255,"exit");
//      ips200_show_int(130, 270,Exit_loop_X,3);
//      ips200_show_int(130, 285, Exit_loop_Y,3);
//      ips200_draw_point(Exit_loop_X, Exit_loop_Y,  RGB565_RED);
//  }
  //********************右直线*****************************
  if( Right_straight_flag==1){
      ips200_show_string (0, 215, "right straight1");
//      ips200_show_float(0, 260,k1,3,3);
//      ips200_show_float(70, 260,k2,3,3);
//      ips200_show_float(140, 260,k3,3,3);
  }
  else{
      ips200_show_string (0, 215, "right straight0");
  }

  //********************圆环*****************************
  ips200_show_string (0,230, "annulus_L");
  ips200_show_int (60,230, annulus_L_memory,3);

  //********************十字*****************************
  ips200_show_string (100,250, "Crossroad");
  ips200_show_int (170,250, Crossroad_memory,3);

  //********************左车库*****************************
  ips200_show_string (0,245, "Garage");
  ips200_show_int (50,245, Left_garage_memory,3);
  //********************左丢线显示*****************************
 // ips200_show_float(0, 275,Lost_point_L_scan_line,3,3);

  //********************斑马线*****************************
  ips200_show_string (0,260, "zebra");
  ips200_show_int (50,260, zebra_crossing_flag,3);

//  //********************速度*****************************
//  ips200_show_float(0, 275, speed1,3,3);
//  ips200_show_float(60, 275,speed2,3,3);

  //********************编码器显示*****************************
//  ips200_show_int(0, 275, speed2, 5);
//  ips200_show_int(60,275, motor2.pid_actual_val, 5);
//  ips200_show_int(0, 290, speed1, 5);
//  ips200_show_int(60,290, motor1.pid_actual_val, 5);
  //********************调试显示*****************************
//ips200_show_int(100, 290,Endline,3);
//  ips200_show_float(100, 275,Lost_point_R_scan_line,3,3);
// ips200_draw_point(Upper_left_inflection_X, Upper_left_inflection_Y,  RGB565_BLUE);
// ips200_draw_point(Upper_right_inflection_X, Upper_right_inflection_Y,  RGB565_BLUE);



 //ips200_show_int (15,275, Lost_right_Flag,3);



}
void drawing(void)
{
    ips200_clear();
    // ********************误差*****************************
    ips200_show_string (0, 125, "Error1");
    ips200_show_float(50, 125, Error1, 4, 2);

    ips200_show_string (100, 125, "End");
    ips200_show_int(140, 125, Endline, 3);

    ips200_show_int( 190, 125,sudu_yingzi ,3);//
    // ********************画线*****************************
    ips200_draw_line(2,end_line,185,end_line,RGB565_BLUE);
    ips200_draw_line(2,start_line,185,start_line,RGB565_BLUE);

    ips200_draw_line(94,10,94,118,RGB565_RED);

    ips200_draw_line(2,100,185,100,RGB565_BLACK);
//    ips200_draw_line(2,50,185,50,RGB565_RED);

//    uint8 left1=left[Lower_left_inflection_Y-1],left2=left[Lower_left_inflection_Y-2],
//          left3=left[Upper_left_inflection_Y+1],left4=left[Upper_left_inflection_Y+2];
//
//     ips200_show_uint(190,0,left1,3);
//    ips200_show_uint(190,15,left2,3);
//    ips200_show_uint(190,40,left3,3);
//   ips200_show_uint(190,55,left4,3);

//    uint8 mid_Error=94-(left[118]+right[118])/2;//35    147
//    ips200_show_string (190, 75, "R");
//    ips200_show_int( 200, 75, 94-right[118],3);

    ips200_show_string (190, 45, "ZM");
    ips200_show_int( 190, 60, zebra_m,3);

    ips200_show_string (190, 75, "TC");
    ips200_show_int( 190, 90, time_count,3);

    ips200_show_string (190, 105, "S");
    ips200_show_int( 200, 105, stop,3);


    uint16 col1=40,col2=70,col3=100,col4=130,col5=160,col6=190;
    ips200_show_string (0, 140, "Type");
    ips200_show_string(col1, 140, "DX");
    ips200_show_string(col2, 140, "BDX");
    ips200_show_string(col3, 140, "SGD");
    ips200_show_string(col4, 140, "XGD");

    ips200_show_string(col5, 140, "MGX");
    ips200_show_string(col6, 140, "MGY");
//    ips200_show_string(215, 140, "ZX");


    ips200_show_string (0, 155, "L");
    ips200_show_int( col1, 155, data_stastics_l,3);
    ips200_show_int( col2, 155, left_num,3);
    ips200_show_int( col3, 155, Upper_left_inflection_Flag,3);
    ips200_show_int( col4, 155, Lower_left_inflection_Flag,3);
    ips200_show_int(col5, 155, roundabout_X, 3);
    ips200_show_int(col6, 155, roundabout_Y, 3);
//    ips200_show_int(215, 155, Left_straight_flag, 3);


    ips200_show_string (0, 170, "R");
    ips200_show_int( col1, 170, data_stastics_r,3);//right_lost_num
    ips200_show_int( col2, 170, right_num,3);
    ips200_show_int( col3, 170, Upper_right_inflection_Flag,3);
    ips200_show_int( col4, 170, Lower_right_inflection_Flag,3);

//    ips200_show_int(col5, 170, right[118], 3);
//    ips200_show_int(col6, 170, right[117], 3);
//    ips200_show_int(215, 155, Right_straight_flag, 3);

    uint16 Type=190,Flag=205,Memo=220;
    ips200_show_string (0,Type, "Type");
    ips200_show_string (0,Flag, "Flag");
    ips200_show_string (0,Memo, "Memo");

    ips200_show_string (col1,Type, "BMX");
    ips200_show_string (col2,Type, "SZ");
    ips200_show_string (col3,Type, "LH");
    ips200_show_string (col4,Type, "RH");
    ips200_show_string (col5,Type, "LB");
    ips200_show_string(col5, Type, "LS");
    ips200_show_string(col6+20, Type, "RS");

//    ips200_show_string (col6,Type, "En");
//    ips200_show_string (col6+20,Type, "ST");



    ips200_show_uint(col1,Flag,road_type.zebra_flag,3);
    ips200_show_uint(col1,Memo,zebra_crossing_memory,3);

    ips200_show_uint(col2,Flag,road_type.Cross,3);
    ips200_show_int (col2,Memo, Crossroad_memory,3);

    ips200_show_uint(col3,Flag,road_type.LeftCirque,3);
    ips200_show_int (col3,Memo, annulus_L_memory,3);

    ips200_show_int (col4,Flag, annulus_R_memory,3);
    ips200_show_uint(col4,Memo,road_type.RightCirque,3);

    ips200_show_uint(col5,Flag,l_up_bizhang_flag,3);
    ips200_show_uint(col5,Memo,r_up_bizhang_flag,3);

    ips200_show_uint (col6,Flag,road_type.L_Cross,3);
    ips200_show_uint(col6+20,Memo, L_Crossroad_memory,3);

    ips200_show_uint (col6,Flag, road_type.R_Cross,3);
    ips200_show_uint (col6+20,Memo,8,3);

//    ips200_show_uint (col6,Flag,Enable_Element,3);
//    ips200_show_uint (col6+20,Flag, 0,3);


    ips200_show_string (0,240, "Type");
    ips200_show_string (0,255, "L");
    ips200_show_string (0,270, "R");

    ips200_show_string (col1,240, "ZX");
    ips200_show_string (col2,240, "TB");


    ips200_show_uint(col1,255,Left_straight_flag,3);
    ips200_show_uint(col2,255,L_tubian_flag,3);

    ips200_show_uint(col1,270,Right_straight_flag,3);
    ips200_show_uint(col2,270,R_tubian_flag,3);

    ips200_show_uint(col1,290,start_line,3);
    ips200_show_uint(col2,290,end_line-40,3);

//    uint8 xiangliang1 = (left[y]-left[y+4])*(left[y-4]-left[y])-16;
//    uint8 xiangliang2 = (right[y]-right[y+4])*(right[y-4]-right[y])-16;
//    ips200_show_int(col1,290,xiangliang1,3);
//    ips200_show_int(col2,290,xiangliang2,3);


    ips200_show_string (col3,240, "LSGD");
    ips200_show_string (col3,275, "RSGD");

    ips200_show_string (col5,240, "LXGD");
    ips200_show_string (col5,275, "RXGD");
    //                 L                                      //R
    ips200_show_uint(col3,255,Upper_left_inflection_X,3);  ips200_show_uint(col4,255,Upper_left_inflection_Y,3);
    ips200_show_uint(col3,290,Upper_right_inflection_X,3); ips200_show_uint(col4,290,Upper_right_inflection_Y,3);

    ips200_show_uint(col5,255,Lower_left_inflection_X,3);  ips200_show_uint(col6,255,Lower_left_inflection_Y,3);
    ips200_show_uint(col5,290,Lower_right_inflection_X,3); ips200_show_uint(col6,290,Lower_right_inflection_Y,3);
//        ips200_show_string (0,290, "imu");
//        ips200_show_float(80,290,eulerAngle.pitch,3,3);
//        ips200_show_string (0,270, "imx");
//        ips200_show_int(100,270,imu660ra_acc_x,5);
//
//        ips200_show_int(0, 170, calkuan[120], 3);
//        ips200_show_int(50, 170, calkuan[118], 3);
//
//        ips200_show_string (0, 170, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
}

//计算宽度开始

void cakkuandu(void)
{
        int i;
        for (i = 120; i >=Endline; i--)
        {
            calkuan[i] = 0;
        }
        for (i = 120; i >= Endline; i--)
        {
            calkuan[i] = right[i]-left[i];
            if(calkuan[i]>120)
            calkuan[i]=120;
        }

//        ips200_show_uint(0,150,calkuan[40],4);
//        ips200_show_uint(50,150,calkuan[60],4);
//        ips200_show_uint(100,150,calkuan[80],4);


}
//
//uint8 flag_centre_right_point = 0;
//uint8 flag_centre_left_point = 0;
//uint8 centre_left_point0 = 0;
//uint8 centre_left_point1 = 0;
//uint8 centre_right_point0 = 0;
//uint8 centre_right_point1 = 0;
//uint8 breakwrong=0;
//void mysmalls(void)
//{
//    int i;
//
//#define LCenter  middle
//
//    flag_centre_right_point = 0;
//    flag_centre_left_point = 0;
//    centre_left_point0= 0;
//    centre_left_point1= 0;
//    centre_right_point0= 0;
//    centre_right_point1= 0;
//    //开始找起始点//
//
//     breakwrong = 0;
//    for (i = 110; i >= 70; i--)
//    {
//        if (IMy_Abs(LCenter[i], LCenter[i - 1]) >= 8)
//        {
//            breakwrong = 1;
//            break;
//        }
//    }
//    for (i = 110; i <= Endline; i--)
//    {
//        if (i>70&& flag_centre_right_point == 0
//        && (LCenter[i - 5] - LCenter[i - 4]) >= 0
//        && (LCenter[i - 6] - LCenter[i - 4]) >= 0
//        && (LCenter[i - 7] - LCenter[i - 4]) >= 0
//        && (LCenter[i - 8] - LCenter[i - 4]) >= 0
//        && (LCenter[i - 8] - LCenter[i - 4]) <= 20                        //判断j-4是否为中线拐点，拐点在左边的那个
//        && (LCenter[i - 3] - LCenter[i - 4]) >= 0
//        && (LCenter[i - 2] - LCenter[i - 4]) >= 0
//        && (LCenter[i - 2] - LCenter[i - 4]) <= 20
//        && (LCenter[i - 1] - LCenter[i - 4]) >= 1
//        && (LCenter[i] - LCenter[i - 4]) >= 1)
//        {
//            centre_right_point[0] = (int)(i - 3);
//            centre_right_point[1] = LCenter[i - 4];
//            flag_centre_right_point = 1;
//        }
//        else if ( i <70 && flag_centre_right_point == 0
//                && (LCenter[i - 3] - LCenter[i - 2]) >= 0
//        && (LCenter[i - 4] - LCenter[i - 2]) >= 0
//        && (LCenter[i - 5] - LCenter[i - 2]) >= 0
//        && (LCenter[i - 6] - LCenter[i - 2]) >= 0
//        && (LCenter[i - 6] - LCenter[i - 2]) <= 30                        //判断j-4是否为中线拐点，拐点在左边的那个
//        && (LCenter[i - 1] - LCenter[i - 2]) >= 0)
//        {
//            centre_right_point[0] = (int)(i - 2);
//            centre_right_point[1] = LCenter[i - 2];
//            flag_centre_right_point = 1;
//        }
//        //中线左拐点
//        if (i >70 && flag_centre_left_point == 0
//        && (LCenter[i - 5] - LCenter[i - 4]) <= 0
//        && (LCenter[i - 6] - LCenter[i - 4]) <= 0
//        && (LCenter[i - 7] - LCenter[i - 4]) <= 0
//        && (LCenter[i - 8] - LCenter[i - 4]) <= 0
//        && (LCenter[i - 8] - LCenter[i - 4]) >= -20       //判断j-4是否为中线拐点，拐点在左边的那个
//        && (LCenter[i - 3] - LCenter[i - 4]) <= 0
//        && (LCenter[i - 2] - LCenter[i - 4]) <= 0
//        && (LCenter[i - 2] - LCenter[i - 4]) >= -20
//        && (LCenter[i - 1] - LCenter[i - 4]) <= -1
//        && (LCenter[i] - LCenter[i - 4]) <= -1)
//        {
//            centre_left_point0 = (int)(i - 3);
//            centre_left_point1= LCenter[i - 4];
//            flag_centre_left_point = 1;
//        }
//        else if ( i <70&& flag_centre_left_point == 0
//        && (LCenter[i - 3] - LCenter[i - 2]) <= 0
//        && (LCenter[i - 4] - LCenter[i - 2]) <= 0
//        && (LCenter[i - 5] - LCenter[i - 2]) <= 0
//        && (LCenter[i - 6] - LCenter[i - 2]) <= 0
//        && (LCenter[i - 6] - LCenter[i - 2]) >= -30       //判断j-4是否为中线拐点，拐点在左边的那个
//        && (LCenter[i - 1] - LCenter[i - 2]) <= 0
//        && LCenter[i - 2] >= 19 && LCenter[i - 2] <= 165)
//        {
//            centre_left_point0 = (int)(i - 2);
//            centre_left_point1= LCenter[i - 2];
//            flag_centre_left_point = 1;
//        }
//    }
////    if (flag_centre_left_point == 1)
////    {
////
////    }
////    if (flag_centre_right_point == 1)
////    {
////
////    }
////    turesmallsflag = 0;
//
//    if (flag_centre_left_point == 1 && flag_centre_right_point == 1 && IMy_Abs(centre_right_point, centre_left_point) >= 4
//        && IMy_Abs(centre_right_point, centre_left_point) <= 50 && (centre_right_point + centre_left_point) > 90
//        &&calkuan[110]<186 && calkuan[100] < 186 && clkuan[95] < 186 && calkuan[90] < 186 && calkuan[85] < 186
//        )
//    {
//        road_type.smalls = 0;
//    }
//
////
////    smallstype1 = 0;
////    smallstype2 = 0;
////    if (turesmallsflag == 1 && break_hangshu > 43)
////    {
////        smallstype1 = 1;
////    }
////    if (turesmallsflag == 1 && break_hangshu <=43)
////    {
////        smallstype2 = 1;
////    }
////    if (threemode != 0 || threemode != 6 || threemode != 12)
////    {
////        turesmallsflag = 0;
////    }
//}



//===================================================元素识别===================================================
int16 l_line_qulv=0, r_line_qulv=0;
void Element_recognition(void)
{
    inflection_point();
//    wandaopanduan();
    left_straight();
    right_straight();//右直线识别
//         banmaxian(110,60);
//    Garage();
//    Ramp_identification();
//    uint8 wandao=Mid_Col(30);
//    识别圆环
//    l_line_qulv = 1000*Process_Curvity(left[MT9V03X_H-10], MT9V03X_H-10, left[MT9V03X_H-25], MT9V03X_H-25, left[MT9V03X_H-40], MT9V03X_H-40);&& fabs(l_line_qulv) <4&&bianxian_tubian(0)&&imag[Lower_left_inflection_Y+2][Lower_left_inflection_X-20]==White
//    r_line_qulv = 1000*Process_Curvity(right[MT9V03X_H-10], MT9V03X_H-10, right[MT9V03X_H-25], MT9V03X_H-25, right[MT9V03X_H-40], MT9V03X_H-40);&& fabs(r_line_qulv) <4)&&bianxian_tubian(1)

     cakkuandu();//计算的赛道宽度
     Check_Zhidao();
     Zhidao();//length==80&&
     Ramp_fuzhu();//坡道辅助识别&&bianxian_tubian(0)==0&&bianxian_tubian(1)==0
     L_R_tubian();//判断边线是否突变  0 不突变   1突变
      stop=0;
//     car_start();
    if(stop==1)
    {
     if(sudu_yingzi>70&&Lost_left_Flag==0&&Lost_right_Flag==0)//&&L_tubian_flag==0&&R_tubian_flag==0
     {
         zebra_crossing(60);
     }
    }
//     car_start();
//    if(Enable_Element==1)
//     road_type.zebra_flag=1;
     if( road_type.zebra_flag==0)
    {
//         l_bizhang();
        //Lower_left_inflection_X-20&&left[Lower_left_inflection_Y-1]==2&&imag[Lower_left_inflection_Y+2][Lower_left_inflection_X-20]==White&&imag[Lower_left_inflection_Y][2]==White
        if(Lost_left_Flag==1&&Lower_left_inflection_Flag==1&&L_tubian_flag==1&&R_tubian_flag==0&&Right_straight_flag==1&&road_type.LeftCirque ==0&&road_type.L_Cross ==0
              &&Upper_right_inflection_Flag==0&&Lower_right_inflection_Flag==0&&annulus_L_Flag==0&&Lower_left_inflection_Y>50)
        {//左环

             road_type.LeftCirque =1;
             annulus_L_Flag=1;
             annulus_L_memory =1;

             road_type.RightCirque =0;
             annulus_R_Flag=0;
             annulus_R_memory =0;

             road_type.L_Cross =0;
             L_Crossroad_Flag=0;

             road_type.Cross = 0;
             Crossroad_Flag=0;
             printf("左圆环标志位：1\n");
        }
//            else if(Lower_right_inflection_Flag==1&&Right_straight_flag==0&&road_type.LeftCirque ==0 &&road_type.L_Cross ==0 )
//            {//左入十字
//                road_type.LeftCirque =0;
//                annulus_L_Flag=0;
//                annulus_L_memory =0;
//
//                road_type.RightCirque =0;
//                annulus_R_Flag=0;
//                annulus_R_memory =0;
//
//                road_type.L_Cross =1;
//                L_Crossroad_Flag=1;
//
//                road_type.Cross = 0;
//                Crossroad_Flag=0;
//            }
     //右圆环判断&&right[Lower_right_inflection_Y-1]==185
      else if(Lost_right_Flag==1&&Crossroad_Flag==0&&L_tubian_flag==0&&R_tubian_flag==1&&Left_straight_flag==1&&Lower_right_inflection_Flag==1
             &&Lower_left_inflection_Flag==0&&Upper_left_inflection_Flag==0&&road_type.RightCirque==0&&Lower_right_inflection_Y>50)
//             &&imag[Lower_right_inflection_Y][184]==White&&imag[Lower_right_inflection_Y+2][184]==White&&imag[Lower_left_inflection_Y+2][Lower_left_inflection_X+10]==White&&imag[Lower_right_inflection_Y+1][Lower_right_inflection_X+20]==White
      {

         road_type.RightCirque =1;
         annulus_R_Flag=1;
         annulus_R_memory =1;

         road_type.LeftCirque =0;
         annulus_L_Flag=0;
         annulus_L_memory =0;

         road_type.Cross = 0;
         Crossroad_Flag=0;
         Crossroad_memory=0;

         Ramp_memory=0;
         road_type.Ramp1=0;

         road_type.straight = 0;
         road_type.bend = 0;
           printf("右圆环标志位：1\n");
       }

//     坡道判断
//      else if(road_type.Ramp1==0&&calkuan[60]>79&&LK>=-0.32&&LK0end>=-0.30&&RK<0.35&&RK0end<0.30&&kerrorL>-0.03&&kerrorR<0.100)
//   {
//
//       road_type.Ramp1=1;
//       Ramp_memory=1;
//
//       road_type.LeftCirque =0;
//       annulus_L_Flag=0;
//       annulus_L_memory =0;
//
//       road_type.RightCirque =0;
//       annulus_R_Flag=0;
//       annulus_R_memory =0;
//
//       road_type.Cross = 0;
//       Crossroad_Flag=0;
//
//       road_type.straight = 0;
//       road_type.bend = 0;
//       printf("坡道标志位：1\n");
//
//   }
     //提前十字处理
      else if((Lower_left_inflection_Flag==1&&Lower_right_inflection_Flag==1)&&Lost_left_Flag==1&&Lost_right_Flag==1&&Crossroad_Flag==0&&road_type.Ramp1==0&&road_type.RightCirque ==0&&road_type.LeftCirque == 0&&road_type.Cross ==0)
      {

        road_type.Cross = 1;
        Crossroad_Flag=1;
        Crossroad_memory=1;

        road_type.LeftCirque =0;
        annulus_L_Flag=0;
        annulus_L_memory =0;

        road_type.RightCirque =0;
        annulus_R_Flag=0;
        annulus_R_memory =0;

        Ramp_memory=0;
        road_type.Ramp1=0;

        road_type.straight = 0;
        road_type.bend = 0;
        printf("十字标志位：1\n");
    }
//     &&wandao==2
    else if(zhidao_flag==1&&road_type.LeftCirque ==0&&road_type.RightCirque ==0
            &&road_type.Cross == 0)
    {
        road_type.straight =1;
        road_type.bend = 0;
        printf("直线标志位：1\n");
    }
//      else if(wandao_flag==1)
//    {
//        road_type.straight =0;
//        road_type.bend = 1;
//        printf("弯道标志位：1\n");
//    }

//    else  if(road_type.LeftCirque ==0&&road_type.RightCirque ==0
//            &&road_type.Cross == 0&& road_type.straight == 0)
//        {
//            road_type.straight =0;
//            if(wandao==0)
//            {
//                road_type.L_bend = 1;
//                road_type.R_bend = 0;
//
//            }
//            if(wandao==1)
//            {
//                road_type.L_bend = 0;
//                road_type.R_bend = 1;
//            }
//            printf("弯道标志位：1\n");
//        }
    }
}
//元素处理
void Element_Handle(void)
{
    /***************圆环*********************/
//
//    if(road_type.L_bihznag)
//    {
//        l_bizhang();
//    }
//    else
        if(road_type.LeftCirque)
    {
//        track_line(0,119,60);
        annulus_L();
        printf(" 左圆环" );
    }
//    else if(road_type.L_Cross ==1)
//    {
//
//    }
    else if(road_type.RightCirque)
    {
        annulus_R();
        printf(" 右圆环" );
    }
//    else if(road_type.Ramp1)// 坡道
//    {
////        Ramp_Handle();
//        Ramp_Handle_m();
//        printf("坡道" );
//    }
    else if(road_type.Cross)// 十字
    {
//        cross_fill(imag,left, right, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r);
        crossroad2();
//        crossroad();
        printf("十字" );
    }


//    else if(road_type.straight)
//    {
//        if(sudu_yingzi<65)
//        {
//            road_type.straight=0;
//        }
//    }
//    else if(road_type.R_bend)
//    {
//        road_type.R_bend=0;
//    }
//    else if(road_type.L_bend)
//    {
//        road_type.L_bend=0;
//    }
}





//===================================================图像处理===================================================

//全部初始化
void All_Init(void)
{
    mt9v03x_init();//摄像头初始化

//    ips200_init(IPS200_TYPE_SPI);
    ips200_init(IPS200_TYPE_PARALLEL8);//双排针ips初始化
    ips200_set_dir (IPS200_PORTAIT);//显示设置方向为竖屏模式

    gpio_init(P33_10,GPO,0,GPO_PUSH_PULL);//蜂鸣器初始化 State 1 响    State  0 不响
}

//
//void image_process(void)
//{
//        int i=0;
//        if(mt9v03x_finish_flag){
//            // 每帧图像数据清除
//            //system_start ();
//           // gpio_set_level(P14_0, 0);
//            camra_date_clean();
//            int time1=system_getval_ms();
////            Get_image(mt9v03x_image);//图像转存
////            binaryzation();
////            Get01change_Dajin( original_image[0],imag[0],150);//图像二值化
////            Sobel_Adpt_origin(mt9v03x_image,imag,Thres);   //soble边缘二值化
//            Get01change_Dajin(mt9v03x_image[0],imag[0],150);//图像二值化
////            Pixle_Filter(imag);// 图像滤波
////            image_filter(imag);//滤波
//            image_draw_rectan(imag);//预处理
//            //清零
//            data_stastics_l = 0;
//            data_stastics_r = 0;
//            if (get_start_point(image_h - 2))//找到起点了，再执行八领域，没找到就一直找
//            {
//                //printf("正在开始八领域\n");
//               search_l_r((uint16)USE_num, imag, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &Endline);
//                //printf("八邻域已结束\n");
//                get_left(data_stastics_l);
//                get_right(data_stastics_r);
//                lost_left();
//                lost_right();
//                cakkuandu();//计算的赛道宽度
////                bizhang();
//                Element_recognition(); //元素识别
//                Element_Handle(); //元素处理
//                middle_line();
//                Finish_Flag=1;
//            }
//            Calculate_Error_1();
//
////            for(i=120;i>0;i--)
////            {
////                printf("left[%d]:%d\n",i,left[i]);
////            }
////            for(i=120;i>0;i--)
////            {
////                printf("right[%d]:%d\n",i,right[i]);
////            }
////            for(i=120;i>0;i--)
////            {
////                printf("points_l[%d]:%d\n",i,points_l[i]);
////            }
//
////            ips200_show_int(40, 130, calkuan[60], 3);
////            ips200_show_int(100, 130, calkuan[100], 3);
//            ips200_draw_line(2,100,185,100,RGB565_RED);
////          IPS_show();
////            drawing();
////            ips200_draw_line(2,110,185,110,RGB565_RED);
////            ips200_draw_line(2,60,185,60,RGB565_RED);
////            ips200_displayimage03x(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
//            ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);
//            //********************边线显示*****************************
////             //根据最终循环次数画出边界点
////             for (i = 0; i < data_stastics_l; i++)
////             {
////                 ips200_draw_point(0, 0, RGB565_RED);
////                 ips200_draw_point(points_l[i][0]+2, points_l[i][1],  RGB565_GREEN);
////             }
////             for (i = 0; i < data_stastics_r; i++)
////             {
////                 ips200_draw_point(points_r[i][0]-2, points_r[i][1],  RGB565_GREEN);
////             }
//             for (i = Endline+10; i < image_h-1; i++)
//             {
//               //  middle[i] = (left[i] + right[i]) >> 1;//求中线
//                 //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
//                 //当然也有多组边线的找法，但是个人感觉很繁琐，不建议
//                 ips200_draw_point(middle[i], i,  RGB565_GREEN);
//                 ips200_draw_point(left[i]+2, i, RGB565_RED);
////                 ips200_draw_point(left[i]+3, i, RGB565_RED);
//                 ips200_draw_point(right[i]-2, i, RGB565_BLUE);
////                 ips200_draw_point(right[i]-3, i, RGB565_BLUE);
//             }
////             //********************误差*****************************
//             ips200_show_string (0, 180, "Error1");
//             ips200_show_int(60, 180, Error1, 4);
////             //********************避障*****************************
////             ips200_show_string (90,180, "L_bi");
////             ips200_show_uint(130,180,road_type.L_bihznag,3);
//            //********************圆环*****************************
//            ips200_show_string (0,230, "annulus_L");
//            ips200_show_int (80,230, annulus_L_memory,3);
//            ips200_show_uint(100,230,road_type.LeftCirque,3);
//
//            ips200_show_string (0,250, "annulus_R");
//            ips200_show_int (80,250, annulus_R_memory,3);
//            ips200_show_uint(100,250,road_type.RightCirque,3);
////
//            //********************十字*****************************
//            ips200_show_string (0,270, "Cross");
//            ips200_show_int (80,270, Crossroad_memory,3);
//            ips200_show_uint(100,270,road_type.Cross,3);
////            //********************斑马线*****************************
//            ips200_show_string (0,290, "zebra");
//            ips200_show_uint(80,290,zebra_crossing_memory,3);
//            ips200_show_uint(100,290,road_type.zebra_flag,3);
////             // ********************坡道*****************************
////            ips200_show_string (120,290, "Ramp");
////            ips200_show_int (140,290,Ramp_memory,3);
////            ips200_show_uint(160,290,road_type.Ramp1,3);
//
////            ips200_show_string (0,290, "imu");
////            ips200_show_float(80,290,eulerAngle.pitch,3,3);
////            ips200_show_string (0,270, "imx");
////            ips200_show_int(100,270,imu660ra_acc_x,5);
//            int time2=system_getval_ms();
//            ips200_show_int(0, 130, time2-time1, 3);
//
//    // gpio_set_level(P14_0, 1);
//    //image_process_time=system_getval ();
//            mt9v03x_finish_flag=0;
//           }
//}

void image_process(void)//512   63
{
        int i=0;
        if(mt9v03x_finish_flag){
            // 每帧图像数据清除
            //system_start ();
           // gpio_set_level(P14_0, 0);
            camra_date_clean();
            int time1=system_getval_ms();
//            Get_image(mt9v03x_image);//图像转存
//            binaryzation();
//            Get01change_Dajin( original_image[0],imag[0],150);//图像二值化
            Sobel_Adpt_origin(mt9v03x_image,imag,Thres);   //soble边缘二值化
//            Get01change_Dajin(mt9v03x_image[0],imag[0],150);//图像二值化
//            Pixle_Filter(imag);// 图像滤波
            image_filter(imag);//滤波
            image_draw_rectan(imag);//预处理
////            find_boundary( imag );                  // 找边线
//            Image_Neighbor_Process();                   // 图像处理

//            清零
            data_stastics_l = 0;
            data_stastics_r = 0;
            if (get_start_point(image_h - 2))//找到起点了，再执行八领域，没找到就一直找
            {
                //printf("正在开始八领域\n");
               search_l_r((uint16)USE_num, imag, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &Endline);
                //printf("八邻域已结束\n");
                get_left(data_stastics_l);
                get_right(data_stastics_r);
                lost_left();
                lost_right();
                cakkuandu();//计算的赛道宽度
//                get_down_point();
//                get_up_point();
//                 f_Lower_left();//左下拐点
//                 f_Lower_right();
//                 frup();
//                 flup();
//                Element_recognition(); //元素识别
//                Element_Handle(); //元素处理
//                middle_line();
                Finish_Flag=1;
            }
            Calculate_Error_1();
            int time2=system_getval_ms();
            ips200_show_int(165, 125, time2-time1, 3);
            ips200_show_gray_image(0, 130, imag[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
            drawing(); //一个字母占九格

//            for(i=120;i>0;i--)
//            {
//                printf("left[%d]:%d\n",i,left[i]);
//            }
//            for(i=120;i>0;i--)
//            {
//                printf("right[%d]:%d\n",i,right[i]);
//            }
//            for(int i=120;i>0;i--)
//            {
//                printf("middle[%d]:%d\n",i,middle[i]);
//            }

            ips200_show_gray_image(0, 150, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
//            ********************边线显示*****************************
//             根据最终循环次数画出边界点
             for (i = 0; i < data_stastics_l; i++)
             {
                 ips200_draw_point(0, 0, RGB565_RED);
                 ips200_draw_point(points_l[i][0]+2, points_l[i][1],  RGB565_GREEN);
             }
             for (i = 0; i < data_stastics_r; i++)
             {
                 ips200_draw_point(points_r[i][0]-2, points_r[i][1],  RGB565_GREEN);
             }
             for (i = Endline+10; i < image_h-1; i++)
             {
               //  middle[i] = (left[i] + right[i]) >> 1;//求中线
//                 求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
                 //当然也有多组边线的找法，但是个人感觉很繁琐，不建议
                 ips200_draw_point(middle[i], i,  RGB565_GREEN);
                 ips200_draw_point(left_dispose[i]+2, i, RGB565_RED);
//                 ips200_draw_point(left[i]+3, i, RGB565_RED);
                 ips200_draw_point(right_dispose[i]-2, i, RGB565_BLUE);
//                 ips200_draw_point(right[i]-3, i, RGB565_BLUE);
             }

    // gpio_set_level(P14_0, 1);
    //image_process_time=system_getval ();
            mt9v03x_finish_flag=0;
           }
}
void camra_date_clean(void)// 每帧图像数据清除
{
        data_stastics_l=0;
        data_stastics_r=0;

        for( uint8 t=0 ; t<120 ; t++ )
        {
            points_l[t][1]=0;
            points_r[t][1]=0;
            right_lost_num=0;
            left_lost_num=0;
            Lost_left_Flag=0;
            Lost_right_Flag=0;
            Lost_point_L_scan_line=0;
            Lost_point_R_scan_line=0;
            left[t]=0;
            right[t]=0;
            middle[t]=0;
        }
}

//float offset_quanzhong [15] = {0.94, 0.92, 0.88, 0.83, 0.77,
//                               0.71, 0.65, 0.59, 0.53, 0.47,
//                               0.47, 0.47, 0.47, 0.47, 0.47,};  //偏差权重


void Beep_Action(uint8 State) //   State 1 响    State  0 不响
{
    gpio_set_level(P33_10,State);
}

float offset_quanzhong [15] = {0.94, 0.92, 0.88, 0.83, 0.77,
                               0.71, 0.65, 0.59, 0.5, 0.50,
                               0.50, 0.50, 0.50, 0.50, 0.50,};  //偏差权重




//三点法计算赛道曲率
float Process_Curvity(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3)
{
    float K;
    int S_of_ABC = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;
    //面积的符号表示方向
    int16 q1 = (int16)((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    uint8 AB = My_Sqrt(q1);
    q1 = (int16)((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    uint8 BC = My_Sqrt(q1);
    q1 = (int16)((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    uint8 AC = My_Sqrt(q1);
    if (AB * BC * AC == 0)
    {
        K = 0;
    }
    else
        K = (float)4 * S_of_ABC / (AB * BC * AC);
    return K;
}

//自己写的开方函数
uint8 My_Sqrt(int16 x)
{
    uint8 ans=0, p=0x80;
    while(p!=0)
    {
        ans += p;
        if(ans*ans>x)
        {
            ans -= p;
        }
        p = (uint8)(p/2);
    }
    return (ans);
}
int16 Sum1;
//平均滑动滤波
void HDPJ_lvbo(uint8 data[], uint8 N, uint8 size)
{
    Sum1 = 0;
    for(uint8 j =0; j <size; j++)
    {
        if(j <N /2)
        {
            for(uint8 k =0; k <N; k++)
            {
                Sum1 +=data[j +k];
            }
            data[j] =Sum1 /N;
        }
        else
            if(j <size -N /2)
            {
                for(uint8 k =0; k <N /2; k++)
                {
                    Sum1 +=(data[j +k] +data[j -k]);
                }
                data[j] = Sum1 /N;
            }
            else
            {
                for(uint8 k =0; k <size -j; k++)
                {
                    Sum1 +=data[j +k];
                }
                for(uint8 k =0; k <(N -size +j); k++)
                {
                    Sum1 +=data[j -k];
                }
                data[j] = Sum1 /N;
            }
        Sum1 = 0;
    }
}
//#define limit(x, y)     ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))    // 限幅 数据范围是 [-32768,32767]
//
//int16 Error1;                    //摄像头处理得到的偏差
//uint8 Error1_endline=10;
//void Calculate_Error_1(void)
//{
//     Error1 = 1;
//     for(uint8 y =MT9V03X_H -1; y >=Error1_endline; y--)    //利用近大远小的权重计算偏差(调试完成)
//     {
//         middle[y] =1.0*(left[y] +right[y])/2;    //这句代码并不是没有用,千万不可以删
//     }
//     HDPJ_lvbo(middle, 30, MT9V03X_H -1);   //平均滑动滤波
//     for(uint8 y =MT9V03X_H -25; y >=MT9V03X_H -40; y--)    //利用近大远小的权重计算偏差(调试完成)
//     {
//         Error1 +=offset_quanzhong[MT9V03X_H-30 -y] *(middle[y] -MT9V03X_W/2);
//     }
//
////     Error1 =Error1+158;
//     Error1 =Error1/2/10+4;
//     Error1 = limit( Error1 , 30 );    // 限幅在+-30以内
//
////      //常规误差
////       for(uint8 i=41;i<=90;i++)
////       {
////           Error1+=(MT9V03X_W/2-((l_border[i]+r_border[i])>>1));//右移1位，等效除2
////       }
////       Error1=Error1/50.0;
//      //(road_type.Barn_r_in == 1) && (pass_barn == 2))
////      {
////          offset = 200;
////      }
////      if((road_type.Barn_l_in == 1) && (pass_barn == 2))
////      {
////          offset = -200;
////      }
//}

#define limit(x, y)     ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))  // 限幅 数据范围是 [-32768,32767]



float Error1;                    //摄像头处理得到的偏差
uint8 Error1_endline=10;
float LastError1=0;
float EC1=0;
float E1=0;
uint8 start_line=0,end_line=0;
void Calculate_Error_1(void)
{
     Error1 = 1;

////     HDPJ_lvbo(middle, 30, MT9V03X_H -1);   //平均滑动滤波
//     for(uint8 y =MT9V03X_H -35; y >=MT9V03X_H -45; y--)    //利用近大远小的权重计算偏差(调试完成)
//     {
//         Error1 +=offset_quanzhong[MT9V03X_H-35 -y] *(middle[y] -MT9V03X_W/2);
// }
      for(uint8 y =MT9V03X_H -1; y >=Error1_endline; y--)    //利用近大远小的权重计算偏差(调试完成)
      {
          middle[y] =1.0*(left_dispose[y] +right_dispose[y])/2;    //这句代码并不是没有用,千万不可以删
//          middle[y] =1.0*(left[y] +right[y])/2;
      }
      uint8 y=0;

       for(uint8 i=40;i<=70;i++)
       {
//
//           start_line=70,end_line=40;
//           for(uint8 i=start_line;i >=end_line;i--)  //常规误差
//           {
//               Error1+=(MT9V03X_W/2-1-middle[i]);
//           }
//           float   Error2=(Error1/30.0-2);
//          E1=Error1=limit(Error2,68);
//
//         EC1 =Error1-LastError1;
//         LastError1=Error1;
//           if((MT9V03X_W/2-1-middle[i])==0&&(Lost_left_Flag==1||Lost_right_Flag==1)&&Upper_left_inflection_Flag==0&&Upper_right_inflection_Flag==0)
           if((MT9V03X_W/2-1-middle[i])==0&&(Lost_left_Flag==1||Lost_right_Flag==1)&&road_type.Cross==0)
           {
               y++;
               if(y>=30)
               {
                   y=30;
               }
           }
           start_line=70+y,end_line=40+y;
           for(uint8 i=start_line;i >=end_line;i--)  //常规误差
           {
               Error1+=(1+y/10)*(MT9V03X_W/2-1-middle[i]);
           }
           float   Error2=(Error1/30.0+1.0);
          E1=Error1=limit(Error2,60);

         EC1 =Error1-LastError1;
         LastError1=Error1;
       }
}

//float Error1;                    //摄像头处理得到的偏差
//uint8 Error1_endline=10;
//float LastError1=0;
//float EC1=0;
//float E1=0;
//uint8 start_line=0,end_line=0;
//void Calculate_Error_1(void)
//{
//     Error1 = 1;
//
//////     HDPJ_lvbo(middle, 30, MT9V03X_H -1);   //平均滑动滤波
////     for(uint8 y =MT9V03X_H -35; y >=MT9V03X_H -45; y--)    //利用近大远小的权重计算偏差(调试完成)
////     {
////         Error1 +=offset_quanzhong[MT9V03X_H-35 -y] *(middle[y] -MT9V03X_W/2);
//// }
//      for(uint8 y =MT9V03X_H -1; y >=Error1_endline; y--)    //利用近大远小的权重计算偏差(调试完成)
//      {
//          middle[y] =1.0*(left_dispose[y] +right_dispose[y])/2+5;    //这句代码并不是没有用,千万不可以删
////          middle[y] =1.0*(left[y] +right[y])/2;
//      }
//      uint8 y=0;
//
//       for(uint8 i=40;i<=70;i++)
//       {
//           if((MT9V03X_W/2-middle[i])==0&&(Lost_left_Flag==1||Lost_right_Flag==1))
//           {
//               y++;
//               if(y>=30)
//               {
//                   y=30;
//               }
//           }
//
//           start_line=70+y,end_line=40+y;
//           for(uint8 i=start_line;i >=end_line;i--)  //常规误差
//           {
//               Error1+=(1+y/10)*(MT9V03X_W/2-1-5-middle[i]);
//           }
//
//          E1=Error1=(Error1/30.0+6+5-3+2);
//
//         EC1 =Error1-LastError1;
//         LastError1=Error1;
//       }
//
////      E1=Error1=(Error1/50.0-2);
//
////     EC1=limit( EC1 ,100 );
//
//      //(road_type.Barn_r_in == 1) && (pass_barn == 2))
////      {
////          offset = 200;
////      }
////      if((road_type.Barn_l_in == 1) && (pass_barn == 2))
////      {
////          offset = -200;
////      }
//}


//int16 Error1;                    //摄像头处理得到的偏差
//uint8 Error1_endline=10;
//int16 LastError1=0;
//int16 EC1=0;
//int16 E1=0;
//void Calculate_Error_1(void)
//{
//     Error1 = 1;
////     for(uint8 y =MT9V03X_H -1; y >=Error1_endline; y--)    //利用近大远小的权重计算偏差(调试完成)
////     {
////         middle[y] =1.0*(left[y] +right[y])/2;    //这句代码并不是没有用,千万不可以删
////     }
//////     HDPJ_lvbo(middle, 30, MT9V03X_H -1);   //平均滑动滤波
////     for(uint8 y =MT9V03X_H -35; y >=MT9V03X_H -45; y--)    //利用近大远小的权重计算偏差(调试完成)
////     {
////         Error1 +=offset_quanzhong[MT9V03X_H-35 -y] *(middle[y] -MT9V03X_W/2);
////     }
//       //常规误差
//      for(uint8 y =MT9V03X_H -1; y >=Error1_endline; y--)    //利用近大远小的权重计算偏差(调试完成)
//      {
//          middle[y] =1.0*(left[y] +right[y])/2;    //这句代码并不是没有用,千万不可以删
//      }
//
//       for(uint8 i=100;i >=50;i--)
//       {
//           Error1+=(MT9V03X_W/2-2-middle[i]);
//       }
////      E1=Error1=(Error1/50.0-2);
//      E1=Error1=(Error1/50.0-1);
//
//     EC1 =Error1-LastError1;
////     EC1=limit( EC1 ,100 );
//     LastError1=Error1;
//      //(road_type.Barn_r_in == 1) && (pass_barn == 2))
////      {
////          offset = 200;
////      }
////      if((road_type.Barn_l_in == 1) && (pass_barn == 2))
////      {
////          offset = -200;
////      }
//}
uint8 sudu_yingzi=0;
//速度因子
void Check_Zhidao(void)//计算直道长度
{//这里要考虑搜线终止行
    uint8 lc =0, rc =0;
    sudu_yingzi =0;

    for(uint8 y =MT9V03X_H-1; y >1; y--)
    {
        if((left[y] <=left[y-1]) && left[y] !=0)   //两边不丢线才计算直道长度
        {
            lc++;
        }

        if((right[y] >=right[y-1]) && right[y] !=MT9V03X_W -1)
        {
            rc++;
        }
    }

    if(lc>=rc)
    {
        sudu_yingzi =lc;
    }
    else
    {
        sudu_yingzi =rc;
    }//sudu_yingzi>75为直道
}

uint8 length=0;
void Zhidao(void)
{
    int i=0;
    for(i=MT9V03X_H;i>Endline;i--)
    {
        if(imag[i][94]==255 && imag[i-1][94]==255 && imag[i-2][94]==255)
        {
            length++;
            break;
        }
    }
}

uint8 Mid_Col(uint8 endline )
{
    int i,length1=0,length2=0,cha;
        for(i=MT9V03X_H;i>endline;i--)
        {
            if(imag[i][80]==0 && imag[i-1][80]==0 && imag[i-2][80]==0)
            {
                break;
            }
            length1=MT9V03X_H-i;
        }
       for(i=MT9V03X_H;i>endline;i--)
       {
           if(imag[i][110]==0 && imag[i-1][110]==0 && imag[i-2][110]==0)
           {
               break;
           }
           length2=MT9V03X_H-i;
       }

       cha=IMy_Abs(length1, length2);

       if(cha<10)
       {
           return 2;//直道
       }
       else
       {
           if(length1<length2)
           {
               return 0;//左弯道
           }
           if(length1>length2)
           {
               return 1;//右弯道
           }
       }
}

void Ramp_fuzhu(void)//坡道辅助识别
{
    //10-60左线斜率  直线为0.280
    regression(1,10,60);
    LK0end = parameterB;
    //10-100左线斜率
    advanced_regression(1, 10, 20,90, 100);
    LK = parameterB;
    //10-60右线斜率
    regression(2, 10, 60);
    RK0end = parameterB;
    //10-100右线斜率
    advanced_regression(2, 10, 20,90, 100);
    RK = parameterB;
    //斜率偏差
//    kerrorL =LK - LK0end;
//    kerrorR =RK - RK0end;
//    ips200_show_float( 0,200,LK,3,3);
//    ips200_show_float( 80,200,RK,3,3);
//    ips200_show_float(0,220, LK0end,3,3);
//    ips200_show_float(80,220, RK0end,3,3);
//    ips200_show_float(0,240, LK-LK0end,3,3);
//    ips200_show_float(80,240, RK-RK0end,3,3);

//  直道                        坡道(60行位于坡道底部时斜率)   坡道(80行位于坡道底部时斜率)
//    LK          RK            LK          RK             LK          RK
//    LK0end      RK0end        LK0end      RK0end         LK0end      RK0end
//    -0.380      0.451         -0.293      0.339          -0.222      0.306
//    -0.399      0.421         -0.211      0.219          -0.217      0.230
//偏差 0.019       0.03          -0.082      0.120          -0.005      0.071

//赛道宽度           40行                    60行                   80行
//  直道             57                     73                     90
//  坡道            >70（初遇坡道）           >80（准备上坡）         >100(坡道中)

}


void Ramp_identification(void)//坡道识别
{
//    Ramp_memory=0;     //坡道计步
//    road_type.Ramp=0;  //坡道标志位
//&&bianxian_tubian(0)==0&&bianxian_tubian(1)==0
//    int up_Ramp_flag=0;
    Ramp_fuzhu();//坡道辅助识别
       if(road_type.Ramp1==0&&calkuan[60]>79)
    {
        if(road_type.Ramp1==0&&calkuan[60]>79&&LK>=-0.32&&LK0end>=-0.30&&RK<0.35&&RK0end<0.30&&kerrorL>-0.03&&kerrorR<0.100)
        {
            road_type.Ramp1=1;
            Ramp_memory=1;
        }
    }
}
//           if()
//           {
//               Ramp_memory=3;
//           }
//       }

//    if(up_Ramp_flag==1)
//    {
//
//    }
//    if(road_type.Ramp==1)
//    {
//            if(Ramp_memory==1)
//            {
//                if()
//                    Ramp_memory=2;
//            }
//    }
//    else if()
//    {
//
//    }
//}
float k_spped=0;

void Ramp_Handle(void)
{
    Ramp_fuzhu();//坡道辅助识别
//    float LK0end=0,LK=0,RK0end=0,RK=0,kerrorL=0,kerrorR=0;
    if(road_type.Ramp1==1)
    {
        if(Ramp_memory==1)
      {
            k_spped=1.5;
          if(calkuan[60]>79)
              Ramp_memory=2;//上坡
      }
      else if(Ramp_memory==2)
      {
          k_spped=1.8;
          if(calkuan[60]<40&&calkuan[80]<80&&kerrorL<-1&&kerrorR>1)
          {
              Ramp_memory=2;//
          }
          else if(calkuan[40]<40&&calkuan[60]<50&&calkuan[80]<80)
          {
              Ramp_memory=3;//坡顶
          }
          else return;
      }
      else if(Ramp_memory==3)
        {
            k_spped=1.3;
            if(calkuan[60]>80&&calkuan[80]>90&&kerrorL>-0.02&&kerrorR<0.06)
            {
                Ramp_memory=4;//减速
            }
            else return;
        }
        else if(Ramp_memory==4)
        {

            if(calkuan[60]>90&&calkuan[80]>100)
            {
                k_spped=-0.5;
                return;
            }
            if(calkuan[60]<80&&calkuan[80]<95)
            {
                Ramp_memory=0;
                road_type.Ramp1=0;
                return;
            }
        }


//  直道                        坡道(60行位于坡道底部时斜率)   坡道(80行位于坡道底部时斜率)
//    LK          RK            LK          RK             LK          RK
//    LK0end      RK0end        LK0end      RK0end         LK0end      RK0end
//    -0.380      0.451         -0.293      0.339          -0.222      0.306
//    -0.399      0.421         -0.211      0.219          -0.217      0.230
//偏差 0.019       0.03          -0.082      0.120          -0.005      0.071
//赛道宽度           40行                    60行                   80行
//  直道             57                     73                     90
//  坡道            >70（初遇坡道）           >80（准备上坡）         >100(坡道中)

}}
int16 Ramp_m;
void Ramp_Handle_m(void)//坡道处理
{
    Ramp_fuzhu();//坡道辅助识别
    if(road_type.Ramp1==1)
    {
        road_type.zebra_flag =0;
        road_type.LeftCirque =0;
        road_type.RightCirque =0;
        road_type.Cross = 0;
        road_type.Ramp1=1;
        if(Ramp_memory==1)
      {
            k_spped=1.5;
          if(calkuan[60]>79)
              Ramp_memory=2;//上坡
      }
      else if(Ramp_memory==2)
      {
//          k_spped=1.8;
          if(calkuan[60]<40&&calkuan[80]<80&&kerrorL<-1&&kerrorR>1)
          {
              Ramp_memory=2;//
          }
      }
        if(Ramp_memory==2)
      {
          encoder.encoder_enable=1;
          Ramp_m=encoder.encodersum_average;
          if(Ramp_m<7000)
          {
              Ramp_memory=2;
              road_type.Ramp1=1;
          }
          else
          {
              Ramp_m=0;
              Ramp_memory=0 ;
              road_type.Ramp1=0;
              k_spped=0;
          }
      }

    }
}

float FMy_Abs(float a, float b)//求两数之差绝对值的浮点数
 {

     if ((a - b) > 0)
         return ((float)(a - b));
     else return ((float)(b - a));
 }

int IMy_Abs(int a, int b)//求两数之差绝对值
 {

     if ((a - b) > 0)
         return ((int)(a - b));
     else return ((int)(b - a));
 }
//求两数之差绝对值结束

uint8 Num=0;
//判断圆环还是十字（利用圆环的第一段圆弧）
uint8 Cirque_or_Cross(uint8 type, uint8 startline)
{//1为左圆环，2为右圆环
    uint8 num =0;
    if(type ==1)//左侧丢线位置y的下方有较多白点,判断为左圆环
    {
        for(uint8 y=startline; y<startline+10; y++)
        {
            for(uint8 x=left[y]; x>1; x--)
            {
                if(imag[y][x] !=0)
                {
                    num ++;
                }
            }
        }
    }
    if(type ==2)//右侧丢线位置y的下方有较多白点,判断为右圆环
    {
        for(uint8 y=startline; y<startline+10; y++)
        {
            for(uint8 x=right[y]; x<MT9V03X_W -2; x++)
            {
                if(imag[y][x] !=0)
                {
                    num ++;
                }
            }
        }
    }
    Num=num;
    return num;
}
/*********************END*****************************************/


uint8 bianxian_tubian(uint8 type)//判断边线是否突变  0左1右
{
    if (type ==0)
    {
        for(uint8 y1 =MT9V03X_H -1; y1>=30; y1=y1-4)
        {

            if(fabs(left[y1] -left[y1-1])<4 && (left[y1] -left[y1-10]) >8)
            {
                return 1;
            }
        }
    }
    else if (type ==1)
    {
        for(uint8 y1 =MT9V03X_H -1; y1>=30; y1=y1-4)
        {
            if(fabs(right[y1] -right[y1-1])<4 && (right[y1] -right[y1-10]) <-8)
            {
                return 1;
            }
        }
    }
    return 0;
}
uint8 L_tubian_flag=0;
uint8 R_tubian_flag=0;
void L_R_tubian(void)//判断边线是否突变  0 不突变   1突变
{
    L_tubian_flag=0;
    R_tubian_flag=0;
        for(uint8 y1 =MT9V03X_H -1; y1>=30; y1=y1-4)
        {

            if(fabs(left[y1] -left[y1-1])<4 && (left[y1] -left[y1-4]) >8&&left[y1-5]==2&&left[y1-1]>20)
            {
                L_tubian_flag=1;
            }
        }

        for(uint8 y1 =MT9V03X_H -1; y1>=30; y1=y1-4)
        {
            if(fabs(right[y1] -right[y1-1])<4 && (right[y1] -right[y1-4]) <-8&&right[y1-5]==185&&right[y1+1]<160)
            {
               R_tubian_flag=1;
            }
        }
}


uint8 l_up_bizhang_x=0;
uint8 l_up_bizhang_y=0;
uint8 l_up_bizhang_flag=0;

uint8 r_up_bizhang_x=0;
uint8 r_up_bizhang_y=0;
uint8 r_up_bizhang_flag=0;
void l_bizhang(void)
{
        for(uint8 y =MT9V03X_H -1; y>=60; y=y--)
        {
//                calkuan[60]=73&&L_tubian_flag==1&&R_tubian_flag==0&&Right_straight_flag==1
            if(left[y-1] -left[y]<4 && (left[y] -left[y+10]) >8&&Lost_left_Flag==0&&Lost_right_Flag==0&&calkuan[y+4]-calkuan[y]>8&&R_tubian_flag==0&&Right_straight_flag==1&&left[y+10]!=2)
            {
                road_type.L_bihznag=1;
                l_up_bizhang_x=left[y]+1;
                l_up_bizhang_y=y-1;
                l_up_bizhang_flag=1;
            }
            if(right[y] -right[y-1]<4 && (right[y+10] -right[y]) >8&&Lost_left_Flag==0&&Lost_right_Flag==0&&calkuan[y+4]-calkuan[y]>8&&L_tubian_flag==0&&Left_straight_flag==1&&right[y+10]!=185)
            {
                road_type.L_bihznag=1;
                r_up_bizhang_x=right[y]+1;
                r_up_bizhang_y=y-1;
                r_up_bizhang_flag=1;
            }
        }
        if(road_type.L_bihznag==1)
        {
            if(l_up_bizhang_flag==1)
            {
                  if(l_up_bizhang_y<100)
                {
                    Addingline(1, l_up_bizhang_x+60,40,l_up_bizhang_x+20, 70);
                    return;
                }
                else
                {
                    Addingline(1, l_up_bizhang_x+40,40,l_up_bizhang_x+10, 70);
                }
            }
            else if(r_up_bizhang_flag==1)
            {
                if(r_up_bizhang_y<100)
               {
                   Addingline(2, r_up_bizhang_x-20,40,r_up_bizhang_x-1, 70);
                   return;
               }
               else
               {
                   Addingline(2, r_up_bizhang_x-10,40,r_up_bizhang_x-1, 70);
               }
            }

            if(calkuan[100]>=100)
            {
                road_type.L_bihznag=0;
                l_up_bizhang_flag=0;
                l_up_bizhang_x=0;
                l_up_bizhang_y=0;

                r_up_bizhang_x=0;
                r_up_bizhang_y=0;
                r_up_bizhang_flag=0;;
                return;
            }
        }
//        if(up_bizhang_memory==2)
//        {
//            if(calkuan[100]<=80)
//            {
//                Addingline(1, 84,50,left[118], 118);
//                return;
//            }
//
//        }
}
