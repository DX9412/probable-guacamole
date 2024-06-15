/*
 * image.h
 *
 *  Created on: 2023年8月25日
 *      Author: Charlie
 */

#ifndef ALGORITHM_IMAGE_H_
#define ALGORITHM_IMAGE_H_


#include "zf_common_headfile.h"


//typedef   signed          char int8;
//typedef   signed short     int int16;
//typedef   signed           int int32;
//typedef unsigned          char uint8;
//typedef unsigned short     int uint16;
//typedef unsigned           int uint32;




//宏定义
#define image_h 120//图像高度
#define image_w 188//图像宽度

#define white_pixel 255
#define black_pixel 0

#define bin_jump_num    1//跳过的点数
#define border_max  image_w-2 //边界最大值
#define border_min  1   //边界最小值
extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//图像数组
extern uint8 Finish_Flag; //处理完成标识位
extern uint32 image_process_time;   //图像处理时间

extern uint8 Thres;//增大白色变多 140
extern uint8 huandao_stage;
extern uint8 xieshizi_stage;

struct ROAD_TYPE
{
     int8 straight;         //直道
     int8 bend;             //弯道
     int8 L_bend;           //左弯道
     int8 R_bend;           //右弯道
     int8 Ramp1;             //坡道
     int8 Cross;            //十字
     int8 L_Cross;          //入左十字
     int8 R_Cross;          //入右十字
     int8 LeftCirque;       //左环岛
     int8 RightCirque;      //右环岛
     int8 L_bihznag;        //左避障
     int8 R_bihznag;        //右避障
     int8 zebra_flag;       //斑马线
     int8 Barn_l_out;       //出左库
     int8 Barn_r_out;       //出右库
     int8 Barn_l_in;        //入左库
     int8 Barn_r_in;        //入右库
     int8 smalls;           // 小s
};


extern struct ROAD_TYPE road_type;




extern uint8 imag[120][188];
extern uint8  middle[120];
extern uint8 threshold_value;


extern void Beep_Action(uint8 State);//   State 1 响    State  0 不响

void All_Init(void);//全部初始化
extern void image_process(void);//图像全操作

void Element_recognition(void);//元素识别
void Element_Handle(void);//元素处理

extern uint8 kuandu_saidao[MT9V03X_H-1];   //储存赛道宽度的数组
void camra_date_clean(void);;// 每帧图像数据清除
uint8 OtsuThreshold(uint8 *image, uint16 col, uint16 row);//大津法图像阈值
void binaryzation(void);//大津法二值化
unsigned char Image_Threshold(unsigned char *image, unsigned short width, unsigned short height, unsigned int image_max_threshold);//大津法计算图像二值化阈值
void Get01change_Dajin(unsigned char *in_image,unsigned char *out_image, unsigned int image_max_threshold);//   大津法图像二值化
void Sobel_Adpt_origin(uint8 (*in_IMG)[image_w], uint8 (*out_IMG)[image_w], uint16 Threshold);//Sobel图像二值化
void Pixle_Filter(uint8(*image)[image_w]);// 图像滤波
void image_filter(uint8(*imag)[image_w]);//形态学滤波，膨胀和腐蚀的思想


void IPS_show(void);//屏幕显示
void drawing(void);//画边线
float absolute(float z);

void inflection_point(void);//拐点总判断
void wandaopanduan(void);//弯道和直道判别

void cakkuandu(void);//赛道宽度
void right_straight(void);//右直线识别
extern uint8 Right_straight_flag; //右直线

void left_straight(void);//左直线识别
extern uint8 Left_straight_flag; //左直线

void Addingline1( uint8 choice, uint8 startX, uint8 startY);//看到拐点延斜率向上延长
void Addingline( uint8 choice, uint8 startX, uint8 startY, uint8 endX, uint8 endY);//有起点和终点的补线函数
void Addingline2( uint8 choice, uint8 startX, uint8 startY);//找到上拐点延斜率向下拉线



void regression(int type, int startline, int endline);//最小二乘法拟合曲线，分别拟合中线，左线，右线,type表示拟合哪几条线
void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2);//计算斜率，截距
int Judgment_symbol(float x, float y);//判断直线斜率是否相同

//上下拐点
void Lower_left(void);
void Lower_right(void);
void Upper_left(void);
void Upper_right(void);

void f_Lower_left(void);//左下拐点


void annulus_L(void);
extern uint8 annulus_L_Flag;       //左圆环
extern uint8 annulus_L_memory;     //左圆环计步

void annulus_R(void);
extern uint8 annulus_R_Flag;       //右圆环

void Ramp_fuzhu(void);//坡道辅助识别
void Ramp_identification(void);//坡道判断
void Ramp_Handle(void);//坡道处理
void Ramp_Handle_m(void);//坡道处理
extern int16 Ramp_m;
extern float k_spped;
extern uint8 Ramp_memory;     //坡道计步
extern float LK0end,LK,RK0end,RK,kerrorL,kerrorR;


void cross_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2]);//十字
extern uint8 Crossroad_Flag;      //十字
extern uint8 Crossroad_memory;     //十字计步
extern uint8 Finish_Flag;    //处理完成标识位

extern uint8 L_Crossroad_Flag;//左斜入十字
extern uint8 L_Crossroad_memory;

void crossroad222(void);
void crossroad(void);
void crossroad111(void);


void banmaxian(int start_point, int end_point);//斑马线识别
void zebra_crossing(uint8 zebra_start);//斑马线识别
extern int16 zebra_m;
extern uint8 zebra_crossing_flag;//斑马线
extern uint8 Left_garage_flag; //左车库
extern uint8 Left_garage_memory;//左车库计步
extern uint8 zebra_crossing_memory;
extern uint8 car_stop;

void car_start(void);
extern uint8 stop;
extern uint16 time_count;

extern uint8 car_state;//0 停车  1开始计数    计时到状态2 发车标志位
extern uint8 start_count;
extern uint8 Enable_Element;

float Process_Curvity(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3);
uint8 My_Sqrt(int16 x);
void HDPJ_lvbo(uint8 data[], uint8 N, uint8 size);//平均滑动滤波

void Calculate_Error_1(void);  //摄像头处理得到的偏差
extern float Error1;
extern uint8 Error1_endline;
extern float EC1;
extern float E1;
extern uint8 start_line,end_line;

extern uint8 sudu_yingzi;    //速度因子
void Check_Zhidao(void);

uint8 Mid_Col(uint8 endline );//计算直道长度
extern uint8 length;

void track_line( uint8 type , uint16 start , uint16 end );//


float FMy_Abs(float a, float b);//求两数之差绝对值的浮点数
int IMy_Abs(int a, int b);//求两数之差绝对值

void mysmalls(void);

void Zhidao(void);//直道高度
uint8 Cirque_or_Cross(uint8 type, uint8 startline);//判断圆环还是十字（利用圆环的第一段圆弧）
uint8 bianxian_tubian(uint8 type);//判断边线是否突变  0左1右
void L_R_tubian(void);//判断边线是否突变  0左1右
extern uint8 L_tubian_flag;
extern uint8 R_tubian_flag;

extern uint8 l_up_bizhang_flag;
extern uint8 l_up_bizhang_x;
extern uint8 l_up_bizhang_y;

extern uint8 r_up_bizhang_flag;
extern uint8 r_up_bizhang_x;
extern uint8 r_up_bizhang_y;
void l_bizhang(void);

#endif /* ALGORITHM_IMAGE_H_ */
