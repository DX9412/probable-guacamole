/*
 * image.h
 *
 *  Created on: 2023��8��25��
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




//�궨��
#define image_h 120//ͼ��߶�
#define image_w 188//ͼ����

#define white_pixel 255
#define black_pixel 0

#define bin_jump_num    1//�����ĵ���
#define border_max  image_w-2 //�߽����ֵ
#define border_min  1   //�߽���Сֵ
extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//ͼ������
extern uint8 Finish_Flag; //������ɱ�ʶλ
extern uint32 image_process_time;   //ͼ����ʱ��

extern uint8 Thres;//�����ɫ��� 140
extern uint8 huandao_stage;
extern uint8 xieshizi_stage;

struct ROAD_TYPE
{
     int8 straight;         //ֱ��
     int8 bend;             //���
     int8 L_bend;           //�����
     int8 R_bend;           //�����
     int8 Ramp1;             //�µ�
     int8 Cross;            //ʮ��
     int8 L_Cross;          //����ʮ��
     int8 R_Cross;          //����ʮ��
     int8 LeftCirque;       //�󻷵�
     int8 RightCirque;      //�һ���
     int8 L_bihznag;        //�����
     int8 R_bihznag;        //�ұ���
     int8 zebra_flag;       //������
     int8 Barn_l_out;       //�����
     int8 Barn_r_out;       //���ҿ�
     int8 Barn_l_in;        //�����
     int8 Barn_r_in;        //���ҿ�
     int8 smalls;           // Сs
};


extern struct ROAD_TYPE road_type;




extern uint8 imag[120][188];
extern uint8  middle[120];
extern uint8 threshold_value;


extern void Beep_Action(uint8 State);//   State 1 ��    State  0 ����

void All_Init(void);//ȫ����ʼ��
extern void image_process(void);//ͼ��ȫ����

void Element_recognition(void);//Ԫ��ʶ��
void Element_Handle(void);//Ԫ�ش���

extern uint8 kuandu_saidao[MT9V03X_H-1];   //����������ȵ�����
void camra_date_clean(void);;// ÿ֡ͼ���������
uint8 OtsuThreshold(uint8 *image, uint16 col, uint16 row);//���ͼ����ֵ
void binaryzation(void);//��򷨶�ֵ��
unsigned char Image_Threshold(unsigned char *image, unsigned short width, unsigned short height, unsigned int image_max_threshold);//��򷨼���ͼ���ֵ����ֵ
void Get01change_Dajin(unsigned char *in_image,unsigned char *out_image, unsigned int image_max_threshold);//   ���ͼ���ֵ��
void Sobel_Adpt_origin(uint8 (*in_IMG)[image_w], uint8 (*out_IMG)[image_w], uint16 Threshold);//Sobelͼ���ֵ��
void Pixle_Filter(uint8(*image)[image_w]);// ͼ���˲�
void image_filter(uint8(*imag)[image_w]);//��̬ѧ�˲������ͺ͸�ʴ��˼��


void IPS_show(void);//��Ļ��ʾ
void drawing(void);//������
float absolute(float z);

void inflection_point(void);//�յ����ж�
void wandaopanduan(void);//�����ֱ���б�

void cakkuandu(void);//�������
void right_straight(void);//��ֱ��ʶ��
extern uint8 Right_straight_flag; //��ֱ��

void left_straight(void);//��ֱ��ʶ��
extern uint8 Left_straight_flag; //��ֱ��

void Addingline1( uint8 choice, uint8 startX, uint8 startY);//�����յ���б�������ӳ�
void Addingline( uint8 choice, uint8 startX, uint8 startY, uint8 endX, uint8 endY);//�������յ�Ĳ��ߺ���
void Addingline2( uint8 choice, uint8 startX, uint8 startY);//�ҵ��Ϲյ���б����������



void regression(int type, int startline, int endline);//��С���˷�������ߣ��ֱ�������ߣ����ߣ�����,type��ʾ����ļ�����
void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2);//����б�ʣ��ؾ�
int Judgment_symbol(float x, float y);//�ж�ֱ��б���Ƿ���ͬ

//���¹յ�
void Lower_left(void);
void Lower_right(void);
void Upper_left(void);
void Upper_right(void);

void f_Lower_left(void);//���¹յ�


void annulus_L(void);
extern uint8 annulus_L_Flag;       //��Բ��
extern uint8 annulus_L_memory;     //��Բ���Ʋ�

void annulus_R(void);
extern uint8 annulus_R_Flag;       //��Բ��

void Ramp_fuzhu(void);//�µ�����ʶ��
void Ramp_identification(void);//�µ��ж�
void Ramp_Handle(void);//�µ�����
void Ramp_Handle_m(void);//�µ�����
extern int16 Ramp_m;
extern float k_spped;
extern uint8 Ramp_memory;     //�µ��Ʋ�
extern float LK0end,LK,RK0end,RK,kerrorL,kerrorR;


void cross_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2]);//ʮ��
extern uint8 Crossroad_Flag;      //ʮ��
extern uint8 Crossroad_memory;     //ʮ�ּƲ�
extern uint8 Finish_Flag;    //������ɱ�ʶλ

extern uint8 L_Crossroad_Flag;//��б��ʮ��
extern uint8 L_Crossroad_memory;

void crossroad222(void);
void crossroad(void);
void crossroad111(void);


void banmaxian(int start_point, int end_point);//������ʶ��
void zebra_crossing(uint8 zebra_start);//������ʶ��
extern int16 zebra_m;
extern uint8 zebra_crossing_flag;//������
extern uint8 Left_garage_flag; //�󳵿�
extern uint8 Left_garage_memory;//�󳵿�Ʋ�
extern uint8 zebra_crossing_memory;
extern uint8 car_stop;

void car_start(void);
extern uint8 stop;
extern uint16 time_count;

extern uint8 car_state;//0 ͣ��  1��ʼ����    ��ʱ��״̬2 ������־λ
extern uint8 start_count;
extern uint8 Enable_Element;

float Process_Curvity(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3);
uint8 My_Sqrt(int16 x);
void HDPJ_lvbo(uint8 data[], uint8 N, uint8 size);//ƽ�������˲�

void Calculate_Error_1(void);  //����ͷ����õ���ƫ��
extern float Error1;
extern uint8 Error1_endline;
extern float EC1;
extern float E1;
extern uint8 start_line,end_line;

extern uint8 sudu_yingzi;    //�ٶ�����
void Check_Zhidao(void);

uint8 Mid_Col(uint8 endline );//����ֱ������
extern uint8 length;

void track_line( uint8 type , uint16 start , uint16 end );//


float FMy_Abs(float a, float b);//������֮�����ֵ�ĸ�����
int IMy_Abs(int a, int b);//������֮�����ֵ

void mysmalls(void);

void Zhidao(void);//ֱ���߶�
uint8 Cirque_or_Cross(uint8 type, uint8 startline);//�ж�Բ������ʮ�֣�����Բ���ĵ�һ��Բ����
uint8 bianxian_tubian(uint8 type);//�жϱ����Ƿ�ͻ��  0��1��
void L_R_tubian(void);//�жϱ����Ƿ�ͻ��  0��1��
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
