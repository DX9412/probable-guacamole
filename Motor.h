
#include "zf_common_headfile.h"

#ifndef CODE_CONTROL_MOTOR_H_
#define CODE_CONTROL_MOTOR_H_


#define KP 0
#define KI 1
#define KD 2


struct PID
{
        float kp;
        float ki;
        float kd;
        float error;
        float last_error;
        float pre_error;
        int16 out;
        int16 last_out;
};
// �ṹ������
typedef struct
{
    int16 l_encoder_pulse;
    int16 r_encoder_pulse;
    bool encoder_enable;
    int16 encodersum_left;
    int16 encodersum_right;
    int16 encodersum_average;
} Encoder_Param;

typedef struct
{
    float iError;                 // ���
    float LastError;              // �ϴ����
    float PrevError;              // ���ϴ����
    float LastData;               // �ϴ�����
    float SumError;               // �ۼ����
    float iErrorHistory[5];       // ��ʷ���
} PID_INFO;

typedef struct
{
    PID_INFO PID_Steer;           // ���PID�ṹ��
//    PID_INFO PID_Drive;           // ���PID�ṹ��
    PID_INFO PID_l;
    PID_INFO PID_r;
} PID_ERECT;

typedef struct
{
        int16 straight;
        int16 bend;
        int16 cross;
        int16 cirque;
        int16 fork;
        int16 lean_cross;
        int16 ramp;
        int16 stop;
        int16 barn;
} SPEED_TYPE;

// ��������
void Encoder_Integral(Encoder_Param *encoder);
void DriveMotor_Ctrl( int16 l_pwm  , int16 r_pwm );
void SteerMotor_Ctrl( int pwm );
int16 Limit_Protection( int16 current_value , int16 min , int16 max );
void Pid_Param_Init( PID_INFO *pid_info );
void Encoder_Param_Init( Encoder_Param *encoder );
void Speed_strategy_Init( SPEED_TYPE *speed_type );
float PID_Position_Steer( PID_INFO *pid_info , float *PID_Parm , float NowPoint , float SetPoint );
//float PID_Position_Motor( PID_INFO *pid_info , float *PID_Parm , float NowPoint , float SetPoint );
int16 L_speed_error(void);
int16 R_speed_error(void);
//void L_speed_pid(struct PID *sptr);
//void R_speed_pid(struct PID *sptr);
void L_speed_pid(PID_INFO *pid_info,float *PID_Parm);
void R_speed_pid(PID_INFO *pid_info,float *PID_Parm);
void Direction_pid(struct PID *sptr);
void Speed_difference_calculate(int16 diff_base);

extern struct PID dpid;
// �ⲿ����
extern Encoder_Param encoder;
extern PID_ERECT pid_type;
extern SPEED_TYPE speed_type;
extern float steerpid_param[4];
//extern float drivepid_param[4];
extern float l_speedpid_param[3];
extern float r_speedpid_param[3];
extern uint8 steermotor_enable;
;
//extern struct PID l_spid, r_spid;

extern int16 L_pid_out;
extern int16 R_pid_out;
extern int16 l_speed_aim;           //����Ŀ���ٶ�
extern int16 r_speed_aim;           //����Ŀ���ٶ�
extern int16 l_speed_now;           //���ֵ�ǰ�ٶ�
extern int16 r_speed_now;           //���ֵ�ǰ�ٶ�
extern float kp_6;
extern float kd_6;
extern int16 differential;





#endif /* CODE_CONTROL_MOTOR_H_ */
