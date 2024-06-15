/*
 * interrupt.h
 *
 *  Created on: 2023��11��20��
 *      Author: Richard Qian
 */

#ifndef CODE_CONTROL_INTERRUPT_H_
#define CODE_CONTROL_INTERRUPT_H_


// ��������
void task_1ms();
void task_2ms();
void task_4ms();
void task_8ms();
void task_10ms();
void task_20ms();


// �ⲿ����
extern int16 steerpwm_output;
extern float motorpwm_output;



#endif /* CODE_CONTROL_INTERRUPT_H_ */
