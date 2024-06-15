/*
 * interrupt.h
 *
 *  Created on: 2023年11月20日
 *      Author: Richard Qian
 */

#ifndef CODE_CONTROL_INTERRUPT_H_
#define CODE_CONTROL_INTERRUPT_H_


// 函数声明
void task_1ms();
void task_2ms();
void task_4ms();
void task_8ms();
void task_10ms();
void task_20ms();


// 外部声明
extern int16 steerpwm_output;
extern float motorpwm_output;



#endif /* CODE_CONTROL_INTERRUPT_H_ */
