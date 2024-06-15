/*
 * image_init.h
 *
 *  Created on: 2024Äê5ÔÂ17ÈÕ
 *      Author: gaga
 */
#include "zf_common_headfile.h"

#ifndef CODE_CAMERA_IMAGE_INIT_H_
#define CODE_CAMERA_IMAGE_INIT_H_

extern double Tx;
extern double Ty;
void Transform_Point1(int x, int y);
void Transform_Point2(int x, int y);
void Transform_Line(int in[][2],float out[][2],int *in_num,int *out_num);
#endif /* CODE_CAMERA_IMAGE_INIT_H_ */
