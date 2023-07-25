/*
 * car_carmera.h
 *
 *  Created on: 2023年7月16日
 *      Author: LENOVO
 */

#ifndef CAR_CAMERA_H_
#define CAR_CAMERA_H_

#include "stdint.h"
#include "math.h"
#include "tim.h"
#include "my_hmi.h"
#include "usart.h"
#include "stdio.h"

extern int setspeed_l;
extern int setspeed_r;
extern int x_set_l;
extern int x_set_r;
extern uint8_t pos_pid_flag; //位置环pid标识
extern uint8_t x_pid;
extern uint8_t x_start;
extern uint8_t wheel_enable;
extern int speed_l;
extern int speed_r;
extern unsigned char buf[64];

extern _Q Q;

void Status_Deal(uint8_t message);


#endif /* CAR_CAMERA_H_ */