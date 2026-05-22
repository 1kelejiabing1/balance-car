#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"                  // Device header

#include "PWM.h"

#define Motor_LeftMt 0
#define Motor_RightMt 1
#define Motor_GPIOx	GPIOB
#define Motor_AIN1	GPIO_Pin_12
#define Motor_AIN2	GPIO_Pin_13 
#define Motor_BIN1	GPIO_Pin_14 
#define Motor_BIN2	GPIO_Pin_15
#define Motor_SpeedMax 100
#define Motor_SpeedMin -100


void Motor_Init(void);

void Motor_SetSpeed(uint8_t Motor_op, int8_t Speed);




#endif
