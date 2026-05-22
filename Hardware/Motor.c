#include "Motor.h"


void Motor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = Motor_AIN1 | Motor_AIN2 | Motor_BIN1 | Motor_BIN2;	//AIN1,AIN2,BIN10,BIN2
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(Motor_GPIOx, &GPIO_InitStruct);
	
	PWM_Init();
}

void Motor_SetSpeed(uint8_t Motor_op, int8_t Speed)
{
	if(Speed > Motor_SpeedMax)
	{
		Speed = Motor_SpeedMax;
	}
	else if(Speed < Motor_SpeedMin)
	{
		Speed = Motor_SpeedMin;
	}
	
	if(Motor_op == Motor_LeftMt)
	{
		if(Speed >= 0)
		{
			GPIO_SetBits(Motor_GPIOx, Motor_AIN1);
			GPIO_ResetBits(Motor_GPIOx, Motor_AIN2);
			PWM_SetCompare1(Speed);
		}
		else
		{
			GPIO_SetBits(Motor_GPIOx, Motor_AIN2);
			GPIO_ResetBits(Motor_GPIOx, Motor_AIN1);
			PWM_SetCompare1(-Speed);
		}
		
	}
	else if(Motor_op == Motor_RightMt)
	{
		if(Speed >= 0)
		{
			GPIO_SetBits(Motor_GPIOx, Motor_BIN2);
			GPIO_ResetBits(Motor_GPIOx, Motor_BIN1);
			PWM_SetCompare2(Speed);
		}
		else
		{
			GPIO_SetBits(Motor_GPIOx, Motor_BIN1);
			GPIO_ResetBits(Motor_GPIOx, Motor_BIN2);
			PWM_SetCompare2(-Speed);
		}
	}
}

