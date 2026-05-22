#include "PWM.h"

void PWM_Init(void)	
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 100 - 1;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 36 - 1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitTypeDef TIM_OCSInittruct;
	TIM_OCStructInit(&TIM_OCSInittruct);
	TIM_OCSInittruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCSInittruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCSInittruct.TIM_Pulse = 20;
	TIM_OCSInittruct.TIM_OutputState = TIM_OutputState_Enable;
	
	TIM_OC1Init(TIM2, &TIM_OCSInittruct);
	TIM_OC2Init(TIM2, &TIM_OCSInittruct);
	
	TIM_Cmd(TIM2, ENABLE);
}

void PWM_SetCompare1(uint8_t Compare)
{
	TIM_SetCompare1(TIM2, Compare);
}

void PWM_SetCompare2(uint8_t Compare)
{
	TIM_SetCompare2(TIM2, Compare);
}
