#include "Key.h"

uint8_t KeyState;

Key_t KeyArray[Key_Num] = {
	[0] = {
		.GPIOx = GPIOB,
		.GPIO_Pin = GPIO_Pin_1
	},
	[1] = {
		.GPIOx = GPIOB,
		.GPIO_Pin = GPIO_Pin_0
	},
	[2] = {
		.GPIOx = GPIOA,
		.GPIO_Pin = GPIO_Pin_5
	},
	[3] = {
		.GPIOx = GPIOA,
		.GPIO_Pin = GPIO_Pin_4
	}
};
		

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	for(int i = 0; i < Key_Num;i++)
	{
		GPIO_InitStruct.GPIO_Pin = KeyArray[i].GPIO_Pin;
		GPIO_Init(KeyArray[i].GPIOx, &GPIO_InitStruct);
	}
}

static uint8_t Key_GetValue(void)
{
	for(int i = 0;i < Key_Num;i++)
	{
		if(GPIO_ReadInputDataBit(KeyArray[i].GPIOx, KeyArray[i].GPIO_Pin) == RESET)
		{
			return i + 1;
		}
	}
	return 0;
}

uint8_t Key_Tick(void)
{
	static uint8_t Count = 0;
	static uint8_t CurrState, PrevState;
	if(Count++ >= 10)
	{
		Count = 0;
		PrevState = CurrState;
		CurrState = Key_GetValue();
		if(CurrState == 0 && PrevState != 0)
		{
			KeyState = PrevState;
		}
	}
}

uint8_t Key_GetKeyState(void)
{
	uint8_t temp = KeyState;
	KeyState = NoKeyDown;
	return temp;
}
