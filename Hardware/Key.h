#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"                  // Device header

typedef struct Key{
	GPIO_TypeDef * 	GPIOx;
	uint16_t		GPIO_Pin;
}Key_t;

#define Key_Num			4
#define NoKeyDown		0
#define K1_Down			1
#define K2_Down 		2
#define K3_Down			3
#define K4_Down			4
		
void Key_Init(void);

static uint8_t Key_GetValue(void);

uint8_t Key_Tick(void);

uint8_t Key_GetKeyState(void);
#endif
