#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"                  // Device header

#define Encoder_SpeedLeft 0
#define Encoder_SpeedRight 1

void Encoder_Init();
int16_t Encoder_GetConter(uint8_t op);

#endif
