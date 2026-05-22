#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"                  // Device header
#include <stdio.h>

typedef struct PID_t
{
	float Target;
	float Actual;
	float ActualPrev;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float ErrorCurrent;
	float ErrorPrev;
	float ErrorInt;
	float ErrorIntMax;
	float ErrorIntMin;
	
	float OutMax;
	float OutMin;
	
	float OutOffset;
}PID_t;

extern PID_t PID_Angle;
extern PID_t PID_Speed;
extern PID_t PID_Turn;
void PID_Init(PID_t* PID);

void PID_Update(PID_t* PID);

void PID_SetKp(PID_t* PID, int8_t Kp);

void PID_SetKi(PID_t* PID, int8_t Ki);

void PID_SetKd(PID_t* PID, int8_t Kd);

float PID_GetKp(PID_t* PID);

float PID_GetKi(PID_t* PID);

float PID_GetKd(PID_t* PID);

void PID_SetTarget(PID_t* PID, float Target);

void PID_SetActual(PID_t* PID, float Actual);

float PID_GetOut(PID_t* PID);
#endif
