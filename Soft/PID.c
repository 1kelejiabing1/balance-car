#include "PID.h"

PID_t PID_Angle = {
	.Kp = 4.0,//3.2
	.Ki = 0.15,//0.3
	.Kd = 5.0,//3.0
	
	.Target = 0,
	
	.OutMax = 100,
	.OutMin = -100,
	
	.OutOffset =4,
	
	.ErrorIntMax = 400,
	.ErrorIntMin = -400
};

PID_t PID_Speed = {
	.Kp = 1.1,
	.Ki = 0.05,
	.Kd = 0,
	
	.Target = 0,
	
	.OutMax = 20,
	.OutMin = -20,
	
	.ErrorIntMax = 100,
	.ErrorIntMin = -100
};

PID_t PID_Turn = {
	.Kp = 4,
	.Ki = 3,
	.Kd = 0,
	
	.OutMax = 50,
	.OutMin = -50,
	
	.ErrorIntMax = 20,
	.ErrorIntMin = -20
};

void PID_Init(PID_t* PID)
{
	PID->Actual = 0;
	PID->ActualPrev = 0;
	PID->Out = 0;
	PID->ErrorCurrent = 0;
	PID->ErrorInt = 0;
	PID->ErrorPrev = 0;
}

void PID_Update(PID_t* PID)
{
	if(PID == NULL)
	{
		return;
	}
	PID->ErrorPrev = PID->ErrorCurrent;
	PID->ErrorCurrent = PID->Target - PID->Actual;
	
	if(PID->Ki)
	{
		PID->ErrorInt += PID->ErrorCurrent;
		
		if(PID->ErrorInt > PID->ErrorIntMax) PID->ErrorInt = PID->ErrorIntMax;
		else if(PID->ErrorInt < PID->ErrorIntMin) PID->ErrorInt = PID->ErrorIntMin;
	}
	else
	{
		PID->ErrorInt = 0;
	}
	
	PID->Out = PID->Kp * PID->ErrorCurrent
				+ PID->Ki * PID->ErrorInt
				- PID->Kd * (PID->Actual - PID->ActualPrev);	//Î¢·ÖÏÈÐÐ
//				+ PID->Kd * (PID->ErrorCurrent - PID->ErrorPrev);
	
	if(PID->Out > 0)PID->Out += PID->OutOffset;
	else if(PID->Out < 0)PID->Out -= PID->OutOffset;

	if(PID->Out > PID->OutMax) PID->Out = PID->OutMax;
	else if(PID->Out < PID->OutMin) PID->Out = PID->OutMin;
	
	PID->ActualPrev = PID->Actual;
}

void PID_SetKp(PID_t* PID, int8_t Kp)
{
	if(PID == NULL)
	{
		return;
	}
	PID->Kp = Kp;
}

void PID_SetKi(PID_t* PID, int8_t Ki)
{
	if(PID == NULL)
	{
		return;
	}
	PID->Ki = Ki;
}

void PID_SetKd(PID_t* PID, int8_t Kd)
{
	if(PID == NULL)
	{
		return;
	}
	PID->Kd = Kd;
}

float PID_GetKp(PID_t* PID)
{
	if(PID == NULL)
	{
		return 0;
	}
	return PID->Kp;
}

float PID_GetKi(PID_t* PID)
{
	if(PID == NULL)
	{
		return 0;
	}
	return PID->Ki;
}

float PID_GetKd(PID_t* PID)
{
	if(PID == NULL)
	{
		return 0;
	}
	return PID->Kd;
}

void PID_SetTarget(PID_t* PID, float Target)
{
	if(PID == NULL)
	{
		return;
	}
	PID->Target = Target;
}

void PID_SetActual(PID_t* PID, float Actual)
{
	if(PID == NULL)
	{
		return;
	}
	PID->Actual = Actual;
}

float PID_GetOut(PID_t* PID)
{
	if(PID == NULL)
	{
		return 0;
	}
	return PID->Out;
}
