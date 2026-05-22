#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Timer.h"
#include "Encoder.h"
#include "MPU6050.h"
#include "MyI2C.h"
#include "Motor.h"
#include "Key.h"
#include "Serial.h"
#include "PID.h"
#include "NRF24L01.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>


volatile int16_t AX, AY, AZ, GX, GY, GZ;

volatile float InitialAngleAccError = 1.8;
volatile float InitialGYError = 0;

volatile float SpeedPIDOut,TurnPIDOut;

volatile float AngleAcc,AngleGyro;
volatile float Alpha = 0.01;
volatile float Angle;

volatile float LeftMtSpeed_Actual, RightMtSpeed_Actual;			//实际左电机速度，实际右电机速度
volatile float AveMtSpeed_Actual, DifMtSpeed_Actual;				//实际平均电机速度，实际差分电机速度

volatile int16_t LeftMtSpeed, RightMtSpeed;							//本轮调整左电机速度，右电机速度
volatile int16_t AveMtSpeed, DifMtSpeed;								//本轮调整平均电机速度，差分电机速度
volatile uint8_t RunFlag = 0;

volatile uint8_t KNum;

uint8_t PacketOP;
int8_t LH, LV, RH, RV;
int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	MPU6050_Init();		//MPU6050初始化
	Serial_Init(&Serial_Bluetooth);
	Serial_Init(&Serial);
	Timer_Init();		//定时器初始化，1ms定时中断一次
	Motor_Init();
	Encoder_Init();
	PID_Init(&PID_Angle);
	PID_Init(&PID_Speed);
	PID_Init(&PID_Turn);
	Key_Init();
	NRF24L01_Init();	//NRF24L01初始化
	
	while (1)
	{
		/*接收按键、NRF24L01数据包、蓝牙数据*/
		KNum = Key_GetKeyState();

		if(NRF24L01_Receive() == 1)
		{
			PacketOP = NRF24L01_RxPacket[0];
			switch(PacketOP)
			{
				case 0x00:
				{
					//PacketOP == 0x00,只接收数据
					// LH = NRF24L01_RxPacket[1];
					LV = NRF24L01_RxPacket[2];
					RH = NRF24L01_RxPacket[3];
					// RV = NRF24L01_RxPacket[4];
					//如果遥控器也按下了按键，会覆盖小车的按键值
					KNum = NRF24L01_RxPacket[5];
					break;
				}
				case 0x01:
				{
					//PacketOP == 0x01,接收数据后发送平衡车数据
					// LH = NRF24L01_RxPacket[1];
					LV = NRF24L01_RxPacket[2];
					RH = NRF24L01_RxPacket[3];
					// RV = NRF24L01_RxPacket[4];
					KNum = NRF24L01_RxPacket[5];

					//发送数据
					NRF24L01_TxPacket[0] = 0x02;
					NRF24L01_TxPacket[1] = LeftMtSpeed;
					NRF24L01_TxPacket[2] = RightMtSpeed;
					*(float *)&NRF24L01_TxPacket[4] = Angle;
					*(float *)&NRF24L01_TxPacket[8] = LeftMtSpeed_Actual;
					*(float *)&NRF24L01_TxPacket[12] = RightMtSpeed_Actual;
					NRF24L01_Send();
					break;
				}
				default:
				{
					//其他数据包，不做处理
				}
			}
		}
		/*蓝牙串口接收数据包处理*/
		/*规定的数据包格式为：[数据1,数据2,数据3,...]*/
		if (Serial_GetRxFlag(&Serial_Bluetooth))		//如果收到数据包
		{
			char *Tag = strtok((char *)Serial_GetData(&Serial_Bluetooth), ",");	//提取数据1，定义为标签Tag
			if (strcmp(Tag, "key") == 0)					//Tag为key，收到按键数据包
			{
				char *Name = strtok(NULL, ",");				//提取数据2，定义为按键名称
				char *Action = strtok(NULL, ",");			//提取数据3，定义为按键动作
				
				/*此处可执行按键操作，目前程序暂时没用到按键*/
			}
			else if (strcmp(Tag, "slider") == 0)			//Tag为slider，收到滑杆数据包
			{
				char *Name = strtok(NULL, ",");				//提取数据2，定义为滑杆名称
				char *Value = strtok(NULL, ",");			//提取数据3，定义为滑杆值
				
				/*执行滑杆操作*/
				if (strcmp(Name, "AngleKp") == 0)
				{
					PID_Angle.Kp = atof(Value);
				}
				else if (strcmp(Name, "AngleKi") == 0)
				{
					PID_Angle.Ki = atof(Value);
				}
				else if (strcmp(Name, "AngleKd") == 0)
				{
					PID_Angle.Kd = atof(Value);
				}
				else if (strcmp(Name, "SpeedKp") == 0)
				{
					PID_Speed.Kp = atof(Value);
				}
				else if (strcmp(Name, "SpeedKi") == 0)
				{
					PID_Speed.Ki = atof(Value);
				}
				else if (strcmp(Name, "SpeedKd") == 0)
				{
					PID_Speed.Kd = atof(Value);
				}
				else if (strcmp(Name, "TurnKp") == 0)
				{
					PID_Turn.Kp = atof(Value);
				}
				else if (strcmp(Name, "TurnKi") == 0)
				{
					PID_Turn.Ki = atof(Value);
				}
				else if (strcmp(Name, "TurnKd") == 0)
				{
					PID_Turn.Kd = atof(Value);
				}
				else if (strcmp(Name, "Offset") == 0)
				{
					PID_Angle.OutOffset = atof(Value);		//PID_Angle积分偏移解决静止时晃动
				}
			}
			else if (strcmp(Tag, "joystick") == 0)			//Tag为joystick，收到摇杆数据包
			{
				LH = atoi(strtok(NULL, ","));		//提取数据2，定义为摇杆值LH
				LV = atoi(strtok(NULL, ","));		//提取数据3，定义为摇杆值LV
				RH = atoi(strtok(NULL, ","));		//提取数据4，定义为摇杆值RH
				RV = atoi(strtok(NULL, ","));		//提取数据5，定义为摇杆值RV
			}
		}
		/*蓝牙串口打印波形，需配合江协科技蓝牙串口小程序实现波形绘制*/
//		Serial_Printf(&Serial_Bluetooth,"[plot,%f,%f,%f]",PID_Angle.ErrorInt,PID_Speed.ErrorInt,PID_Turn.ErrorInt);
//		Serial_Printf(&Serial_Bluetooth,"[plot,%f,%f]",PID_Angle.Target,Angle);
//		Serial_Printf(&Serial_Bluetooth,"[plot,%f,%f,%f,%f]",PID_Speed.Target,AveMtSpeed_Actual,PID_Turn.Target,DifMtSpeed_Actual);	//绘制PID_Speed.Target和AveMtSpeed_Actual的波形

		/*执行操作*/
		/*执行摇杆操作*/
		PID_SetTarget(&PID_Speed, LV / 20.0);
		PID_SetTarget(&PID_Turn,DifMtSpeed = RH / 20.0);//摇杆值RH缩放后，控制差分PWM，左右转弯控制
		/*接收到遥控器按下按键9，则切换RunFlag*/
		if(KNum == 9 || KNum == K1_Down)
		{
			RunFlag = RunFlag?0:1;
			KNum = NoKeyDown;
		}
		/*OLED显示*/
		OLED_Clear();
		OLED_Printf(0, 0, OLED_6X8, "  Angle");
		OLED_Printf(0, 8, OLED_6X8, "P:%05.2f", PID_Angle.Kp);
		OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", PID_Angle.Ki);
		OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", PID_Angle.Kd);
		OLED_Printf(0, 32, OLED_6X8, "T:%+05.1f", PID_Angle.Target);
		OLED_Printf(0, 40, OLED_6X8, "A:%+05.1f", Angle);
		OLED_Printf(0, 48, OLED_6X8, "O:%+05.0f", PID_Angle.Out);
		OLED_Printf(0, 56, OLED_6X8, "GY:%+05d", GY);
		OLED_Printf(56, 56, OLED_6X8, "of:%+05.0f", PID_Angle.OutOffset);
		OLED_Printf(50, 0, OLED_6X8, "Speed");
		OLED_Printf(50, 8, OLED_6X8, "%05.2f", PID_Speed.Kp);
		OLED_Printf(50, 16, OLED_6X8, "%05.2f", PID_Speed.Ki);
		OLED_Printf(50, 24, OLED_6X8, "%05.2f", PID_Speed.Kd);
		OLED_Printf(50, 32, OLED_6X8, "%+05.1f", PID_Speed.Target);
		OLED_Printf(50, 40, OLED_6X8, "%+05.1f", AveMtSpeed);
		OLED_Printf(50, 48, OLED_6X8, "%+05.0f", PID_Speed.Out);
		OLED_Printf(88, 0, OLED_6X8, "Turn");
		OLED_Printf(88, 8, OLED_6X8, "%05.2f", PID_Turn.Kp);
		OLED_Printf(88, 16, OLED_6X8, "%05.2f", PID_Turn.Ki);
		OLED_Printf(88, 24, OLED_6X8, "%05.2f", PID_Turn.Kd);
		OLED_Printf(88, 32, OLED_6X8, "%+05.1f", PID_Turn.Target);
		OLED_Printf(88, 40, OLED_6X8, "%+05.1f", DifMtSpeed);
		OLED_Printf(88, 48, OLED_6X8, "%+05.0f", PID_Turn.Out);
		OLED_Update();				//OLED更新
	}
}

void TIM1_UP_IRQHandler(void)
{
	static uint8_t AnglePID_Cut = 0;
	static uint8_t Speed_TurnPID_Cnt = 0;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Key_Tick();			//按键计时函数
		if(Speed_TurnPID_Cnt++ >= 50)
		{
			Speed_TurnPID_Cnt = 0;
			
			/*获取编码器的计次值增量，并计算电机旋转速度*/
			/*Encoder_Get函数，可以获取两次读取编码器的计次值增量*/
			/*编码器磁铁旋转一圈，计次增量为44，编码器读取间隔是50ms（0.05s）*/
			/*因此磁铁旋转速度 = 计次增量 / 44 / 0.05，单位是转每秒*/
			/*平衡车使用的电机带有减速箱，减速比为9.27666*/
			/*因此电机输出轴旋转速度 = 磁铁旋转速度 / 9.27666，单位是转每秒*/
			LeftMtSpeed_Actual = Encoder_GetConter(Encoder_SpeedLeft) / 44.0 / 0.05 / 9.27666;
			RightMtSpeed_Actual = Encoder_GetConter(Encoder_SpeedRight) / 44.0 / 0.05 / 9.27666;
			
			AveMtSpeed_Actual = (LeftMtSpeed_Actual + RightMtSpeed_Actual) / 2.0;
			DifMtSpeed_Actual = LeftMtSpeed_Actual - RightMtSpeed_Actual;
			if(RunFlag)
			{
				PID_SetActual(&PID_Speed, AveMtSpeed_Actual);
				PID_Update(&PID_Speed);
				SpeedPIDOut = PID_GetOut(&PID_Speed);
				
				PID_SetActual(&PID_Turn, DifMtSpeed_Actual);
				PID_Update(&PID_Turn);
				TurnPIDOut = PID_GetOut(&PID_Turn);
				
			}
			
		}
		if(AnglePID_Cut++ >= 10)
		{
			AnglePID_Cut = 0;
			MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
			
			GY -= InitialGYError;
			AngleAcc = -atan2(AX,AZ) / 3.14159265 * 180 - InitialAngleAccError;
			AngleGyro = Angle + GY / 32768.0 * 2000 * 0.01;
			
			Angle = Alpha * AngleAcc + (1 - Alpha) * AngleGyro;
			if(RunFlag)
			{
				if(Angle > 50 || Angle < -50)
				{
					RunFlag = 0;
				}
				
				PID_SetActual(&PID_Angle, Angle);

				PID_SetTarget(&PID_Angle, SpeedPIDOut);
				
				PID_Update(&PID_Angle);
				
				AveMtSpeed = -PID_GetOut(&PID_Angle);
				
				LeftMtSpeed = AveMtSpeed + TurnPIDOut / 2;
				RightMtSpeed = AveMtSpeed - TurnPIDOut / 2;
				if(LeftMtSpeed > Motor_SpeedMax){LeftMtSpeed = Motor_SpeedMax;}
				else if(LeftMtSpeed < Motor_SpeedMin){LeftMtSpeed = Motor_SpeedMin;}
				if(RightMtSpeed > Motor_SpeedMax){RightMtSpeed = Motor_SpeedMax;}
				else if(RightMtSpeed < Motor_SpeedMin){RightMtSpeed = Motor_SpeedMin;}
				
				Motor_SetSpeed(Motor_LeftMt, LeftMtSpeed);
				Motor_SetSpeed(Motor_RightMt, RightMtSpeed);
			}
			else
			{
				Motor_SetSpeed(Motor_LeftMt, 0);
				Motor_SetSpeed(Motor_RightMt, 0);
			}
			
		}
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}



