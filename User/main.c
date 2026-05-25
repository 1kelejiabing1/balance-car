/**
 * @file    main.c
 * @brief   平衡车主程序
 * @note    实现两轮自平衡小车的核心控制逻辑
 *         包含角度环、速度环、位置环、转向环等多级PID控制
 */

#include "stm32f10x.h"
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

/*============================================================================
 * 控制模式定义
 *============================================================================*/

/**
 * @brief   平衡小车控制模式
 * @note    0: 无线/蓝牙遥控模式（摇杆控制前进/转向）
 *         1: 按键控制模式（定距前进/定角转向）
 */
uint8_t ControlMode = 1;

/*============================================================================
 * MPU6050传感器数据
 *============================================================================*/

volatile int16_t AX, AY, AZ;        /**< 加速度计原始数据（X/Y/Z轴） */
volatile int16_t GX, GY, GZ;        /**< 陀螺仪原始数据（X/Y/Z轴） */

/**
 * @brief   传感器校准偏移值
 * @note    用于消除传感器安装误差和零点漂移
 */
volatile float InitialAngleAccError = 1.8;      /**< 加速度计角度校准值 */
volatile float InitialGYError = 0;              /**< 陀螺仪Y轴零点偏移 */
volatile float InitialAngleError = -1.5;        /**< 角度初始误差 */
volatile float InitialGZError = -39;            /**< 陀螺仪Z轴零点偏移 */

/*============================================================================
 * PID输出变量
 *============================================================================*/

volatile float SpeedPIDOut;     /**< 速度环PID输出（角度环目标偏移） */
volatile float TurnPIDOut;      /**< 转向环PID输出（左右轮差速值） */
volatile float PositionPIDOut;  /**< 位置环PID输出（速度环目标速度） */

/*============================================================================
 * 角度计算变量
 *============================================================================*/

/**
 * @brief   角度计算相关变量
 * @note    采用互补滤波融合加速度计和陀螺仪数据
 */
volatile float AngleAcc;        /**< 加速度计计算出的角度 */
volatile float AngleGyro_Y;     /**< 陀螺仪积分角度 */
volatile float Alpha = 0.01;    /**< 互补滤波系数（加速度计权重） */
volatile float Angle;           /**< 融合后的俯仰角（用于直立控制） */

/**
 * @brief   转向角度计算变量
 * @note    Z轴陀螺仪积分得到转向角度
 */
volatile float AngleGyro_Z;     /**< Z轴陀螺仪积分角度 */
volatile float TurnAngle;       /**< 当前转向角度（用于定角转向） */

/*============================================================================
 * 电机速度变量
 *============================================================================*/

volatile float LeftMtSpeed_Actual;      /**< 左电机实际速度（转/秒） */
volatile float RightMtSpeed_Actual;     /**< 右电机实际速度（转/秒） */
volatile float AveMtSpeed_Actual;       /**< 左右电机平均速度 */
volatile float DifMtSpeed_Actual;       /**< 左右电机速度差 */

volatile int16_t LeftMtSpeed;           /**< 左电机目标PWM值 */
volatile int16_t RightMtSpeed;          /**< 右电机目标PWM值 */
volatile int16_t AveMtSpeed;            /**< 平均速度目标值 */
volatile int16_t DifMtSpeed;            /**< 差速目标值 */

/*============================================================================
 * 位置环变量
 *============================================================================*/

volatile float Revolution_Left;         /**< 左电机转数（圈） */
volatile float Revolution_Right;        /**< 右电机转数（圈） */
volatile float AveRevolution;           /**< 平均转数增量 */
volatile float AveRevolutionInt;        /**< 累计平均转数（位置反馈） */

/*============================================================================
 * 控制标志位
 *============================================================================*/

volatile uint8_t RunFlag = 0;           /**< 运行标志（1：运行，0：停止） */
volatile uint8_t KNum;                  /**< 按键值（用于按键控制） */

/*============================================================================
 * 无线通信变量
 *============================================================================*/

uint8_t PacketOP;                       /**< NRF24L01数据包操作码 */
int8_t LH, LV;                          /**< 摇杆值（左右/前后） */
int8_t RH, RV;                          /**< 摇杆值（转向/油门） */

/*============================================================================
 * 主函数
 *============================================================================*/

/**
 * @brief   主函数
 * @param   无
 * @retval  无
 * 
 * @note    完成所有模块初始化后进入主循环
 *         主循环负责：遥控数据接收、按键处理、OLED显示
 *         控制算法在TIM1中断中执行（1ms周期）
 */
int main(void)
{
    /*========================== 模块初始化 ==========================*/
    OLED_Init();                    /* OLED显示屏初始化 */
    MPU6050_Init();                 /* MPU6050六轴传感器初始化 */
    Serial_Init(&Serial_Bluetooth); /* 蓝牙串口初始化（用于无线调试） */
    Serial_Init(&Serial);           /* 调试串口初始化（用于printf） */
    Timer_Init();                   /* 定时器初始化（1ms中断） */
    Motor_Init();                   /* 电机驱动初始化 */
    Encoder_Init();                 /* 编码器初始化（测速） */
    
    /* PID控制器初始化 */
    PID_Init(&PID_Angle);           /* 角度环（直立控制） */
    PID_Init(&PID_Speed);           /* 速度环（速度控制） */
    PID_Init(&PID_Turn);            /* 转向环（转向控制） */
    PID_Init(&PID_Position);        /* 位置环（定距控制） */
    PID_Init(&PID_TurnAngle);       /* 转向角度环（定角控制） */
    
    Key_Init();                     /* 按键初始化 */
    NRF24L01_Init();                /* 2.4G无线模块初始化 */
    
    /*========================== 主循环 ==========================*/
    while (1)
    {
        /*------------------ 获取按键状态 ------------------*/
        KNum = Key_GetKeyState();
        
        /*------------------ NRF24L01无线数据接收 ------------------*/
        if (NRF24L01_Receive() == 1)
        {
            PacketOP = NRF24L01_RxPacket[0];
            switch (PacketOP)
            {
                case 0x00:  /* 只接收数据模式 */
                    LV = NRF24L01_RxPacket[2];      /* 前后摇杆值 */
                    RH = NRF24L01_RxPacket[3];      /* 转向摇杆值 */
                    KNum = NRF24L01_RxPacket[5];    /* 按键值（覆盖本地按键） */
                    break;
                    
                case 0x01:  /* 接收数据并回传状态模式 */
                    LV = NRF24L01_RxPacket[2];
                    RH = NRF24L01_RxPacket[3];
                    KNum = NRF24L01_RxPacket[5];
                    
                    /* 回传平衡车状态数据 */
                    NRF24L01_TxPacket[0] = 0x02;                    /* 数据包标识 */
                    NRF24L01_TxPacket[1] = LeftMtSpeed;             /* 左电机PWM */
                    NRF24L01_TxPacket[2] = RightMtSpeed;            /* 右电机PWM */
                    *(float *)&NRF24L01_TxPacket[4] = Angle;        /* 当前角度 */
                    *(float *)&NRF24L01_TxPacket[8] = LeftMtSpeed_Actual;   /* 左轮速度 */
                    *(float *)&NRF24L01_TxPacket[12] = RightMtSpeed_Actual; /* 右轮速度 */
                    NRF24L01_Send();
                    break;
                    
                default:
                    break;
            }
        }
        
        /*------------------ 蓝牙串口数据接收（参数调节）------------------*/
        /* 数据包格式：[数据1,数据2,数据3,...] */
        if (Serial_GetRxFlag(&Serial_Bluetooth))
        {
            char *Tag = strtok((char *)Serial_GetData(&Serial_Bluetooth), ",");
            
            if (strcmp(Tag, "key") == 0)                /* 按键数据包（预留） */
            {
                char *Name = strtok(NULL, ",");
                char *Action = strtok(NULL, ",");
                /* 可在此添加按键响应代码 */
            }
            else if (strcmp(Tag, "slider") == 0)        /* 滑杆数据包（PID参数调节） */
            {
                char *Name = strtok(NULL, ",");
                char *Value = strtok(NULL, ",");
                
                /* 角度环PID参数 */
                if (strcmp(Name, "AngleKp") == 0)       PID_Angle.Kp = atof(Value);
                else if (strcmp(Name, "AngleKi") == 0)  PID_Angle.Ki = atof(Value);
                else if (strcmp(Name, "AngleKd") == 0)  PID_Angle.Kd = atof(Value);
                
                /* 速度环PID参数 */
                else if (strcmp(Name, "SpeedKp") == 0)  PID_Speed.Kp = atof(Value);
                else if (strcmp(Name, "SpeedKi") == 0)  PID_Speed.Ki = atof(Value);
                else if (strcmp(Name, "SpeedKd") == 0)  PID_Speed.Kd = atof(Value);
                
                /* 转向环PID参数 */
                else if (strcmp(Name, "TurnKp") == 0)   PID_Turn.Kp = atof(Value);
                else if (strcmp(Name, "TurnKi") == 0)   PID_Turn.Ki = atof(Value);
                else if (strcmp(Name, "TurnKd") == 0)   PID_Turn.Kd = atof(Value);
                
                /* 角度环死区补偿 */
                else if (strcmp(Name, "Offset") == 0)   PID_Angle.OutOffset = atof(Value);
                
                /* 位置环PID参数 */
                else if (strcmp(Name, "PositionKp") == 0) PID_Position.Kp = atof(Value);
                else if (strcmp(Name, "PositionKi") == 0) PID_Position.Ki = atof(Value);
                else if (strcmp(Name, "PositionKd") == 0) PID_Position.Kd = atof(Value);
                
                /* 转向角度环PID参数 */
                else if (strcmp(Name, "TAKp") == 0)     PID_TurnAngle.Kp = atof(Value);
                else if (strcmp(Name, "TAKi") == 0)     PID_TurnAngle.Ki = atof(Value);
                else if (strcmp(Name, "TAKd") == 0)     PID_TurnAngle.Kd = atof(Value);
                else if (strcmp(Name, "OF") == 0)       PID_TurnAngle.OutOffset = atof(Value);
            }
            else if (strcmp(Tag, "joystick") == 0)      /* 摇杆数据包 */
            {
                LH = atoi(strtok(NULL, ","));
                LV = atoi(strtok(NULL, ","));
                RH = atoi(strtok(NULL, ","));
                RV = atoi(strtok(NULL, ","));
            }
        }
        
        /*------------------ 蓝牙波形输出（调试用）------------------*/
        Serial_Printf(&Serial_Bluetooth, "[plot,%f,%f]", PID_TurnAngle.Target, TurnAngle);
        
        /*------------------ 控制模式执行 ------------------*/
        if (ControlMode == 0)   /* 无线/蓝牙遥控模式 */
        {
            /* 摇杆控制：LV控制前后速度，RH控制转向 */
            PID_SetTarget(&PID_Speed, LV / 20.0);
            PID_SetTarget(&PID_Turn, RH / 20.0);
        }
        
        /*------------------ 按键事件处理 ------------------*/
        switch (KNum)
        {
            case K1_Down:       /* K1：启动/停止 */
            case 9:
                RunFlag = RunFlag ? 0 : 1;
                KNum = NoKeyDown;
                break;
                
            case K2_Down:       /* K2：切换控制模式 */
            case 3:
                ControlMode = ControlMode ? 0 : 1;
                KNum = NoKeyDown;
                break;
                
            case K3_Down:       /* K3：增加目标位置 */
            case 1:
                PID_SetTarget(&PID_Position, PID_GetTarget(&PID_Position) + 10.0);
                PID_SetTarget(&PID_Turn, 0.0);
                break;
                
            case 2:             /* 2：减少目标位置 */
                PID_SetTarget(&PID_Position, PID_GetTarget(&PID_Position) - 10.0);
                PID_SetTarget(&PID_Turn, 0.0);
                break;
                
            case K4_Down:       /* K4：右转 */
            case 7:
                PID_SetTarget(&PID_TurnAngle, PID_GetTarget(&PID_TurnAngle) + 10.0);
                break;
                
            case 8:             /* 8：左转 */
                PID_SetTarget(&PID_TurnAngle, PID_GetTarget(&PID_TurnAngle) - 10.0);
                break;
                
            default:
                break;
        }
        
        /*------------------ OLED显示 ------------------*/
        OLED_Clear();
        
        /* 转向角度环调试信息显示 */
        OLED_Printf(0, 0, OLED_6X8, "RF:%d", RunFlag);
        OLED_Printf(40, 0, OLED_6X8, "CM:%d", ControlMode);
        OLED_Printf(80, 0, OLED_6X8, "K:%d", KNum);
        
        OLED_Printf(0, 8, OLED_6X8, "Kp:%05.2f", PID_TurnAngle.Kp);
        OLED_Printf(0, 16, OLED_6X8, "Ki:%05.2f", PID_TurnAngle.Ki);
        OLED_Printf(0, 24, OLED_6X8, "Kd:%05.2f", PID_TurnAngle.Kd);
        
        OLED_Printf(56, 8, OLED_6X8, "PT:%+04.3f", PID_TurnAngle.Target);
        OLED_Printf(56, 16, OLED_6X8, "PA:%+04.3f", PID_TurnAngle.Actual);
        OLED_Printf(56, 24, OLED_6X8, "PO:%+04.3f", PID_TurnAngle.Out);
        
        OLED_Printf(0, 32, OLED_6X8, "GZ:%+06d", GZ);
        OLED_Printf(0, 40, OLED_6X8, "EZ:%+04.6f", TurnAngle);
        
        OLED_Update();
    }
}

/*============================================================================
 * TIM1更新中断服务函数（1ms周期）
 *============================================================================*/

/**
 * @brief   TIM1更新中断服务函数
 * @note    每1ms进入一次，执行周期性控制任务
 * 
 * @par     任务分配：
 *          - Key_Tick：     按键扫描（10ms执行一次状态更新）
 *          - 速度环/转向环：50ms执行一次（编码器读取、PID计算）
 *          - 角度环：       10ms执行一次（传感器读取、角度计算、电机输出）
 * 
 * @par     控制层级说明：
 *          - 最外层：位置环（定距控制）→ 速度环目标
 *          - 中间层：速度环（速度控制）→ 角度环目标偏移
 *          - 最内层：角度环（直立控制）→ 直接控制电机PWM
 *          - 转向环：      独立控制左右差速
 */
void TIM1_UP_IRQHandler(void)
{
    static uint8_t AnglePID_Cnt = 0;        /* 角度环计数器（每10ms执行） */
    static uint16_t Speed_TurnPID_Cnt = 0;  /* 速度/转向环计数器（每50ms执行） */
    
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        /* 按键扫描（每10ms处理一次状态变化） */
        Key_Tick();
        
        /*==================== 速度环 & 转向环（50ms周期） ====================*/
        if (Speed_TurnPID_Cnt++ >= 50)
        {
            Speed_TurnPID_Cnt = 0;
            
            /* 计算电机转数（圈） */
            /* 编码器每圈44个脉冲，四倍频后176个计数，减速比9.27666 */
            Revolution_Left = Encoder_GetConter(Encoder_SpeedLeft) / 44.0f / 9.27666f;
            Revolution_Right = Encoder_GetConter(Encoder_SpeedRight) / 44.0f / 9.27666f;
            
            /* 计算电机实际速度（转/秒） */
            LeftMtSpeed_Actual = Revolution_Left / 0.05f;
            RightMtSpeed_Actual = Revolution_Right / 0.05f;
            
            /* 计算平均速度和差速 */
            AveRevolution = (Revolution_Left + Revolution_Right);
            AveMtSpeed_Actual = (LeftMtSpeed_Actual + RightMtSpeed_Actual) / 2.0f;
            DifMtSpeed_Actual = LeftMtSpeed_Actual - RightMtSpeed_Actual;
            
            if (RunFlag)
            {
                /*------------------ 位置环（定距前进）------------------*/
                if (ControlMode == 1)
                {
                    AveRevolutionInt += AveRevolution;
                    
                    /* 到达目标位置，停止前进 */
                    if (fabs(AveRevolutionInt - PID_GetTarget(&PID_Position)) < 0.1f)
                    {
                        AveRevolutionInt = 0;
                        PID_SetTarget(&PID_Position, 0);
                    }
                    
                    PID_SetActual(&PID_Position, AveRevolutionInt);
                    PID_Update(&PID_Position);
                    PositionPIDOut = PID_GetOut(&PID_Position);
                    PID_SetTarget(&PID_Speed, PositionPIDOut);
                }
                
                /*------------------ 速度环 ------------------*/
                PID_SetActual(&PID_Speed, AveMtSpeed_Actual);
                PID_Update(&PID_Speed);
                SpeedPIDOut = PID_GetOut(&PID_Speed);
                
                /*------------------ 转向角度环（定角转向）------------------*/
                float TurnAnglePIDOut = 0;
                if (ControlMode == 1)
                {
                    /* 到达目标角度，停止转向 */
                    if (fabs(TurnAngle - PID_GetTarget(&PID_TurnAngle)) < 0.5f)
                    {
                        TurnAngle = 0;
                        PID_SetTarget(&PID_TurnAngle, 0);
                    }
                    
                    PID_SetActual(&PID_TurnAngle, TurnAngle);
                    PID_Update(&PID_TurnAngle);
                    TurnAnglePIDOut = PID_GetOut(&PID_TurnAngle);
                    PID_SetTarget(&PID_Turn, -TurnAnglePIDOut);
                }
                
                /*------------------ 转向环 ------------------*/
                PID_SetActual(&PID_Turn, DifMtSpeed_Actual);
                PID_Update(&PID_Turn);
                TurnPIDOut = PID_GetOut(&PID_Turn);
            }
        }
        
        /*==================== 角度环（10ms周期） ====================*/
        if (AnglePID_Cnt++ >= 10)
        {
            AnglePID_Cnt = 0;
            
            /* 读取MPU6050传感器数据 */
            MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
            GY -= InitialGYError;               /* 陀螺仪零点校准 */
            
            /* 加速度计计算角度（俯仰角） */
            AngleAcc = -atan2(AX, AZ) / 3.14159265f * 180.0f - InitialAngleAccError;
            
            /* 陀螺仪积分角度 */
            AngleGyro_Y = Angle + GY / 32768.0f * 2000.0f * 0.01f;
            
            /* 互补滤波融合角度 */
            Angle = Alpha * AngleAcc + (1.0f - Alpha) * AngleGyro_Y;
            
            /* 计算转向角度（Z轴积分） */
            GZ -= InitialGZError;               /* Z轴陀螺仪零点校准 */
            AngleGyro_Z = TurnAngle + GZ / 32768.0f * 2000.0f * 0.01f;
            TurnAngle = (1.0f - Alpha) * AngleGyro_Z;
            
            if (RunFlag)
            {
                /* 倾角过大保护：超过±50度时停止运行 */
                if (Angle > 50 || Angle < -50)
                {
                    RunFlag = 0;
                }
                
                /*------------------ 角度环（直立控制）------------------*/
                PID_SetActual(&PID_Angle, Angle);           /* 设置当前角度 */
                PID_SetTarget(&PID_Angle, SpeedPIDOut);     /* 速度环输出作为角度环目标 */
                PID_Update(&PID_Angle);                     /* PID计算 */
                
                /* 角度环输出取反后作为平均速度目标 */
                AveMtSpeed = -PID_GetOut(&PID_Angle);
                
                /* 计算左右电机PWM（平均速度 ± 转向差速） */
                LeftMtSpeed = AveMtSpeed + TurnPIDOut / 2;
                RightMtSpeed = AveMtSpeed - TurnPIDOut / 2;
                
                /* PWM限幅 */
                if (LeftMtSpeed > Motor_SpeedMax)   LeftMtSpeed = Motor_SpeedMax;
                else if (LeftMtSpeed < Motor_SpeedMin) LeftMtSpeed = Motor_SpeedMin;
                
                if (RightMtSpeed > Motor_SpeedMax)  RightMtSpeed = Motor_SpeedMax;
                else if (RightMtSpeed < Motor_SpeedMin) RightMtSpeed = Motor_SpeedMin;
                
                /* 输出PWM控制电机 */
                Motor_SetSpeed(Motor_LeftMt, LeftMtSpeed);
                Motor_SetSpeed(Motor_RightMt, RightMtSpeed);
            }
            else
            {
                /* 停止电机 */
                Motor_SetSpeed(Motor_LeftMt, 0);
                Motor_SetSpeed(Motor_RightMt, 0);
            }
        }
        
        /* 清除中断标志位 */
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
