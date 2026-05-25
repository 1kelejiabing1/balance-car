/**
 * @file    PID.c
 * @brief   PID控制器模块实现
 * @note    实现位置式PID控制算法
 *         包含5个PID实例：角度环、速度环、位置环、转向环、转向角度环
 */

#include "PID.h"

/*============================================================================
 * PID实例定义
 *============================================================================*/

/**
 * @brief   角度环PID参数（直立环）
 * @note    用于保持车体直立，是平衡车最核心的控制环
 *         
 * @par     参数说明：
 *          - Kp = 4.0：    角度误差响应系数，小车倾斜时产生回复力矩
 *          - Ki = 0.15：   积分系数，消除静态角度误差（防止长时间倾斜）
 *          - Kd = 5.0：    微分系数，增加阻尼，抑制振荡
 *          - OutOffset = 4：电机死区补偿，克服电机启动静摩擦力
 *          
 * @par     工作原理：
 *          角度环输出 = 电机PWM值（直接控制电机）
 *          小车前倾时，电机正转使车轮向前追赶重心
 *          小车后倾时，电机反转使车轮向后追赶重心
 */
PID_t PID_Angle = {
    .Kp = 4.0,          /* 比例系数：角度误差响应 */
    .Ki = 0.15,         /* 积分系数：消除稳态误差 */
    .Kd = 5.0,          /* 微分系数：增加阻尼，抑制振荡 */
    
    .Target = 0,        /* 目标角度：0度为直立 */
    
    .OutMax = 100,      /* 输出上限：最大PWM值 */
    .OutMin = -100,     /* 输出下限：最小PWM值 */
    
    .OutOffset = 4,     /* 死区补偿：克服电机启动静摩擦力 */
    
    .ErrorIntMax = 400, /* 积分限幅上限：防止积分饱和 */
    .ErrorIntMin = -400 /* 积分限幅下限 */
};

/**
 * @brief   速度环PID参数
 * @note    用于控制小车前进/后退速度
 *         速度环输出作为角度环的目标角度偏移
 * 
 * @par     参数说明：
 *          - Kp = 1.1：   速度误差响应系数
 *          - Ki = 0.05：  积分系数，消除速度稳态误差
 *          - Kd = 0：     微分系数（未使用）
 *          
 * @par     工作原理：
 *          速度环输出 = 目标倾角偏移量
 *          想前进时，速度环让小车略微前倾，角度环驱动车轮追赶重心
 *          想停止时，速度环让小车回到直立姿态
 */
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

/**
 * @brief   位置环PID参数
 * @note    用于控制小车定距前进
 *         位置环输出作为速度环的目标速度
 * 
 * @par     参数说明：
 *          - Kp = 0.43：  位置误差响应系数
 *          - Ki = 0：     积分系数设为0（避免目标位置前后摆动）
 *          - Kd = 1.6：   微分系数，提供阻尼
 *          - OutOffset = 0.73：死区补偿，克服电机静摩擦
 * 
 * @par     设计说明：
 *          位置环使用Ki会导致小车在目标位置前后摆动
 *          稳态误差通过OutOffset补偿实现
 */
PID_t PID_Position = {
    .Kp = 0.43,
    .Ki = 0,            /* 位置环不使用积分，避免振荡 */
    .Kd = 1.6,
    
    .Target = 0,
    
    .OutMax = 5,
    .OutMin = -5,
    
    .OutOffset = 0.73,  /* 死区补偿，克服电机静摩擦 */
    
    .ErrorIntMax = 1000,
    .ErrorIntMin = -1000
};

/**
 * @brief   转向环PID参数
 * @note    用于控制小车左右转向速度
 *         转向环输出叠加到左右电机PWM上（差速转向）
 * 
 * @par     参数说明：
 *          - Kp = 4：     转向速度误差响应系数
 *          - Ki = 3：     积分系数，消除转向稳态误差
 *          - Kd = 0：     微分系数（未使用）
 *          
 * @par     工作原理：
 *          转向输出正值：左电机增加PWM，右电机减少PWM（左转）
 *          转向输出负值：左电机减少PWM，右电机增加PWM（右转）
 */
PID_t PID_Turn = {
    .Kp = 4,
    .Ki = 3,
    .Kd = 0,
    
    .OutMax = 50,
    .OutMin = -50,
    
    .ErrorIntMax = 20,
    .ErrorIntMin = -20
};

/**
 * @brief   转向角度环PID参数
 * @note    用于控制小车定角转向
 *         转向角度环输出作为转向环的目标值
 * 
 * @par     参数说明：
 *          - Kp = 0.1：   转向角度误差响应系数
 *          - Ki = 0：     积分系数（未使用，通过反复调节实现）
 *          - Kd = 0：     微分系数（未使用）
 *          
 * @par     工作原理：
 *          输入：Z轴陀螺仪积分角度（当前朝向）
 *          输出：转向环目标速度
 *          实现精确旋转指定角度
 */
PID_t PID_TurnAngle = {
    .Kp = 0.1,
    .Ki = 0,
    .Kd = 0,
    
    .Target = 0,
    
    .OutMax = 5,
    .OutMin = -5,
    
    .OutOffset = 0,
    
    .ErrorIntMax = 1000000,
    .ErrorIntMin = -1000000
};

/*============================================================================
 * 函数实现
 *============================================================================*/

/**
 * @brief   PID结构体初始化
 * @param   PID    PID结构体指针
 * @retval  无
 * 
 * @note    清零所有状态变量，保留PID系数和目标值不变
 */
void PID_Init(PID_t* PID)
{
    if (PID == NULL) return;
    
    PID->Actual = 0;
    PID->ActualPrev = 0;
    PID->Out = 0;
    PID->ErrorCurrent = 0;
    PID->ErrorInt = 0;
    PID->ErrorPrev = 0;
}

/**
 * @brief   PID计算函数（核心算法）
 * @param   PID    PID结构体指针
 * @retval  无
 * 
 * @note    位置式PID算法公式：
 *          Out = Kp × e(k) + Ki × Σe(k) - Kd × (y(k) - y(k-1))
 *          
 *          @arg  e(k) = Target - Actual：当前误差
 *          @arg  Σe(k)：误差积分累积值
 *          @arg  y(k) - y(k-1)：测量值变化率（微分先行）
 * 
 * @par     微分先行的优点：
 *          对测量值微分而非误差微分，可以避免目标值突变时产生微分冲击
 *          例如：目标值从0突变到100，传统微分会产生巨大的输出跳变
 * 
 * @par     积分限幅的作用：
 *          防止系统长时间存在误差时积分项无限增大
 *          避免积分饱和导致的系统响应迟钝
 * 
 * @par     死区补偿的作用：
 *          电机存在静摩擦力，较小的PWM输出无法驱动电机
 *          在输出上叠加一个偏移值，克服静摩擦
 */
void PID_Update(PID_t* PID)
{
    if (PID == NULL)
    {
        return;
    }
    
    /* 保存上一时刻误差 */
    PID->ErrorPrev = PID->ErrorCurrent;
    
    /* 计算当前误差 */
    PID->ErrorCurrent = PID->Target - PID->Actual;
    
    /* 积分累加 + 积分限幅 */
    if (PID->Ki)                                    /* Ki不为0时才积分 */
    {
        PID->ErrorInt += PID->ErrorCurrent;
        
        /* 积分限幅：防止积分饱和 */
        if (PID->ErrorInt > PID->ErrorIntMax)
            PID->ErrorInt = PID->ErrorIntMax;
        else if (PID->ErrorInt < PID->ErrorIntMin)
            PID->ErrorInt = PID->ErrorIntMin;
    }
    else
    {
        PID->ErrorInt = 0;                          /* Ki=0时清空积分项 */
    }
    
    /* 位置式PID输出（微分先行：对测量值微分） */
    PID->Out = PID->Kp * PID->ErrorCurrent           /* 比例项 */
                + PID->Ki * PID->ErrorInt            /* 积分项 */
                - PID->Kd * (PID->Actual - PID->ActualPrev);  /* 微分项（微分先行） */
    
    /* 电机死区补偿：克服电机静摩擦力 */
    if (PID->Out > 0)
        PID->Out += PID->OutOffset;
    else if (PID->Out < 0)
        PID->Out -= PID->OutOffset;
    
    /* 输出限幅：限制输出范围 */
    if (PID->Out > PID->OutMax)
        PID->Out = PID->OutMax;
    else if (PID->Out < PID->OutMin)
        PID->Out = PID->OutMin;
    
    /* 保存上一时刻测量值，供下次微分使用 */
    PID->ActualPrev = PID->Actual;
}

/**
 * @brief   设置比例系数Kp
 * @param   PID    PID结构体指针
 * @param   Kp     比例系数
 * @retval  无
 */
void PID_SetKp(PID_t* PID, int8_t Kp)
{
    if (PID == NULL) return;
    PID->Kp = Kp;
}

/**
 * @brief   设置积分系数Ki
 * @param   PID    PID结构体指针
 * @param   Ki     积分系数
 * @retval  无
 */
void PID_SetKi(PID_t* PID, int8_t Ki)
{
    if (PID == NULL) return;
    PID->Ki = Ki;
}

/**
 * @brief   设置微分系数Kd
 * @param   PID    PID结构体指针
 * @param   Kd     微分系数
 * @retval  无
 */
void PID_SetKd(PID_t* PID, int8_t Kd)
{
    if (PID == NULL) return;
    PID->Kd = Kd;
}

/**
 * @brief   获取比例系数Kp
 * @param   PID    PID结构体指针
 * @retval  比例系数Kp
 */
float PID_GetKp(PID_t* PID)
{
    if (PID == NULL) return 0;
    return PID->Kp;
}

/**
 * @brief   获取积分系数Ki
 * @param   PID    PID结构体指针
 * @retval  积分系数Ki
 */
float PID_GetKi(PID_t* PID)
{
    if (PID == NULL) return 0;
    return PID->Ki;
}

/**
 * @brief   获取微分系数Kd
 * @param   PID    PID结构体指针
 * @retval  微分系数Kd
 */
float PID_GetKd(PID_t* PID)
{
    if (PID == NULL) return 0;
    return PID->Kd;
}

/**
 * @brief   设置PID目标值
 * @param   PID       PID结构体指针
 * @param   Target    目标值
 * @retval  无
 */
void PID_SetTarget(PID_t* PID, float Target)
{
    if (PID == NULL) return;
    PID->Target = Target;
}

/**
 * @brief   设置PID实际测量值
 * @param   PID       PID结构体指针
 * @param   Actual    实际值
 * @retval  无
 */
void PID_SetActual(PID_t* PID, float Actual)
{
    if (PID == NULL) return;
    PID->Actual = Actual;
}

/**
 * @brief   获取PID输出值
 * @param   PID    PID结构体指针
 * @retval  PID输出值
 */
float PID_GetOut(PID_t* PID)
{
    if (PID == NULL) return 0;
    return PID->Out;
}

/**
 * @brief   获取PID目标值
 * @param   PID    PID结构体指针
 * @retval  目标值
 */
float PID_GetTarget(PID_t* PID)
{
    if (PID == NULL) return 0;
    return PID->Target;
}
