/**
 * @file    PID.h
 * @brief   PID控制器模块头文件
 * @note    实现位置式PID控制算法，支持微分先行、积分限幅、死区补偿
 *         提供5个PID实例：角度环、速度环、位置环、转向环、转向角度环
 */

#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>

/*============================================================================
 * 数据结构定义
 *============================================================================*/

/**
 * @brief   PID控制器结构体
 * @note    封装PID控制器的所有参数和状态变量
 */
typedef struct PID_t
{
    /* 目标值与实际值 */
    float Target;           /**< 目标值（设定值） */
    float Actual;           /**< 实际值（反馈值） */
    float ActualPrev;       /**< 上一时刻实际值（用于微分先行） */
    float Out;              /**< PID控制器输出值 */
    
    /* PID系数 */
    float Kp;               /**< 比例系数（Proportional） */
    float Ki;               /**< 积分系数（Integral） */
    float Kd;               /**< 微分系数（Derivative） */
    
    /* 误差变量 */
    float ErrorCurrent;     /**< 当前误差 = Target - Actual */
    float ErrorPrev;        /**< 上一时刻误差 */
    float ErrorInt;         /**< 误差积分累积值 */
    float ErrorIntMax;      /**< 积分限幅上限 */
    float ErrorIntMin;      /**< 积分限幅下限 */
    
    /* 输出限幅 */
    float OutMax;           /**< 输出上限 */
    float OutMin;           /**< 输出下限 */
    
    /* 补偿参数 */
    float OutOffset;        /**< 输出死区补偿值（克服电机静态阻力） */
} PID_t;

/*============================================================================
 * 宏定义
 *============================================================================*/

/**
 * @brief   积分分离阈值（未使用，预留）
 * @note    当误差绝对值大于此阈值时，积分项不起作用
 *         可防止启动时积分饱和
 */
#define KiThreshold 5.0

/*============================================================================
 * 外部全局变量声明
 *============================================================================*/

/**
 * @defgroup PID_Instances PID实例
 * @{
 */

/**
 * @brief   角度环PID（直立环）
 * @note    用于保持车体直立平衡
 *         输入：当前角度（融合值），输出：电机基础PWM
 */
extern PID_t PID_Angle;

/**
 * @brief   速度环PID
 * @note    用于控制小车前进/后退速度
 *         输入：平均电机速度，输出：角度环目标角度偏移
 */
extern PID_t PID_Speed;

/**
 * @brief   位置环PID
 * @note    用于控制小车定距前进
 *         输入：电机累计转数，输出：速度环目标速度
 *         注意：Ki=0，稳态误差通过OutOffset补偿
 */
extern PID_t PID_Position;

/**
 * @brief   转向环PID
 * @note    用于控制小车左右转向速度
 *         输入：左右电机速度差，输出：转向PWM叠加值
 */
extern PID_t PID_Turn;

/**
 * @brief   转向角度环PID
 * @note    用于控制小车定角转向
 *         输入：Z轴陀螺仪积分角度，输出：转向环目标角度
 */
extern PID_t PID_TurnAngle;

/** @} */

/*============================================================================
 * API函数声明
 *============================================================================*/

/**
 * @defgroup PID_API PID控制器API函数
 * @{
 */

/**
 * @brief   PID结构体初始化
 * @param   PID    PID结构体指针
 * @retval  无
 * 
 * @note    清零所有状态变量（Actual, Out, ErrorCurrent, ErrorInt等）
 *         不影响PID系数和目标值
 * 
 * @par     使用示例：
 * @code
 *   PID_Init(&PID_Angle);
 *   PID_Init(&PID_Speed);
 * @endcode
 */
void PID_Init(PID_t* PID);

/**
 * @brief   PID计算函数（核心算法）
 * @param   PID    PID结构体指针
 * @retval  无
 * 
 * @note    采用位置式PID算法，公式如下：
 *          Out = Kp × e(k) + Ki × Σe(k) - Kd × (Actual - ActualPrev)
 *          
 *          @arg 微分先行：对测量值微分而非误差微分，抑制噪声
 *          @arg 积分限幅：防止积分饱和
 *          @arg 死区补偿：克服电机启动时的静摩擦力
 * 
 * @par     使用示例：
 * @code
 *   // 设置目标值和实际值
 *   PID_SetTarget(&PID_Angle, 0);
 *   PID_SetActual(&PID_Angle, current_angle);
 *   
 *   // 执行PID计算
 *   PID_Update(&PID_Angle);
 *   
 *   // 获取输出值
 *   float output = PID_GetOut(&PID_Angle);
 * @endcode
 */
void PID_Update(PID_t* PID);

/**
 * @brief   设置比例系数Kp
 * @param   PID    PID结构体指针
 * @param   Kp     比例系数（正数）
 * @retval  无
 * 
 * @note    Kp越大，响应越快，但过大会引起振荡
 */
void PID_SetKp(PID_t* PID, int8_t Kp);

/**
 * @brief   设置积分系数Ki
 * @param   PID    PID结构体指针
 * @param   Ki     积分系数（正数）
 * @retval  无
 * 
 * @note    Ki用于消除稳态误差，过大会引起积分饱和
 *         当Ki=0时，积分项不起作用
 */
void PID_SetKi(PID_t* PID, int8_t Ki);

/**
 * @brief   设置微分系数Kd
 * @param   PID    PID结构体指针
 * @param   Kd     微分系数（正数）
 * @retval  无
 * 
 * @note    Kd用于预测误差变化趋势，增加系统阻尼
 *         过大会放大噪声
 */
void PID_SetKd(PID_t* PID, int8_t Kd);

/**
 * @brief   获取比例系数Kp
 * @param   PID    PID结构体指针
 * @retval  比例系数Kp
 */
float PID_GetKp(PID_t* PID);

/**
 * @brief   获取积分系数Ki
 * @param   PID    PID结构体指针
 * @retval  积分系数Ki
 */
float PID_GetKi(PID_t* PID);

/**
 * @brief   获取微分系数Kd
 * @param   PID    PID结构体指针
 * @retval  微分系数Kd
 */
float PID_GetKd(PID_t* PID);

/**
 * @brief   设置PID目标值
 * @param   PID       PID结构体指针
 * @param   Target    目标值
 * @retval  无
 * 
 * @par     使用示例：
 * @code
 *   // 设置角度环目标为直立（0度）
 *   PID_SetTarget(&PID_Angle, 0);
 *   
 *   // 设置速度环目标为前进（10转/秒）
 *   PID_SetTarget(&PID_Speed, 10.0f);
 * @endcode
 */
void PID_SetTarget(PID_t* PID, float Target);

/**
 * @brief   设置PID实际测量值
 * @param   PID       PID结构体指针
 * @param   Actual    实际值
 * @retval  无
 * 
 * @note    通常在传感器读取后调用
 */
void PID_SetActual(PID_t* PID, float Actual);

/**
 * @brief   获取PID目标值
 * @param   PID    PID结构体指针
 * @retval  目标值
 */
float PID_GetTarget(PID_t* PID);

/**
 * @brief   获取PID输出值
 * @param   PID    PID结构体指针
 * @retval  PID输出值
 * 
 * @note    调用PID_Update后获取计算结果
 */
float PID_GetOut(PID_t* PID);

/** @} */

#endif /* __PID_H */
