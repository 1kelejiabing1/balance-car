/**
 * @file    Motor.h
 * @brief   电机驱动模块头文件
 * @note    支持左右两个直流电机的方向和速度控制
 *         速度控制依赖PWM模块（TIM2_CH1/CH2）
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "PWM.h"

/*============================================================================
 * 宏定义
 *============================================================================*/

/**
 * @defgroup Motor_Select 电机选择
 * @{
 */
#define Motor_LeftMt     0   /**< 左电机选择 */
#define Motor_RightMt    1   /**< 右电机选择 */
/** @} */

/**
 * @defgroup Motor_Pin 电机控制引脚定义
 * @note   使用GPIOB的Pin12~Pin15控制电机方向
 * @{
 */
#define Motor_GPIOx      GPIOB           /**< 电机控制引脚所在GPIO端口 */
#define Motor_AIN1       GPIO_Pin_12     /**< 左电机方向控制引脚1 */
#define Motor_AIN2       GPIO_Pin_13     /**< 左电机方向控制引脚2 */
#define Motor_BIN1       GPIO_Pin_14     /**< 右电机方向控制引脚1 */
#define Motor_BIN2       GPIO_Pin_15     /**< 右电机方向控制引脚2 */
/** @} */

/**
 * @defgroup Motor_Speed 电机速度范围
 * @note   速度值范围：-100 ~ 100，对应PWM占空比0%~100%
 * @{
 */
#define Motor_SpeedMax   100     /**< 最大转速（正向满速） */
#define Motor_SpeedMin   -100    /**< 最小转速（反向满速） */
/** @} */

/*============================================================================
 * API函数声明
 *============================================================================*/

/**
 * @defgroup Motor_API 电机API函数
 * @{
 */

/**
 * @brief   电机驱动初始化函数
 * @param   无
 * @retval  无
 * 
 * @note    使用前必须调用此函数初始化电机驱动
 *         会同时初始化PWM模块产生速度控制信号
 * 
 * @par     使用示例：
 * @code
 *   // 在系统初始化时调用
 *   Motor_Init();
 * @endcode
 */
void Motor_Init(void);

/**
 * @brief   设置电机转速和方向
 * @param   Motor_op   电机选择
 *          @arg Motor_LeftMt  (0) - 左电机
 *          @arg Motor_RightMt (1) - 右电机
 * @param   Speed      转速值，范围：Motor_SpeedMin ~ Motor_SpeedMax
 *          - 正数：正转
 *          - 负数：反转
 *          - 0：停止
 * @retval  无
 * 
 * @note    速度值会自动限幅到有效范围内
 * 
 * @par     电机驱动真值表：
 *          | 电机 | Speed | AIN1/BIN1 | AIN2/BIN2 | PWM   | 转向 |
 *          |------|-------|-----------|-----------|-------|------|
 *          | 左   | >0    | 1         | 0         | Speed | 正转 |
 *          | 左   | <0    | 0         | 1         | -Speed| 反转 |
 *          | 右   | >0    | 0         | 1         | Speed | 正转 |
 *          | 右   | <0    | 1         | 0         | -Speed| 反转 |
 * 
 * @par     使用示例：
 * @code
 *   // 左电机50%速度正转
 *   Motor_SetSpeed(Motor_LeftMt, 50);
 *   
 *   // 右电机30%速度反转
 *   Motor_SetSpeed(Motor_RightMt, -30);
 * @endcode
 */
void Motor_SetSpeed(uint8_t Motor_op, int8_t Speed);

/** @} */

#endif /* __MOTOR_H */
