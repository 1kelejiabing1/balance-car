/**
 * @file    PWM.h
 * @brief   PWM输出模块头文件
 * @note    使用TIM2产生两路PWM信号控制电机速度
 *         - CH1 (PA0) -> 左电机速度控制
 *         - CH2 (PA1) -> 右电机速度控制
 */

#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

/*============================================================================
 * API函数声明
 *============================================================================*/

/**
 * @defgroup PWM_API PWM API函数
 * @{
 */

/**
 * @brief   PWM初始化函数
 * @param   无
 * @retval  无
 * 
 * @note    配置TIM2的CH1和CH2为PWM输出模式
 *         时钟频率：72MHz / 36 = 2MHz
 *         PWM频率：2MHz / 100 = 20kHz
 *         占空比分辨率：0~100对应0%~100%
 * 
 * @par     引脚分配：
 *          - PA0 (TIM2_CH1) -> 左电机速度控制
 *          - PA1 (TIM2_CH2) -> 右电机速度控制
 * 
 * @par     使用示例：
 * @code
 *   // 在系统初始化时调用
 *   PWM_Init();
 * @endcode
 */
void PWM_Init(void);

/**
 * @brief   设置TIM2_CH1的PWM占空比（左电机）
 * @param   Compare   比较值，范围：0~100
 *                    对应PWM占空比：0%~100%
 * @retval  无
 * 
 * @par     使用示例：
 * @code
 *   // 设置左电机50%占空比
 *   PWM_SetCompare1(50);
 *   
 *   // 停止左电机
 *   PWM_SetCompare1(0);
 * @endcode
 */
void PWM_SetCompare1(uint8_t Compare);

/**
 * @brief   设置TIM2_CH2的PWM占空比（右电机）
 * @param   Compare   比较值，范围：0~100
 *                    对应PWM占空比：0%~100%
 * @retval  无
 * 
 * @par     使用示例：
 * @code
 *   // 设置右电机30%占空比
 *   PWM_SetCompare2(30);
 *   
 *   // 停止右电机
 *   PWM_SetCompare2(0);
 * @endcode
 */
void PWM_SetCompare2(uint8_t Compare);

/** @} */

#endif /* __PWM_H */

