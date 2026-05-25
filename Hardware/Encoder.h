/**
 * @file    Encoder.h
 * @brief   编码器驱动模块头文件
 * @note    使用TIM3和TIM4的编码器模式读取左右电机转速
 *         编码器连接：左电机-TIM3(PA6,PA7)，右电机-TIM4(PB6,PB7)
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

/*============================================================================
 * 宏定义
 *============================================================================*/

/**
 * @defgroup Encoder_Select 编码器选择
 * @{
 */
#define Encoder_SpeedLeft   0   /**< 左电机编码器选择 */
#define Encoder_SpeedRight  1   /**< 右电机编码器选择 */
/** @} */

/*============================================================================
 * API函数声明
 *============================================================================*/

/**
 * @defgroup Encoder_API 编码器API函数
 * @{
 */

/**
 * @brief   编码器初始化函数
 * @param   无
 * @retval  无
 * 
 * @note    此函数完成以下配置：
 *          - TIM3编码器模式：左电机编码器，上升沿采样
 *          - TIM4编码器模式：右电机编码器，CH1下降沿/CH2上升沿采样
 *          - 计数器自动重装载值为65535（16位最大计数值）
 *          - 输入滤波系数0xF，滤除高频噪声
 * 
 * @par     引脚分配：
 *          - 左电机编码器：PA6 (TIM3_CH1), PA7 (TIM3_CH2)
 *          - 右电机编码器：PB6 (TIM4_CH1), PB7 (TIM4_CH2)
 * 
 * @par     使用示例：
 * @code
 *   // 在系统初始化时调用
 *   Encoder_Init();
 * @endcode
 */
void Encoder_Init(void);

/**
 * @brief   获取编码器计数值（读取后自动清零）
 * @param   op   编码器选择
 *          @arg Encoder_SpeedLeft  (0) - 左电机编码器
 *          @arg Encoder_SpeedRight (1) - 右电机编码器
 * @retval  编码器计数值（范围：-32768~32767）
 * 
 * @note    编码器采用正交解码模式，计数值可正可负
 *         正数表示正转，负数表示反转
 *         读取后自动清零，适合周期性读取计算转速
 * 
 * @par     转速计算公式：
 *          - 编码器磁铁每圈脉冲数：44
 *          - 电机减速比：9.27666
 *          - 电机轴每圈对应编码器计数值：44 × 4 × 9.27666 = 1632（四倍频）
 *          - 实际转速(rps) = (计数值 / 1632) / 采样周期(秒)
 * 
 * @par     使用示例：
 * @code
 *   // 每50ms读取一次编码器值
 *   int16_t left_count = Encoder_GetConter(Encoder_SpeedLeft);
 *   int16_t right_count = Encoder_GetConter(Encoder_SpeedRight);
 *   
 *   // 计算转速（转/秒）
 *   float left_speed = left_count / 1632.0f / 0.05f;
 * @endcode
 */
int16_t Encoder_GetConter(uint8_t op);

/** @} */

#endif /* __ENCODER_H */
