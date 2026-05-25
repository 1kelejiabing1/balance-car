/**
 * @file    Timer.h
 * @brief   定时器驱动模块头文件
 * @note    使用TIM1产生1ms周期中断，作为系统时间基准
 *         中断服务函数在main.c中实现
 */

#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"

/*============================================================================
 * API函数声明
 *============================================================================*/

/**
 * @defgroup Timer_API 定时器API函数
 * @{
 */

/**
 * @brief   定时器初始化函数
 * @param   无
 * @retval  无
 * 
 * @note    配置TIM1产生1ms周期中断
 *         中断优先级：抢占优先级1，子优先级1
 * 
 * @par     定时时间计算：
 *          系统时钟 = 72MHz
 *          预分频系数 PSC = 72 - 1 = 71
 *          自动重装载值 ARR = 1000 - 1 = 999
 *          
 *          定时器时钟 = 72MHz / (71 + 1) = 1MHz
 *          中断周期 = (999 + 1) / 1MHz = 1ms
 * 
 * @par     使用示例：
 * @code
 *   // 在系统初始化时调用
 *   Timer_Init();
 *   
 *   // 中断服务函数在main.c中实现
 *   void TIM1_UP_IRQHandler(void)
 *   {
 *       if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
 *       {
 *           // 每1ms执行一次的任务
 *           Key_Tick();      // 按键扫描（每10ms处理一次）
 *           // 其他周期性任务...
 *           TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
 *       }
 *   }
 * @endcode
 */
void Timer_Init(void);

/** @} */

#endif /* __TIMER_H */
