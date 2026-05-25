/**
 * @file    Timer.c
 * @brief   定时器驱动模块实现
 * @note    使用TIM1高级定时器产生1ms周期中断
 *         为系统提供精确的时间基准
 */

#include "Timer.h"

/*============================================================================
 * 函数实现
 *============================================================================*/

/**
 * @brief   定时器初始化函数
 * @param   无
 * @retval  无
 * 
 * @note    此函数完成以下配置：
 *          - 使能TIM1时钟
 *          - 配置TIM1时基参数
 *          - 配置更新中断
 *          - 设置NVIC中断优先级
 *          - 使能TIM1计数器
 * 
 * @par     定时周期计算公式：
 *          中断频率 = 系统时钟 / (PSC + 1) / (ARR + 1)
 *          
 *          代入参数：
 *          - 系统时钟 = 72MHz = 72,000,000 Hz
 *          - PSC = 72 - 1 = 71
 *          - ARR = 1000 - 1 = 999
 *          
 *          定时器时钟 = 72,000,000 / 72 = 1,000,000 Hz = 1MHz
 *          中断频率 = 1,000,000 / 1000 = 1000 Hz
 *          中断周期 = 1 / 1000 = 0.001秒 = 1ms
 * 
 * @par     注意：
 *          TIM1是高级定时器，具有重复计数器功能
 *          此处未使用重复计数器，将其设为0
 */
void Timer_Init(void)
{
    /*-------------------------- 时钟使能 --------------------------*/
    /* 使能TIM1时钟（TIM1在APB2总线上） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    /*-------------------------- TIM1时基配置 --------------------------*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;        /* 时钟分频系数 = 1 */
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;    /* 向上计数模式 */
    TIM_TimeBaseInitStruct.TIM_Period = 1000 - 1;                   /* ARR = 999，计数值范围0~999 */
    TIM_TimeBaseInitStruct.TIM_Prescaler = 72 - 1;                  /* PSC = 71，预分频系数72 */
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;               /* 重复计数器（高级定时器特有） */
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);
    
    /*-------------------------- 中断配置 --------------------------*/
    /* 清除更新中断标志位，防止进入中断后立即触发 */
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    
    /* 配置NVIC中断优先级 */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM1_UP_IRQn;                 /* TIM1更新中断通道 */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;                    /* 使能中断通道 */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;          /* 抢占优先级 = 1 */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;                 /* 子优先级 = 1 */
    NVIC_Init(&NVIC_InitStruct);
    
    /* 使能TIM1更新中断 */
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    
    /*-------------------------- 启动定时器 --------------------------*/
    /* 使能TIM1计数器 */
    TIM_Cmd(TIM1, ENABLE);
}
