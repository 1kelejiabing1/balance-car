/**
 * @file    PWM.c
 * @brief   PWM输出模块实现
 * @note    使用TIM2产生两路PWM信号控制电机速度
 */

#include "PWM.h"

/*============================================================================
 * 函数实现
 *============================================================================*/

/**
 * @brief   PWM初始化函数
 * @param   无
 * @retval  无
 * 
 * @note    此函数完成以下配置：
 *          - 使能GPIOA和TIM2时钟
 *          - 配置PA0和PA1为复用推挽输出模式
 *          - 配置TIM2时基参数
 *          - 配置TIM2_CH1和CH2为PWM模式1
 * 
 * @par     PWM频率计算：
 *          系统时钟 = 72MHz
 *          预分频系数 PSC = 36 - 1 = 35
 *          自动重装载值 ARR = 100 - 1 = 99
 *          
 *          定时器时钟 = 72MHz / (35 + 1) = 2MHz
 *          PWM频率 = 2MHz / (99 + 1) = 20kHz
 *          
 *          占空比 = Compare / (ARR + 1) = Compare / 100
 *          即Compare = 50 时，占空比 = 50%
 * 
 * @par     使用示例：
 * @code
 *   // 在系统初始化时调用
 *   PWM_Init();
 * @endcode
 */
void PWM_Init(void)
{
    /*-------------------------- 时钟使能 --------------------------*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  /* 使能GPIOA时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   /* 使能TIM2时钟 */
    
    /*-------------------------- GPIO引脚配置 --------------------------*/
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;            /* 复用推挽输出模式 */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;     /* PA0 (CH1) 和 PA1 (CH2) */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /*-------------------------- TIM2时基配置 --------------------------*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;    /* 时钟分频系数 = 1 */
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; /* 向上计数模式 */
    TIM_TimeBaseInitStruct.TIM_Period = 100 - 1;                /* ARR = 99，PWM周期100个计数单位 */
    TIM_TimeBaseInitStruct.TIM_Prescaler = 36 - 1;              /* PSC = 35，预分频系数36 */
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;           /* 重复计数器（高级定时器用） */
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
    
    /*-------------------------- PWM输出通道配置 --------------------------*/
    TIM_OCInitTypeDef TIM_OCSInittruct;
    TIM_OCStructInit(&TIM_OCSInittruct);                        /* 结构体初始化 */
    TIM_OCSInittruct.TIM_OCMode = TIM_OCMode_PWM1;              /* PWM模式1 */
    TIM_OCSInittruct.TIM_OCPolarity = TIM_OCPolarity_High;      /* 高电平有效 */
    TIM_OCSInittruct.TIM_Pulse = 20;                            /* 初始占空比20% */
    TIM_OCSInittruct.TIM_OutputState = TIM_OutputState_Enable;  /* 输出使能 */
    
    /* 初始化通道1（左电机速度控制） */
    TIM_OC1Init(TIM2, &TIM_OCSInittruct);
    
    /* 初始化通道2（右电机速度控制） */
    TIM_OC2Init(TIM2, &TIM_OCSInittruct);
    
    /*-------------------------- 使能定时器 --------------------------*/
    TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief   设置TIM2_CH1的PWM占空比（左电机）
 * @param   Compare   比较值，范围：0~100
 *                    对应PWM占空比：0%~100%
 * @retval  无
 * 
 * @note    占空比计算公式：Duty = Compare / 100 × 100%
 *          Compare = 0 时，占空比 0%（电机停止）
 *          Compare = 100 时，占空比 100%（电机全速）
 * 
 * @par     使用示例：
 * @code
 *   // 左电机50%速度
 *   PWM_SetCompare1(50);
 *   
 *   // 左电机全速
 *   PWM_SetCompare1(100);
 * @endcode
 */
void PWM_SetCompare1(uint8_t Compare)
{
    TIM_SetCompare1(TIM2, Compare);
}

/**
 * @brief   设置TIM2_CH2的PWM占空比（右电机）
 * @param   Compare   比较值，范围：0~100
 *                    对应PWM占空比：0%~100%
 * @retval  无
 * 
 * @note    占空比计算公式：Duty = Compare / 100 × 100%
 *          Compare = 0 时，占空比 0%（电机停止）
 *          Compare = 100 时，占空比 100%（电机全速）
 * 
 * @par     使用示例：
 * @code
 *   // 右电机30%速度
 *   PWM_SetCompare2(30);
 *   
 *   // 右电机停止
 *   PWM_SetCompare2(0);
 * @endcode
 */
void PWM_SetCompare2(uint8_t Compare)
{
    TIM_SetCompare2(TIM2, Compare);
}
