/**
 * @file    Motor.c
 * @brief   电机驱动模块实现
 * @note    支持左右两个直流电机的方向和速度控制
 *         使用GPIO控制电机方向，PWM控制电机转速
 */

#include "Motor.h"

/*============================================================================
 * 函数实现
 *============================================================================*/

/**
 * @brief   电机驱动初始化函数
 * @param   无
 * @retval  无
 * 
 * @note    此函数完成以下配置：
 *          - 使能GPIOB时钟
 *          - 配置电机控制引脚为推挽输出模式
 *          - 初始化PWM模块（用于速度控制）
 * 
 * @par     引脚分配：
 *          - AIN1 (PB12) - 左电机方向控制引脚1
 *          - AIN2 (PB13) - 左电机方向控制引脚2
 *          - BIN1 (PB14) - 右电机方向控制引脚1
 *          - BIN2 (PB15) - 右电机方向控制引脚2
 *          - PWM1 (PA0)  - 左电机速度控制（TIM2_CH1）
 *          - PWM2 (PA1)  - 右电机速度控制（TIM2_CH2）
 */
void Motor_Init(void)
{
    /* 使能GPIOB时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    /* 配置电机控制引脚为推挽输出模式 */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin = Motor_AIN1 | Motor_AIN2 | Motor_BIN1 | Motor_BIN2;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Motor_GPIOx, &GPIO_InitStruct);
    
    /* 初始化PWM模块（产生电机速度控制信号） */
    PWM_Init();
}

/**
 * @brief   设置电机转速和方向
 * @param   Motor_op   电机选择
 *          @arg Motor_LeftMt  (0) - 左电机
 *          @arg Motor_RightMt (1) - 右电机
 * @param   Speed      转速值，范围：-100 ~ 100
 *          - 正数：正转
 *          - 负数：反转
 *          - 0：停止
 * @retval  无
 * 
 * @note    此函数会自动进行限幅处理，超出范围的值会被限制在边界值
 * 
 * @par     电机驱动逻辑（左电机）：
 *          - Speed > 0：AIN1=1, AIN2=0 → 正转
 *          - Speed < 0：AIN1=0, AIN2=1 → 反转
 *          - PWM占空比 = |Speed|（0~100对应0%~100%）
 * 
 * @par     电机驱动逻辑（右电机）：
 *          - Speed > 0：BIN1=0, BIN2=1 → 正转
 *          - Speed < 0：BIN1=1, BIN2=0 → 反转
 *          - PWM占空比 = |Speed|（0~100对应0%~100%）
 * 
 * @par     使用示例：
 * @code
 *   // 左电机以50%速度正转
 *   Motor_SetSpeed(Motor_LeftMt, 50);
 *   
 *   // 右电机以30%速度反转
 *   Motor_SetSpeed(Motor_RightMt, -30);
 *   
 *   // 停止所有电机
 *   Motor_SetSpeed(Motor_LeftMt, 0);
 *   Motor_SetSpeed(Motor_RightMt, 0);
 * @endcode
 */
void Motor_SetSpeed(uint8_t Motor_op, int8_t Speed)
{
    /* 速度限幅处理 */
    if(Speed > Motor_SpeedMax)
    {
        Speed = Motor_SpeedMax;
    }
    else if(Speed < Motor_SpeedMin)
    {
        Speed = Motor_SpeedMin;
    }
    
    /* 左电机控制 */
    if(Motor_op == Motor_LeftMt)
    {
        if(Speed >= 0)      /* 正转 */
        {
            GPIO_SetBits(Motor_GPIOx, Motor_AIN1);
            GPIO_ResetBits(Motor_GPIOx, Motor_AIN2);
            PWM_SetCompare1(Speed);      /* 设置PWM占空比 */
        }
        else                /* 反转 */
        {
            GPIO_SetBits(Motor_GPIOx, Motor_AIN2);
            GPIO_ResetBits(Motor_GPIOx, Motor_AIN1);
            PWM_SetCompare1(-Speed);     /* 取绝对值设置PWM占空比 */
        }
    }
    /* 右电机控制 */
    else if(Motor_op == Motor_RightMt)
    {
        if(Speed >= 0)      /* 正转 */
        {
            GPIO_SetBits(Motor_GPIOx, Motor_BIN2);
            GPIO_ResetBits(Motor_GPIOx, Motor_BIN1);
            PWM_SetCompare2(Speed);      /* 设置PWM占空比 */
        }
        else                /* 反转 */
        {
            GPIO_SetBits(Motor_GPIOx, Motor_BIN1);
            GPIO_ResetBits(Motor_GPIOx, Motor_BIN2);
            PWM_SetCompare2(-Speed);     /* 取绝对值设置PWM占空比 */
        }
    }
}

