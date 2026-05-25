/**
 * @file    Encoder.c
 * @brief   编码器驱动模块实现
 * @note    使用STM32定时器的编码器接口模式读取正交编码器信号
 *         支持四倍频计数，提高转速测量精度
 */

#include "Encoder.h"

/*============================================================================
 * 函数实现
 *============================================================================*/

/**
 * @brief   编码器初始化函数
 * @param   无
 * @retval  无
 * 
 * @note    左电机编码器（TIM3）配置：
 *          - 编码器模式：TI1和TI2双沿计数（四倍频）
 *          - 输入极性：CH1和CH2均为上升沿采样
 *          - 计数器范围：0~65535（自动重装载）
 * 
 * @note    右电机编码器（TIM4）配置：
 *          - 编码器模式：TI1和TI2双沿计数（四倍频）
 *          - 输入极性：CH1下降沿采样，CH2上升沿采样
 *          - 计数器范围：0~65535（自动重装载）
 * 
 * @note    为什么右电机使用不同极性？
 *          由于电机安装方向或编码器AB相接线差异，需要通过调整极性
 *          使两个编码器在电机同向转动时输出同向计数值
 */
void Encoder_Init(void)
{
    /*==========================================================================
     * 左电机编码器初始化（TIM3）
     *==========================================================================*/
    
    /* 使能GPIOA和TIM3时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    /* 配置编码器引脚为上拉输入模式 */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;          /* 上拉输入模式 */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; /* PA6(TIM3_CH1), PA7(TIM3_CH2) */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* 配置TIM3时基 */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;      /* 时钟分频系数 = 1 */
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;  /* 向上计数模式 */
    TIM_TimeBaseInitStruct.TIM_Period = 65536 - 1;                /* ARR = 65535，最大计数值 */
    TIM_TimeBaseInitStruct.TIM_Prescaler = 1 - 1;                 /* PSC = 0，不分频 */
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;             /* 重复计数器（高级定时器用） */
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
    
    /* 配置输入捕获滤波 */
    TIM_ICInitTypeDef TIM_ICInitStruct;
    TIM_ICStructInit(&TIM_ICInitStruct);                         /* 结构体初始化 */
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;                /* 配置通道1 */
    TIM_ICInitStruct.TIM_ICFilter = 0xF;                         /* 滤波系数0xF，滤除高频噪声 */
    TIM_ICInit(TIM3, &TIM_ICInitStruct);
    
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;                /* 配置通道2 */
    TIM_ICInitStruct.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM3, &TIM_ICInitStruct);
    
    /* 配置编码器接口模式 */
    /* 模式：TI1和TI2双沿计数（四倍频），CH1和CH2均为上升沿采样 */
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    /* 使能TIM3计数器 */
    TIM_Cmd(TIM3, ENABLE);
    
    /*==========================================================================
     * 右电机编码器初始化（TIM4）
     *==========================================================================*/
    
    /* 使能GPIOB和TIM4时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    /* 配置编码器引脚为上拉输入模式 */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;          /* 上拉输入模式 */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; /* PB6(TIM4_CH1), PB7(TIM4_CH2) */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* 配置TIM4时基 */
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = 65536 - 1;      /* ARR = 65535，最大计数值 */
    TIM_TimeBaseInitStruct.TIM_Prescaler = 1 - 1;       /* PSC = 0，不分频 */
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
    
    /* 配置输入捕获滤波 */
    TIM_ICStructInit(&TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM4, &TIM_ICInitStruct);
    
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStruct.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM4, &TIM_ICInitStruct);
    
    /* 配置编码器接口模式 */
    /* 模式：TI1和TI2双沿计数（四倍频），CH1下降沿采样，CH2上升沿采样 */
    /* 注意：这里使用下降沿采样是为了使左右编码器计数方向一致 */
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Rising);
    
    /* 使能TIM4计数器 */
    TIM_Cmd(TIM4, ENABLE);
}

/**
 * @brief   获取编码器计数值（读取后自动清零）
 * @param   op   编码器选择
 *          @arg Encoder_SpeedLeft  (0) - 左电机编码器（TIM3）
 *          @arg Encoder_SpeedRight (1) - 右电机编码器（TIM4）
 * @retval  编码器计数值（范围：-32768~32767）
 * 
 * @note    读取后自动清零计数器，适合周期性采样
 *         由于STM32定时器计数器为16位，返回值使用int16_t类型
 *         最大值32767对应约20圈，超出会溢出，需在合理采样周期内读取
 * 
 * @par     编码器计数值方向说明：
 *          - 正计数值：电机正转
 *          - 负计数值：电机反转
 *          - 计数值绝对值：表示转动角度（四倍频后）
 * 
 * @par     使用示例：
 * @code
 *   // 在速度环中周期性调用（例如50ms周期）
 *   int16_t left_encoder = Encoder_GetConter(Encoder_SpeedLeft);
 *   int16_t right_encoder = Encoder_GetConter(Encoder_SpeedRight);
 *   
 *   // 计算实际转速
 *   // 编码器磁铁每圈44个脉冲，四倍频后为176个计数
 *   // 减速比9.27666，电机轴每圈对应1632个计数
 *   float left_speed_rps = left_encoder / 1632.0f / 0.05f;  // 转/秒
 * @endcode
 */
int16_t Encoder_GetConter(uint8_t op)
{
    int16_t temp = 0;
    
    if(op == Encoder_SpeedLeft)         /* 左电机编码器 */
    {
        temp = TIM_GetCounter(TIM3);    /* 读取TIM3当前计数值 */
        TIM_SetCounter(TIM3, 0);        /* 清零计数器，准备下次采样 */
        return temp;
    }
    else if(op == Encoder_SpeedRight)   /* 右电机编码器 */
    {
        temp = TIM_GetCounter(TIM4);    /* 读取TIM4当前计数值 */
        TIM_SetCounter(TIM4, 0);        /* 清零计数器，准备下次采样 */
        return temp;
    }
    
    return 0;                           /* 无效参数，返回0 */
}
