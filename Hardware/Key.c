/**
 * @file    Key.c
 * @brief   按键驱动模块实现
 * @note    支持4个独立按键，采用状态机实现消抖和边沿检测
 */

#include "Key.h"

/*============================================================================
 * 全局变量定义
 *============================================================================*/

/**
 * @brief   按键状态缓存变量
 * @note    存储经过消抖和边沿检测后的按键值
 *         由Key_Tick函数更新，由Key_GetKeyState函数读取并清零
 */
uint8_t KeyState;

/**
 * @brief   按键配置数组
 * @note    定义每个按键对应的GPIO端口和引脚
 *         索引0~3分别对应K1~K4按键
 */
Key_t KeyArray[Key_Num] = {
    [0] = {                     /* K1按键 - GPIOB_Pin_1 */
        .GPIOx = GPIOB,
        .GPIO_Pin = GPIO_Pin_1
    },
    [1] = {                     /* K2按键 - GPIOB_Pin_0 */
        .GPIOx = GPIOB,
        .GPIO_Pin = GPIO_Pin_0
    },
    [2] = {                     /* K3按键 - GPIOA_Pin_5 */
        .GPIOx = GPIOA,
        .GPIO_Pin = GPIO_Pin_5
    },
    [3] = {                     /* K4按键 - GPIOA_Pin_4 */
        .GPIOx = GPIOA,
        .GPIO_Pin = GPIO_Pin_4
    }
};

/*============================================================================
 * 函数实现
 *============================================================================*/

/**
 * @brief   按键初始化函数
 * @param   无
 * @retval  无
 * 
 * @note    此函数完成以下配置：
 *          - 使能GPIOA和GPIOB时钟
 *          - 将所有按键引脚配置为上拉输入模式
 *          - 上拉输入模式下，按键未按下时引脚为高电平
 *           按下时引脚被拉低（需外部按键电路将引脚接地）
 */
void Key_Init(void)
{
    /* 使能GPIOA和GPIOB时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    /* 配置GPIO引脚为上拉输入模式 */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;   /* 上拉输入模式 */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    
    /* 循环配置所有按键引脚 */
    for(int i = 0; i < Key_Num; i++)
    {
        GPIO_InitStruct.GPIO_Pin = KeyArray[i].GPIO_Pin;
        GPIO_Init(KeyArray[i].GPIOx, &GPIO_InitStruct);
    }
}

/**
 * @brief   获取当前按键值（实时读取）
 * @param   无
 * @retval  当前按下的按键值
 *          @arg NoKeyDown (0)  - 无按键按下
 *          @arg K1_Down   (13) - K1按键按下
 *          @arg K2_Down   (14) - K2按键按下
 *          @arg K3_Down   (15) - K3按键按下
 *          @arg K4_Down   (16) - K4按键按下
 * 
 * @note    此函数为静态函数，仅在本文件内部调用
 *         采用优先级顺序检测：索引越小的按键优先级越高
 *         只返回第一个检测到的按键，实现单按键识别
 */
static uint8_t Key_GetValue(void)
{
    /* 遍历所有按键，按数组顺序检测 */
    for(int i = 0; i < Key_Num; i++)
    {
        /* 检测引脚是否为低电平（按键按下） */
        if(GPIO_ReadInputDataBit(KeyArray[i].GPIOx, KeyArray[i].GPIO_Pin) == RESET)
        {
            uint8_t ret;
            /* 根据按键索引返回对应的按键值 */
            switch(i)
            {
                case 0:
                {
                    ret = K1_Down;
                    break;
                }
                case 1:
                {
                    ret = K2_Down;
                    break;
                }
                case 2:
                {
                    ret = K3_Down;
                    break;
                }
                case 3:
                {
                    ret = K4_Down;
                    break;
                }
                default:
                {
                    /* 错误处理：无效的按键索引 */
                    return NoKeyDown;
                }
            }
            return ret;
        }
    }
    return NoKeyDown;
}

/**
 * @brief   按键状态机处理函数（周期性调用）
 * @param   无
 * @retval  无
 * 
 * @note    此函数需要在定时器中断中周期性调用（建议10ms调用一次）
 *         实现按键消抖和边沿检测功能
 * 
 * @par     工作原理：
 *          1. 每10ms读取一次按键状态
 *          2. 检测到按键值从非0变为0时（按键释放的边沿）
 *          3. 将该按键值存入KeyState变量供主循环读取
 * 
 * @par     消抖说明：
 *          通过10ms间隔采样，可以有效滤除按键抖动
 *          只有连续稳定采样到释放状态才触发事件
 */
uint8_t Key_Tick(void)
{
    static uint8_t Count = 0;           /* 分频计数器 */
    static uint8_t CurrState, PrevState; /* 当前和上一时刻按键值 */
    
    /* 每调用10次才进行一次状态检测（配合10ms中断，即100ms检测一次） */
    if(Count++ >= 10)
    {
        Count = 0;
        PrevState = CurrState;               /* 保存上一时刻状态 */
        CurrState = Key_GetValue();          /* 读取当前按键值 */
        
        /* 边沿检测：当前无按键按下，但上一时刻有按键按下 */
        /* 即检测到按键释放的下降沿 */
        if(CurrState == 0 && PrevState != 0)
        {
            KeyState = PrevState;            /* 记录本次按键事件 */
        }
    }
}

/**
 * @brief   获取按键状态（读取后自动清零）
 * @param   无
 * @retval  按键值
 *          @arg NoKeyDown (0)  - 无按键事件
 *          @arg K1_Down   (13) - K1按键被按下并释放
 *          @arg K2_Down   (14) - K2按键被按下并释放
 *          @arg K3_Down   (15) - K3按键被按下并释放
 *          @arg K4_Down   (16) - K4按键被按下并释放
 * 
 * @note    此函数在主循环中调用，用于获取按键事件
 *         读取后会立即清零KeyState，避免重复处理同一按键事件
 *         返回值13~16的设计是为了避开遥控器按键值0~12的冲突
 * 
 * @par     使用示例：
 * @code
 *   void main_loop(void)
 *   {
 *       uint8_t key = Key_GetKeyState();
 *       switch(key)
 *       {
 *           case K1_Down:
 *               // 处理K1按键事件
 *               break;
 *           case K2_Down:
 *               // 处理K2按键事件
 *               break;
 *       }
 *   }
 * @endcode
 */
uint8_t Key_GetKeyState(void)
{
    uint8_t temp = KeyState;    /* 保存当前按键状态 */
    KeyState = NoKeyDown;       /* 清除按键状态，防止重复处理 */
    return temp;                /* 返回保存的按键状态 */
}
