/**
 * @file    Key.h
 * @brief   按键驱动模块头文件
 * @note    支持4个独立按键，采用边沿检测方式触发
 *         按键值范围13~16，避免与遥控器按键值0~12冲突
 */

#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

/*============================================================================
 * 数据结构定义
 *============================================================================*/

/**
 * @brief   按键配置结构体
 * @note    用于定义单个按键的硬件连接信息
 */
typedef struct Key {
    GPIO_TypeDef*   GPIOx;      /**< 按键连接的GPIO端口 */
    uint16_t        GPIO_Pin;   /**< 按键连接的GPIO引脚 */
} Key_t;

/*============================================================================
 * 宏定义
 *============================================================================*/

/**
 * @defgroup Key_Config 按键配置
 * @{
 */

/**
 * @brief   按键数量
 * @note    当前配置为4个独立按键
 */
#define Key_Num     4

/** @} */

/**
 * @defgroup Key_Values 按键值定义
 * @note    按键值从13开始，避免与遥控器按键值0~12冲突
 *         遥控器按键值范围：0~12，保留给无线遥控功能
 * @{
 */
#define NoKeyDown   0   /**< 无按键事件 */
#define K1_Down     13  /**< K1按键事件 */
#define K2_Down     14  /**< K2按键事件 */
#define K3_Down     15  /**< K3按键事件 */
#define K4_Down     16  /**< K4按键事件 */

/** @} */

/*============================================================================
 * API函数声明
 *============================================================================*/

/**
 * @defgroup Key_API 按键API函数
 * @{
 */

/**
 * @brief   按键初始化函数
 * @param   无
 * @retval  无
 * 
 * @note    使用前必须调用此函数初始化按键硬件
 *         配置所有按键引脚为上拉输入模式
 * 
 * @par     使用示例：
 * @code
 *   // 在系统初始化时调用
 *   Key_Init();
 * @endcode
 */
void Key_Init(void);

/**
 * @brief   获取当前按键值（实时读取，静态函数）
 * @param   无
 * @retval  当前按下的按键值（优先级顺序）
 * @note    此函数为静态函数，仅供本模块内部调用
 *         用于获取按键的实时状态
 */
static uint8_t Key_GetValue(void);

/**
 * @brief   按键状态机处理函数（周期性调用）
 * @param   无
 * @retval  无
 * 
 * @note    必须在定时器中断中周期性调用（建议10ms调用一次）
 *         实现按键消抖和边沿检测功能
 *         调用频率建议：每10ms调用一次，内部每100ms（10次）检测一次状态变化
 * 
 * @par     使用示例：
 * @code
 *   // 在10ms定时器中断中调用
 *   void TIM1_UP_IRQHandler(void)
 *   {
 *       Key_Tick();  // 每10ms调用一次
 *   }
 * @endcode
 */
uint8_t Key_Tick(void);

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
 * @note    在主循环中轮询调用，获取按键事件
 *         每次读取后会自动清零，确保每个按键事件只被处理一次
 * 
 * @par     使用示例：
 * @code
 *   void main(void)
 *   {
 *       Key_Init();
 *       while(1)
 *       {
 *           uint8_t key = Key_GetKeyState();
 *           if(key == K1_Down)
 *           {
 *               // 执行K1按键对应的操作
 *           }
 *       }
 *   }
 * @endcode
 */
uint8_t Key_GetKeyState(void);

/** @} */

#endif /* __KEY_H */
