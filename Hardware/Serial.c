#include "Serial.h"

Serial_Device_t Serial = {
    .USARTx = USART1,
    .GPIOx = GPIOA,
    .RCC_USART = RCC_APB2Periph_USART1,
    .RCC_GPIO = RCC_APB2Periph_GPIOA,
    .Pin_Tx = GPIO_Pin_9,
    .Pin_Rx = GPIO_Pin_10,
    .IRQn = USART1_IRQn,
    .PreemptPriority = 1,
    .SubPriority = 1,
};

Serial_Device_t Serial_Bluetooth = {
    .USARTx = USART2,
    .GPIOx = GPIOA,
    .RCC_USART = RCC_APB1Periph_USART2,
    .RCC_GPIO = RCC_APB2Periph_GPIOA,
    .Pin_Tx = GPIO_Pin_2,
    .Pin_Rx = GPIO_Pin_3,
    .IRQn = USART2_IRQn,
    .PreemptPriority = 2,    // 中断优先级
    .SubPriority = 2,
};



void Serial_Init(Serial_Device_t* serial_dev)
{
    // 空指针校验
    if (serial_dev == NULL)
    {
        return;
    }
    
    // 时钟使能（区分 APB1 / APB2）
    if(serial_dev->RCC_USART == RCC_APB2Periph_USART1)
    {
        RCC_APB2PeriphClockCmd(serial_dev->RCC_USART, ENABLE);
    }
    else
    {
        RCC_APB1PeriphClockCmd(serial_dev->RCC_USART, ENABLE);
    }
    RCC_APB2PeriphClockCmd(serial_dev->RCC_GPIO, ENABLE);
    
    // 配置 TX 引脚：复用推挽输出
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin = serial_dev->Pin_Tx;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(serial_dev->GPIOx, &GPIO_InitStruct);
    
    // 配置 RX 引脚：上拉输入
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Pin = serial_dev->Pin_Rx;
    GPIO_Init(serial_dev->GPIOx, &GPIO_InitStruct);
    
    // 串口基础配置：波特率9600，8N1
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = SERIAL_BaudRate;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_Init(serial_dev->USARTx, &USART_InitStruct);
    
    // 使能接收中断
    USART_ITConfig(serial_dev->USARTx, USART_IT_RXNE, ENABLE);
    
    // 配置中断优先级
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = serial_dev->IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = serial_dev->PreemptPriority;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = serial_dev->SubPriority;
    NVIC_Init(&NVIC_InitStruct);
	
    // 开启串口
    USART_Cmd(serial_dev->USARTx, ENABLE);
}

/**
 * @brief  串口发送单字节
 * @param  serial_dev: 串口设备指针
 * @param  Byte: 待发送字节
 * @retval 无
 */
void Serial_SendByte(Serial_Device_t* serial_dev, uint8_t Byte)
{
    if (serial_dev == NULL) return;
    
    USART_SendData(serial_dev->USARTx, Byte);
    while(USART_GetFlagStatus(serial_dev->USARTx, USART_FLAG_TXE) == RESET);
}

void Serial_SendString(Serial_Device_t* serial_dev, char *String)
{
	if (serial_dev == NULL) return;
	for(int i = 0;String[i] != '\0';i++)
	{
		Serial_SendByte(serial_dev, String[i]);
	}
}

void Serial_Printf(Serial_Device_t* serial_dev, char *format, ...)
{
	char String[100];				//定义字符数组
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);					//结束变量arg
	Serial_SendString(serial_dev, String);		//串口发送字符数组（字符串）
}

uint8_t Serial_GetRxFlag(Serial_Device_t* serial_dev)
{
	if (serial_dev == NULL) return 0;
	if(serial_dev->RxFlag)
	{
		serial_dev->RxFlag = 0;		// 清除接收完成标志
		return 1;
	}
	return 0;
}
/**
 * @brief  获取一帧完整数据
 * @param  serial_dev: 串口设备指针
 * @retval 数据缓冲区指针
 */
uint8_t* Serial_GetData(Serial_Device_t* serial_dev)
{
    if (serial_dev == NULL) return NULL;
    return serial_dev->RxBuf;
}

void Serial_RxIrqHandler(Serial_Device_t* serial_dev, uint8_t ReceiveData)
{
	if (serial_dev == NULL) return;
	if(ReceiveData == '[' && serial_dev->RxState == 0)
	{
		serial_dev->RxBufcnt = 0;
		serial_dev->RxState = 1;
	}
	else if(serial_dev->RxState == 1 && serial_dev->RxBufcnt < SERIAL_BufLength)
	{
		if(ReceiveData == ']')
		{
			serial_dev->RxBuf[serial_dev->RxBufcnt++] = '\0';
			serial_dev->RxState = 0;
			serial_dev->RxFlag = 1;
		}
		else
		{
			serial_dev->RxBuf[serial_dev->RxBufcnt++] = ReceiveData;
		}
	}
	
	
}

/**
 * @brief  USART1 中断服务函数（普通串口模块）
 * @retval 无
 */
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(Serial.USARTx, USART_IT_RXNE) == SET)
    {
        uint8_t ReceiveData = USART_ReceiveData(Serial.USARTx);
        Serial_RxIrqHandler(&Serial, ReceiveData);
        USART_ClearITPendingBit(Serial.USARTx, USART_IT_RXNE);
    }
}

/**
 * @brief  USART2 中断服务函数（蓝牙模块）
 * @retval 无
 */
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(Serial_Bluetooth.USARTx, USART_IT_RXNE) == SET)
    {
        uint8_t ReceiveData = USART_ReceiveData(Serial_Bluetooth.USARTx);
		Serial_RxIrqHandler(&Serial_Bluetooth, ReceiveData);
        USART_ClearITPendingBit(Serial_Bluetooth.USARTx, USART_IT_RXNE);
    }
}

int fputc(int ch, FILE *f)
{
	Serial_SendByte(&Serial, ch);
	return ch;
}