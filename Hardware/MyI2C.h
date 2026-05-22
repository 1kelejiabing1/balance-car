#ifndef __MYI2C_H
#define __MYI2C_H

#include "stm32f10x.h"                  // Device header



/*
typedef struct
{
	uint32_t	RCC_GPIOx,
	uint16_t	SCL_Pin,
	uint16_t	SDA_Pin
}myi2c_t
*/
#define	RCC_GPIOx	RCC_APB2Periph_GPIOB
#define SCL_Pin 	GPIO_Pin_10
#define SDA_Pin 	GPIO_Pin_11
#define	GPIOPort 	GPIOB


void MyI2C_Init(void);

static void MyI2C_W_SCL(uint8_t Bitval);

static void MyI2C_W_SDA(uint8_t Bitval);

static uint8_t MyI2C_R_SCL(void);

static uint8_t MyI2C_R_SDA(void);

void MyI2C_Start(void);

void MyI2C_Stop(void);

void MyI2C_SendByte(uint8_t Byte);

uint8_t MyI2C_ReceiveByte(void);

uint8_t MyI2C_ReceiveAck(void);

void MyI2C_SendAck(uint8_t Ack);

#endif
