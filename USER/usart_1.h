
#ifndef __USART1_H
#define __USART1_H

//头文件包含
#include <stm32f10x.h>
#include "stdio.h"
#include <string.h>


typedef struct 
{
	uint8_t Send_Userdata;
	uint8_t Send_Status;
	uint8_t Send_Senser;
	uint8_t Send_PID1;
	uint8_t Send_PID2;
	uint8_t Send_PID3;
	uint8_t Send_RCData;
	uint8_t Send_Offset;
	uint8_t Send_MotoPwm;

}dt_flag_t;


//函数声明
void USART1_Send_Byte(u16 dat);
uint8_t USART1_Receive_Byte(void);
void Usart1_GPIO_Init(void);
void Usart1_Configuration(uint32_t BaudRate); 
void Send_Senser(void);

#endif



