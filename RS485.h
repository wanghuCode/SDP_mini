#ifndef __RS485_H
#define __RS485_H			 
#include "sys.h"
#include "hub_motor.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//RS485驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/7
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//模式控制
#define RS485_TX_EN		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET)	//485模式控制 PA4 写
#define RS485_RX_EN		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET)	//485模式控制 PA4 读

void USART2_Init(u32 bound);
void RS485_Send_Data(USART_TypeDef* UARTx,u8 buf);	
#endif	   

