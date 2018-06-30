#ifndef __RS485_H
#define __RS485_H			 
#include "sys.h"
#include "hub_motor.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//RS485���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/7
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//ģʽ����
#define RS485_TX_EN		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET)	//485ģʽ���� PA4 д
#define RS485_RX_EN		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET)	//485ģʽ���� PA4 ��

void USART2_Init(u32 bound);
void RS485_Send_Data(USART_TypeDef* UARTx,u8 buf);	
#endif	   

