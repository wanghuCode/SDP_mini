#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "rpstm32.h"
#include "led.h"
  	  
// 左轮 接收缓存区 	
u8 RS485_RX_BUF[64];  	//接收缓冲,最大64个字节.
// 左轮     接收到的数据长度
u8 RS485_RX_CNT=0;   

extern LCD_ShowxNum(u16 x,u16 y,u32 num,u8 len,u8 size,u8 mode);

//USART2中断函数 右轮
void USART2_IRQHandler(void){

	u8 res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
		{
		res =USART_ReceiveData(USART2);	//读取接收到的数据
		
     if(RS485_RX_CNT<64){
		 
		 RS485_RX_BUF[RS485_RX_CNT]=res;
		
		 RS485_RX_CNT++;
		 }			 
    }
} 

//rs485通信初始化
 void USART2_Init(u32 bound){
	
	USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
//	DMA_InitTypeDef DMA_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能UART2   GPIOA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	//UART2_TX   GPIOA2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA2
   
  //USART2_RX	  GPIOA3初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOB11  
	
  //接收发送使能PA4
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//PA4
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA4

   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART2, &USART_InitStructure); //初始化串口
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART2, ENABLE);                    //使能串口
	
		//Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

}

//RS485发送1个字节数据
//buf:发送值
void RS485_Send_Data(USART_TypeDef* UARTx,u8 buf)
{
  
	while(USART_GetFlagStatus(UARTx,USART_FLAG_TC)==RESET);//等待发送结束		
  
	USART_SendData(UARTx,buf); //发送数据

}

