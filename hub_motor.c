#include "hub_motor.h"
#include  <string.h>

  extern u8 RS485_RX_BUF[64]; 		//接收缓冲,最大64个字节
  extern u8 RS485_RX_CNT;   			//接收到的数据长度
  
  par parm_tx;//指令包参数结构体
  instruction inst;//指令包结构体
	
  response res;
	
  char Flag_lock_R=0;//堵转标志
  char Flag_lock_L=0;//堵转标志
  
  uint16_t last_dis_r=0;
  uint16_t now_dis_r=0;
  uint16_t last_dis_l=0;
  uint16_t now_dis_l=0;
  
  uint8_t DirectR=0;//正反转标志
  uint8_t DirectL=0;//正反转标志
	
	
extern void delay(uint32_t ms)

//函数功能:初始化电机
void  Hub_motor_Init(void){

  memset(&parm_tx, 0,sizeof(parm_tx));
  memset(&inst, 0,sizeof(inst));
  memset(&res, 0,sizeof(res));
  
  USART2_Init(256000);
  
  NowDis(USART2,RMotor);
  delay(1);
  NowDis(USART2,LMotor);
}

//函数功能：电机恢复出厂值
void Reset_MOTOR(USART_TypeDef* UARTx){

  parm_tx.count=0;
  Inspack_Set(RESET_CON,parm_tx,RMotor);
  Motor_send_insturction(UARTx,inst);

}

//函数功能：计算校验和
static uint8_t checksum_deal(instruction inst){
  char i;  
  uint16_t checksum=0;
  uint16_t sum=0;
  uint16_t temp=0;
  
  sum=inst.ID+inst.length+inst.Instruction;
  if(inst.parm.count>0){
  for( i=0;i<inst.parm.count;i++){
     temp+=inst.parm.data[i];
     }
  }
  checksum=temp+sum;
  if(checksum>255)
          checksum=(uint8_t)(checksum&0xff);
  else 
          checksum=(uint8_t)checksum;
  return checksum;
  
}

//函数功能：指令包设置
//传入参数：ins:指令，par:参数结构体，
static void Inspack_Set(uint8_t ins,par par,uint8_t id){
	
  inst.head1=0xD5;
  inst.head2=0x5D;
  
  inst.ID=id;

  
  inst.length=par.count+2;
	
  inst.Instruction=ins;
	
  if(par.count!=0){
  inst.parm=par;
  }
  
  inst.checksum=checksum_deal(inst);
  
  memset(&par, 0,par.count);//清空参数，因为有些指令不带参数，清空数据以防误发参数
	
}

//函数功能：发送指令包
//传入参数：指令包结构体
//          要使用的串口
//使用之前必须运行指令包初始化函数！！！！
static void Motor_send_insturction(USART_TypeDef* UARTx,instruction ins){
	char i;
	//发送使能
  RS485_TX_EN;
	
  //发送数据
  RS485_Send_Data(UARTx,ins.head1);
  RS485_Send_Data(UARTx,ins.head2);
  
  RS485_Send_Data(UARTx,ins.ID);
  
  RS485_Send_Data(UARTx,ins.length);
  
  RS485_Send_Data(UARTx,ins.Instruction);
  
  
  for(i =0;i<ins.parm.count;i++){
          RS485_Send_Data(UARTx,ins.parm.data[i]);
    }
	
  RS485_Send_Data(UARTx,ins.checksum);

  while(USART_GetFlagStatus(UARTx,USART_FLAG_TC)==RESET);
		
	//使能接受
  RS485_RX_EN;
  RS485_RX_CNT=0;

	
}

//函数功能：接收到的数据缓存到回应包结构体
//传入参数：RS485通信接收到的数据,接收到的字节数 
//          *data:RS485_R_RX_BUF
//                RS485_L_RX_BUF
//          rx_cnt：                 
void Motor_response(uint8_t *data,uint8_t rx_cnt){
  char i;  
  
  memset(&res,0,sizeof(res));//清空接收结构体
  
  res.head1=data[rx_cnt];
  res.head2=data[rx_cnt+1];
	
  res.ID=data[rx_cnt+2];
  
  res.length=data[rx_cnt+3];
  
  res.error=data[rx_cnt+4];
          
  for(i=0;i<res.length-2;i++){
              res.parm.data[i]=data[rx_cnt+4+i];
          }
  res.parm.count=res.length-2;    
  
	
  res.checksum=data[rx_cnt+4+i+1];
		
  
}

//函数功能：电机运行
//传入参数：speed: 2*PI*R*data/360=speed 单位：m/s  R=0.086m
//          最大转速 4.6m/s
//          direct:1,反转  0,正转 
//当speed=0时电机停止转动，此处通过写0来进行刹车
void motor_run(USART_TypeDef* UARTx,float speed,uint8_t direct,uint8_t id){
	
  float data=speed*360/2*3.14*8.6;
  uint16_t temp=data*100;
  parm_tx.count=3;
  parm_tx.data[0]=0x20;//寄存器地址
  parm_tx.data[1]=(uint8_t)(temp&0xff);//速度值低位
  parm_tx.data[2]=(uint8_t)((temp&0xff00)>>8|(direct<<8));//速度值高位 最高位为符号位
	
  if(id==RMotor){
    DirectR=direct;
    }
    else if(id==LMotor){
    DirectL=direct;
    }
    
    Inspack_Set(WRITE_DATA,parm_tx,id);
    Motor_send_insturction(UARTx,inst);

}

//函数功能：读当前角度值
//左右都要读的时候尽量在一个读完之后延时一段时间再进行度下一个
void ReadDisData(USART_TypeDef* UARTx,uint8_t id){

  parm_tx.count=2;
  parm_tx.data[0]=0x28;//寄存器地址，当前角度寄存器地址
  parm_tx.data[1]=0x02;//读两个字节

  Inspack_Set(READ_DATA,parm_tx,id);
  Motor_send_insturction(UARTx,inst);
  
}

//函数功能：得到当前角度值
uint16_t Read_angle(void){
	
  uint16_t angle;

  angle=res.parm.data[0]+res.parm.data[1]<<8; 
	
  return angle;
  
}

//函数功能：查询收到的角度值
//输入参数：RS485通信接收到的数据,接收到的字节数 
//          *data:RS485_RX_BUF
// 输出参数：16位无符号值
uint16_t Recive_data(uint8_t cnt,uint8_t *buf){
  char i=0;
  for(i=0;i<cnt;i++){
      if(buf[i]==0xd5&&buf[i+1]==0x5d){
        if(buf[i+2]==0x0A){
                
          switch(buf[i+3]){
        
                case STOP:Flag_lock_R=1;Motor_response(buf,i);break;
                default:Flag_lock_R=0;Motor_response(buf,i);break;
                        
          }
        }
        else if(buf[i+2]==0x0B){
        
                switch(buf[i+3]){
        
                case STOP:Flag_lock_L=1;Motor_response(buf,i);break;
                default:Flag_lock_L=0;Motor_response(buf,i);break;
                        
          }
        }
       }
   }
	
	return Read_angle();
}

//函数功能：计算里程值 单位：mm
uint16_t Calculate_distance(uint16_t angle){

  uint16_t dis;
  dis=angle*2*3.14*86/360;
  
  return dis;
}
//函数功能：电机读取当前里程值
//输入参数：UARTx:usart2  
//输出参数；电机当前里程
uint16_t NowDis(USART_TypeDef* UARTx,uint8_t id){
  
  uint16_t temp;
  ReadDisData(UARTx,id);
  delay(2);
  if(id==RMotor){
     now_angle_r=Calculate_distance(Recive_data(RS485_RX_CNT,RS485_RX_BUF));
	}
  else if(id==LMotor){
     now_angle_l=Calculate_distance(Recive_data(RS485_RX_CNT,RS485_RX_BUF));
	}
  
  if(id==RMotor)
    temp=CalculateAngleR();
  else if(id==LMotor)
    temp=CalculateAngleL();
  
  return temp;
}

//函数功能：计算右电机走过的里程值
//输入参数：无  DirectR:1 为反转  0为正转
//输出参数：角度值
uint32_t CalculateDisR(void){

    uint32_t temp;
    
    if(DirectR){  //反转
      if(now_dis_r=<last_dis_r)
        temp=last_dis_r-now_dis_r;       
      
    else 
        temp=65535-now_dis_r+last_dis_r;
    
    last_dis_r=now_dis_r;   //存储本次里程
    } 
    else{
    if(now_dis_r<last_dis_r)
        temp=65535-last_dis_r+now_dis_r;
    else 
        temp=now_dis_r-last_dis_r;
    
    last_dis_r=now_dis_r;
    }
    return temp;

}
//函数功能：计算左电机走过的里程值
//输入参数：无
//输出参数：里程值
uint32_t CalculateDisL(void){

    uint32_t temp;
    if(DirectL){  //正转
    if(now_dis_l>=last_dis_l)
            temp=now_dis_l-last_dis_l;
    else 
            temp=65535-last_dis_l+now_dis_l;
    
    last_dis_r=now_dis_r;
    } 
    else{
    if(now_dis_l>=last_dis_l)
            temp=65535-now_dis_l+last_dis_l;
    else 
            temp=last_dis_l-now_dis_l;
    
    last_dis_r=now_dis_r;
    }
    return temp;

}
//函数功能：查询右电机是否堵转
//输入参数：
//输出参数：
char MotorLockR(void){
	
   return Flag_lock_R;
}
//函数功能：查询左电机是否堵转
//输入参数：
//输出参数：
char MotorLockL(void){
	
	return Flag_lock_L;
}
char MotorLock(void){
  
  return MotorLockR()|MotorLockL();

}


/*
 * 机器人是否在地上检测初始化函数
 */
void init_ontheground_detect(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = ONGROUND_RIGHT | ONGROUND_LEFT;//PD1 PD10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(ONGROUND_GPIO, &GPIO_InitStructure);
}
/*
 * 机器人是否在地上检测函数
 */
_u8 is_ontheground(void)
{
    if ((0 == GPIO_ReadInputDataBit(ONGROUND_GPIO, ONGROUND_RIGHT))
     || (0 == GPIO_ReadInputDataBit(ONGROUND_GPIO, ONGROUND_LEFT))) {
        return 0;
    } else {
        return 1;
    }
}


