#include "hub_motor.h"
#include  <string.h>

  extern u8 RS485_RX_BUF[64]; 		//���ջ���,���64���ֽ�
  extern u8 RS485_RX_CNT;   			//���յ������ݳ���
  
  par parm_tx;//ָ��������ṹ��
  instruction inst;//ָ����ṹ��
	
  response res;
	
  char Flag_lock_R=0;//��ת��־
  char Flag_lock_L=0;//��ת��־
  
  uint16_t last_dis_r=0;
  uint16_t now_dis_r=0;
  uint16_t last_dis_l=0;
  uint16_t now_dis_l=0;
  
  uint8_t DirectR=0;//����ת��־
  uint8_t DirectL=0;//����ת��־
	
	
extern void delay(uint32_t ms)

//��������:��ʼ�����
void  Hub_motor_Init(void){

  memset(&parm_tx, 0,sizeof(parm_tx));
  memset(&inst, 0,sizeof(inst));
  memset(&res, 0,sizeof(res));
  
  USART2_Init(256000);
  
  NowDis(USART2,RMotor);
  delay(1);
  NowDis(USART2,LMotor);
}

//�������ܣ�����ָ�����ֵ
void Reset_MOTOR(USART_TypeDef* UARTx){

  parm_tx.count=0;
  Inspack_Set(RESET_CON,parm_tx,RMotor);
  Motor_send_insturction(UARTx,inst);

}

//�������ܣ�����У���
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

//�������ܣ�ָ�������
//���������ins:ָ�par:�����ṹ�壬
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
  
  memset(&par, 0,par.count);//��ղ�������Ϊ��Щָ�����������������Է��󷢲���
	
}

//�������ܣ�����ָ���
//���������ָ����ṹ��
//          Ҫʹ�õĴ���
//ʹ��֮ǰ��������ָ�����ʼ��������������
static void Motor_send_insturction(USART_TypeDef* UARTx,instruction ins){
	char i;
	//����ʹ��
  RS485_TX_EN;
	
  //��������
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
		
	//ʹ�ܽ���
  RS485_RX_EN;
  RS485_RX_CNT=0;

	
}

//�������ܣ����յ������ݻ��浽��Ӧ���ṹ��
//���������RS485ͨ�Ž��յ�������,���յ����ֽ��� 
//          *data:RS485_R_RX_BUF
//                RS485_L_RX_BUF
//          rx_cnt��                 
void Motor_response(uint8_t *data,uint8_t rx_cnt){
  char i;  
  
  memset(&res,0,sizeof(res));//��ս��սṹ��
  
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

//�������ܣ��������
//���������speed: 2*PI*R*data/360=speed ��λ��m/s  R=0.086m
//          ���ת�� 4.6m/s
//          direct:1,��ת  0,��ת 
//��speed=0ʱ���ֹͣת�����˴�ͨ��д0������ɲ��
void motor_run(USART_TypeDef* UARTx,float speed,uint8_t direct,uint8_t id){
	
  float data=speed*360/2*3.14*8.6;
  uint16_t temp=data*100;
  parm_tx.count=3;
  parm_tx.data[0]=0x20;//�Ĵ�����ַ
  parm_tx.data[1]=(uint8_t)(temp&0xff);//�ٶ�ֵ��λ
  parm_tx.data[2]=(uint8_t)((temp&0xff00)>>8|(direct<<8));//�ٶ�ֵ��λ ���λΪ����λ
	
  if(id==RMotor){
    DirectR=direct;
    }
    else if(id==LMotor){
    DirectL=direct;
    }
    
    Inspack_Set(WRITE_DATA,parm_tx,id);
    Motor_send_insturction(UARTx,inst);

}

//�������ܣ�����ǰ�Ƕ�ֵ
//���Ҷ�Ҫ����ʱ������һ������֮����ʱһ��ʱ���ٽ��ж���һ��
void ReadDisData(USART_TypeDef* UARTx,uint8_t id){

  parm_tx.count=2;
  parm_tx.data[0]=0x28;//�Ĵ�����ַ����ǰ�ǶȼĴ�����ַ
  parm_tx.data[1]=0x02;//�������ֽ�

  Inspack_Set(READ_DATA,parm_tx,id);
  Motor_send_insturction(UARTx,inst);
  
}

//�������ܣ��õ���ǰ�Ƕ�ֵ
uint16_t Read_angle(void){
	
  uint16_t angle;

  angle=res.parm.data[0]+res.parm.data[1]<<8; 
	
  return angle;
  
}

//�������ܣ���ѯ�յ��ĽǶ�ֵ
//���������RS485ͨ�Ž��յ�������,���յ����ֽ��� 
//          *data:RS485_RX_BUF
// ���������16λ�޷���ֵ
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

//�������ܣ��������ֵ ��λ��mm
uint16_t Calculate_distance(uint16_t angle){

  uint16_t dis;
  dis=angle*2*3.14*86/360;
  
  return dis;
}
//�������ܣ������ȡ��ǰ���ֵ
//���������UARTx:usart2  
//��������������ǰ���
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

//�������ܣ������ҵ���߹������ֵ
//�����������  DirectR:1 Ϊ��ת  0Ϊ��ת
//����������Ƕ�ֵ
uint32_t CalculateDisR(void){

    uint32_t temp;
    
    if(DirectR){  //��ת
      if(now_dis_r=<last_dis_r)
        temp=last_dis_r-now_dis_r;       
      
    else 
        temp=65535-now_dis_r+last_dis_r;
    
    last_dis_r=now_dis_r;   //�洢�������
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
//�������ܣ����������߹������ֵ
//�����������
//������������ֵ
uint32_t CalculateDisL(void){

    uint32_t temp;
    if(DirectL){  //��ת
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
//�������ܣ���ѯ�ҵ���Ƿ��ת
//���������
//���������
char MotorLockR(void){
	
   return Flag_lock_R;
}
//�������ܣ���ѯ�����Ƿ��ת
//���������
//���������
char MotorLockL(void){
	
	return Flag_lock_L;
}
char MotorLock(void){
  
  return MotorLockR()|MotorLockL();

}


/*
 * �������Ƿ��ڵ��ϼ���ʼ������
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
 * �������Ƿ��ڵ��ϼ�⺯��
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


