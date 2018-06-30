#ifndef _HUB_MOTOR_H
#define _HUB_MOTOR_H
#include "rs485.h"
#include "stdint.h"

//����ָ��
//0xff Ϊ�Զ���ָ��
#define PING 0X01
#define READ_DATA (0X02)
#define WRITE_DATA (0X03)
#define REG_WRITE 0X04
#define ACTION 0X05
#define RESET_CON 0X06
#define START_END 0X07
#define SYNC_WRITE 0X83
#define TRAJ_WRITE 0X0A
#define TRAJ_ACTION 0X0B

//Ӧ���ERROR״̬
#define STOP 0X40
#define ERROR_INS 0X20
#define ERROR_CHECKSUM 0X10
#define LACK_VOLTAGE 0X08
#define OVER_VOLTAGE 0X04
#define OVER_CURRENT 0X02
#define OVER_HOT 0X01

#define RIGHT_MOTOR USART2

#define ForwardR 0
#define BackwardR 1

#define ForwardL 1
#define BankwardL 0

#define RMotor 0x0a
#define LMotor 0x0b

//��ؼ�����
#define ONGROUND_GPIO       GPIOD
#define ONGROUND_LEFT       GPIO_Pin_10
#define ONGROUND_RIGHT      GPIO_Pin_1

//�����ṹ��
struct _parmeter{
    uint8_t count;//�����ֽ���
	uint8_t data[8];//�����������ݣ���������8���ֽ�����
};

typedef struct _parmeter par;
typedef struct _instruction instruction;
typedef struct _response response;
//ָ����ṹ��
struct _instruction{
	//֡ͷ
	uint8_t head1;
	uint8_t head2;
	
	//ID��
	uint8_t ID;
	
	//���ݳ���
	uint8_t length;
	
	//ָ���ֽ�
	uint8_t Instruction;
	
	//����
	par parm;

    //У���
	uint8_t  checksum;
};

//Ӧ����ṹ��
struct _response{
	//֡ͷ
	uint8_t head1;
	uint8_t head2;
	
	//ID��
	uint8_t ID;
	
	//���ݳ���
	uint8_t length;
	
	//��ǰ״̬
	uint8_t error;
	
	//����
	par parm;
	
	//У���
	uint8_t  checksum;

};

void  Hub_motor_Init(void);
static uint8_t checksum_deal(instruction inst);
static void Inspack_Set(uint8_t ins,par par,uint8_t id);
static void Motor_send_insturction(USART_TypeDef* UARTx,instruction ins);
void Motor_response(uint8_t *data,uint8_t rx_cnt);
void motor_run(USART_TypeDef* UARTx,uint16_t speed,uint8_t direct,uint8_t id);
void Reset_MOTOR(USART_TypeDef* UARTx);
void recive_res(void);
uint16_t Recive_data(uint8_t cnt,uint8_t *buf);
uint16_t Read_angle(void);
uint16_t Calculate_distance(uint16_t angle);
void ReadDisData(USART_TypeDef* UARTx,uint8_t id);
void NowDis(USART_TypeDef* UARTx,uint8_t id);
uint16_t CalculateDisR(void);
uint16_t CalculateDisL(void);
char MotorLockR(void);
char MotorLockL(void);
char MotorLock(void);

void init_ontheground_detect(void);
uint8_t is_ontheground(void);
#endif

