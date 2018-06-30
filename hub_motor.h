#ifndef _HUB_MOTOR_H
#define _HUB_MOTOR_H
#include "rs485.h"
#include "stdint.h"

//可用指令
//0xff 为自定义指令
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

//应答包ERROR状态
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

//落地检测相关
#define ONGROUND_GPIO       GPIOD
#define ONGROUND_LEFT       GPIO_Pin_10
#define ONGROUND_RIGHT      GPIO_Pin_1

//参数结构体
struct _parmeter{
    uint8_t count;//参数字节数
	uint8_t data[8];//所带参数数据，假设最多带8个字节数据
};

typedef struct _parmeter par;
typedef struct _instruction instruction;
typedef struct _response response;
//指令包结构体
struct _instruction{
	//帧头
	uint8_t head1;
	uint8_t head2;
	
	//ID号
	uint8_t ID;
	
	//数据长度
	uint8_t length;
	
	//指令字节
	uint8_t Instruction;
	
	//参数
	par parm;

    //校验和
	uint8_t  checksum;
};

//应答包结构体
struct _response{
	//帧头
	uint8_t head1;
	uint8_t head2;
	
	//ID号
	uint8_t ID;
	
	//数据长度
	uint8_t length;
	
	//当前状态
	uint8_t error;
	
	//参数
	par parm;
	
	//校验和
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

