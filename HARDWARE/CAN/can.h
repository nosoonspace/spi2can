#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"
#include "main.h"	    					    
										 							 				    
void CAN1_Mode_Init(void);//CAN初始化
void CAN2_Mode_Init(void);

uint8_t CAN_Send_Msg(uint8_t* msg,uint8_t ID,CAN_TypeDef* CAN_num);		//发送数据

uint8_t CAN1_Receive_Msg(uint8_t *buf);							//接收数据
uint8_t CAN2_Receive_Msg(uint8_t *buf);

typedef struct{
	uint8_t len;
	uint8_t ID;
	uint8_t Data[8];
}CANMessage;

#endif 








