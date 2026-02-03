#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "sys.h"

void CAN1_Mode_Init(void)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
		CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    //使能相关时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTB时钟	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA8,PA9
	
		//引脚复用映射配置
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOA11复用为CAN1
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
		CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=ENABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=ENABLE; //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=DISABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal; //CAN_Mode_LoopBack	; //模式设置 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=CAN_BS1_8tq; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq 6
  	CAN_InitStructure.CAN_Prescaler=3;  //分频系数(Fdiv)为brp+1	3
	
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
    
		//配置过滤器
		CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit; //16位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x00;////16位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x00;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x00;//16位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x00;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
	
}    
void CAN2_Mode_Init(void)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
		CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    //使能相关时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTB时钟	                   											 
	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN2时钟
	
    //初始化GPIO
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA12,PA13
	
		//引脚复用映射配置
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOA11复用为CAN2
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOA12复用为CAN2
	  
  	//CAN单元设置
		CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=ENABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=ENABLE; //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=DISABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal; //	; //模式设置 
		//CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=CAN_BS1_8tq; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=3;  //分频系数(Fdiv)为brp+1	
	
		CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN2 
    
		//配置过滤器	
		CAN_FilterInitStructure.CAN_FilterNumber=14;	  //过滤器1
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit; //16位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x00;////16位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x00;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x00;//16位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x00;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO1;//过滤器1关联到FIFO1
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器1
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
} 

//can发送一组数据(固定格式:ID,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
uint8_t CAN_Send_Msg(uint8_t* msg, uint8_t ID, CAN_TypeDef* CAN_num)
{	
  uint8_t mbox;
  uint8_t i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=ID;	 // 标准标识符
  TxMessage.ExtId=0x00;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用标准标识符
  TxMessage.RTR=0;		  //数据帧
  TxMessage.DLC=8;       // 消息类型为数据帧，一帧8位
  for(i=0;i<8;i++)  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN_num, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN_num, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束	
  if(i>=0XFFF)return 1;
  return 0;
}

//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
uint8_t CAN1_Receive_Msg(uint8_t *buf)
{		   		   
 	uint8_t i;
	CanRxMsg RxMessage;
	
	if(CAN_MessagePending(CAN1,CAN_FIFO0)==0) return 0;		//没有接收到数据,直接退出 
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
	for(i=0;i<RxMessage.DLC;i++)
		buf[i]=RxMessage.Data[i];
	return RxMessage.DLC;  	
}

uint8_t CAN2_Receive_Msg(uint8_t *buf)
{		   		   
 	uint8_t i;
	CanRxMsg RxMessage;
    if(CAN_MessagePending(CAN2,CAN_FIFO1)==0) return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN2, CAN_FIFO1, &RxMessage);//读取数据	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];
	return RxMessage.DLC;  	
}














