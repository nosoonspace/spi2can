#include "spi.h"
#include "stdio.h"
void spi_init(void){

	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//中断时钟
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	//时钟使能
	RCC_AHB1ClockGatingCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	//printf("\r\n spi_in! \r\n");
	
	//spi硬件时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);							
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE);	
	
	//配置PA5-PA7复用功能模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//片选
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//引脚复用
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	

	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//PA4-PA7连接到SPI硬件
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource4,GPIO_AF_SPI1); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1); 

	SPI_I2S_DeInit(SPI1);
	//配置SPI参数
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave; //从机模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;//16位数据位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;       //mode_0
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;     //
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;        //片选引脚由NSS管脚控制
	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//硬件时钟APB2,90MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	
	//使能spi硬件工作
	SPI_Init(SPI1,&SPI_InitStructure);
	SPI_I2S_ITConfig(SPI1,SPI_I2S_IT_RXNE,ENABLE);       // 接收缓冲区非空时生成中断
	//SPI_I2S_ITConfig(SPI1,SPI_I2S_IT_TXE,ENABLE);       // 发送缓冲区为空时生成中断
	SPI_SSOutputCmd(SPI1, DISABLE);
	SPI_Cmd(SPI1,ENABLE);
	
	
	//配置中断
	
	
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_Init(&NVIC_InitStructure);
	
	
}
