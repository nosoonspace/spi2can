#include "main.h"
#include "math_ops.h"
#include "leg_Message.h"
#include "spi.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "can.h"


spi_data_t spi_data; // 发给UPbaord的状态数据
spi_command_t spi_command; // 从UPbaord接收的指令数据
uint8_t in1, in2, out;
static uint8_t enabled = 0; //电机使能状态
volatile uint8_t spi_data_ready = 0; // SPI数据接收完成标志
//uint8_t zero_flag = 0;//上电置零
// spi buffers
uint16_t rx_buff[RX_LEN];
uint16_t tx_buff[TX_LEN];

leg_state l1_state, l2_state;
leg_control l1_control, l2_control;

CANMessage a1_can, a2_can, h1_can, h2_can, k1_can, k2_can;

CANMessage rxMsg1,rxMsg2;
//int estop = 1;

//int softstop_joint(joint_state state, joint_control * control, float limit_p, float limit_n){
//    if((state.p)>=limit_p){
//        //control->p_des = limit_p;
//        control->v_des = 0.0f;
//        control->kp = 0;
//        control->kd = KD_SOFTSTOP;
//        control->t_ff += KP_SOFTSTOP*(limit_p - state.p);
//        return 1;
//    }
//    else if((state.p)<=limit_n){
//        //control->p_des = limit_n;
//        control->v_des = 0.0f;
//        control->kp = 0;
//        control->kd = KD_SOFTSTOP;
//        control->t_ff += KP_SOFTSTOP*(limit_n - state.p);
//        return 1;
//    }
//    return 0;
//    
// }

void pack_cmd(CANMessage * msg, joint_control joint){
	 
     /// limit data to be within bounds ///
     float p_des = fminf(fmaxf(P_MIN, joint.p_des), P_MAX);
     float v_des = fminf(fmaxf(V_MIN, joint.v_des), V_MAX);
     float kp = fminf(fmaxf(KP_MIN, joint.kp), KP_MAX);
     float kd = fminf(fmaxf(KD_MIN, joint.kd), KD_MAX);
     float t_ff = fminf(fmaxf(T_MIN, joint.t_ff), T_MAX);
     /// convert floats to unsigned ints ///
     uint16_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);            
     uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
     uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
     uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
     uint16_t t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
     /// pack ints into the can buffer ///
     msg->Data[0] = p_int>>8;
     msg->Data[1] = p_int&0xFF;
     msg->Data[2] = v_int>>4;
     msg->Data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
     msg->Data[4] = kp_int&0xFF;
     msg->Data[5] = kd_int>>4;
     msg->Data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
     msg->Data[7] = t_int&0xff;
}

// 解包电机驱动返回的信息
void unpack_reply(CANMessage msg, leg_state * leg){	
	
    /// unpack ints from can buffer ///
    uint16_t id = msg.Data[0];
    uint16_t p_int = (msg.Data[1]<<8)|msg.Data[2];
    uint16_t v_int = (msg.Data[3]<<4)|(msg.Data[4]>>4);
    uint16_t i_int = ((msg.Data[4]&0xF)<<8)|msg.Data[5];
    /// convert uints to floats ///
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);
    
    if(id==1){
        leg->a.p = p;
        leg->a.v = v;
        leg->a.t = t;
	}
    else if(id==2){
        leg->h.p = p;
        leg->h.v = v;
        leg->h.t = t;
	}
    else if(id==3){
        leg->k.p = p;
        leg->k.v = v;
        leg->k.t = t;
	}
}

void PackAll(void){
    pack_cmd(&a1_can, l1_control.a); 
    pack_cmd(&a2_can, l2_control.a); 
    pack_cmd(&h1_can, l1_control.h); 
    pack_cmd(&h2_can, l2_control.h); 
    pack_cmd(&k1_can, l1_control.k); 
    pack_cmd(&k2_can, l2_control.k); 
    
}

void WriteAll(void){
	out = CAN_Send_Msg(a2_can.Data,a2_can.ID,CAN2);
    delay_us(50);
	out = CAN_Send_Msg(a1_can.Data,a1_can.ID,CAN1);
    delay_us(50);
	out = CAN_Send_Msg(h2_can.Data,h2_can.ID,CAN2);
    delay_us(50);
	out = CAN_Send_Msg(h1_can.Data,h1_can.ID,CAN1);				
	delay_us(50);
	out = CAN_Send_Msg(k2_can.Data,k2_can.ID,CAN2);
    delay_us(50);
	out = CAN_Send_Msg(k1_can.Data,k1_can.ID,CAN1);
    delay_us(50);

}

void zero_commond(void){
	spi_command.q_des_abad[0] = 0;	
	spi_command.q_des_hip[0] = 0;
	spi_command.q_des_knee[0] = 0;
	spi_command.qd_des_abad[0] = 0;
	spi_command.qd_des_hip[0] = 0;
	spi_command.qd_des_knee[0] = 0;
	spi_command.kp_abad[0] = 0;
	spi_command.kp_hip[0] = 0;
	spi_command.kp_knee[0] = 0;
	spi_command.kd_abad[0] = 0;
	spi_command.kd_hip[0] = 0;
	spi_command.kd_knee[0] = 0;
	spi_command.tau_abad_ff[0] = 0;
	spi_command.tau_hip_ff[0] = 0;
	spi_command.tau_knee_ff[0] = 0;
	spi_command.flags[0] = 0;

	
	spi_command.q_des_abad[1] = 0;
	spi_command.q_des_hip[1] = 0;
	spi_command.q_des_knee[1] = 0;
	spi_command.qd_des_abad[1] = 0;
	spi_command.qd_des_hip[1] = 0;
	spi_command.qd_des_knee[1] = 0;
	spi_command.kp_abad[1] = 0;
	spi_command.kp_hip[1] = 0;
	spi_command.kp_knee[1] = 0;
	spi_command.kd_abad[1] = 0;
	spi_command.kd_hip[1] = 0;
	spi_command.kd_knee[1] = 0;
	spi_command.tau_abad_ff[1] = 0;
	spi_command.tau_hip_ff[1] = 0;
	spi_command.tau_knee_ff[1] = 0;
	spi_command.flags[1] = 0;
}

void zero(void){
	for(int i =0; i<TX_LEN ;i++){
		tx_buff[i] = 0;
	}
	spi_data.q_abad[0] = 0;
	spi_data.q_hip[0] = 0;
	spi_data.q_knee[0] = 0;
	spi_data.qd_abad[0] = 0;
	spi_data.qd_hip[0] = 0;
	spi_data.qd_knee[0] = 0;
	spi_data.t_abad[0] = 0;
	spi_data.t_hip[0] = 0;
	spi_data.t_knee[0] = 0;
	spi_data.flags[0] = 0;

	spi_data.q_abad[1] = 0;
	spi_data.q_hip[1] = 0;
	spi_data.q_knee[1] = 0;
	spi_data.qd_abad[1] = 0;
	spi_data.qd_hip[1] = 0;
	spi_data.qd_knee[1] = 0;
	spi_data.t_abad[1] = 0;
	spi_data.t_hip[1] = 0;
	spi_data.flags[1] = 0;

	spi_command.q_des_abad[0] = 0;	
	spi_command.q_des_hip[0] = 0;
	spi_command.q_des_knee[0] = 0;
	spi_command.qd_des_abad[0] = 0;
	spi_command.qd_des_hip[0] = 0;
	spi_command.qd_des_knee[0] = 0;
	spi_command.kp_abad[0] = 0;
	spi_command.kp_hip[0] = 0;
	spi_command.kp_knee[0] = 0;
	spi_command.kd_abad[0] = 0;
	spi_command.kd_hip[0] = 0;
	spi_command.kd_knee[0] = 0;
	spi_command.tau_abad_ff[0] = 0;
	spi_command.tau_hip_ff[0] = 0;
	spi_command.tau_knee_ff[0] = 0;
	spi_command.flags[0] = 0;

	
	spi_command.q_des_abad[1] = 0;
	spi_command.q_des_hip[1] = 0;
	spi_command.q_des_knee[1] = 0;
	spi_command.qd_des_abad[1] = 0;
	spi_command.qd_des_hip[1] = 0;
	spi_command.qd_des_knee[1] = 0;
	spi_command.kp_abad[1] = 0;
	spi_command.kp_hip[1] = 0;
	spi_command.kp_knee[1] = 0;
	spi_command.kd_abad[1] = 0;
	spi_command.kd_hip[1] = 0;
	spi_command.kd_knee[1] = 0;
	spi_command.tau_abad_ff[1] = 0;
	spi_command.tau_hip_ff[1] = 0;
	spi_command.tau_knee_ff[1] = 0;
	spi_command.flags[1] = 0;
	
}
void zero_control(void){
	l1_control.a.p_des = 0;
	l1_control.a.v_des = 0;
	l1_control.a.kp    = 0;
	l1_control.a.kd    = 0;
	l1_control.a.t_ff  = 0;
	l1_control.h.p_des = 0;
	l1_control.h.v_des = 0;
	l1_control.h.kp    = 0;
	l1_control.h.kd    = 0;
	l1_control.h.t_ff  = 0;
	l1_control.k.p_des = 0;
	l1_control.k.v_des = 0;
	l1_control.k.kp    = 0;
	l1_control.k.kd    = 0;
	l1_control.k.t_ff  = 0;
	
	l2_control.a.p_des = 0;
	l2_control.a.v_des = 0;
	l2_control.a.kp    = 0;
	l2_control.a.kd    = 0;
	l2_control.a.t_ff  = 0;
	l2_control.h.p_des = 0;
	l2_control.h.v_des = 0;
	l2_control.h.kp    = 0;
	l2_control.h.kd    = 0;
	l2_control.h.t_ff  = 0;
	l2_control.k.p_des = 0;
	l2_control.k.v_des = 0;
	l2_control.k.kp    = 0;
	l2_control.k.kd    = 0;
	l2_control.k.t_ff  = 0;
	
}
void EnterMotorMode(CANMessage * msg){
    msg->Data[0] = 0xFF;
    msg->Data[1] = 0xFF;
    msg->Data[2] = 0xFF;
    msg->Data[3] = 0xFF;
    msg->Data[4] = 0xFF;
    msg->Data[5] = 0xFF;
    msg->Data[6] = 0xFF;
    msg->Data[7] = 0xFC;
    //WriteAll();
    }
void Zero(CANMessage * msg){
    msg->Data[0] = 0xFF;
    msg->Data[1] = 0xFF;
    msg->Data[2] = 0xFF;
    msg->Data[3] = 0xFF;
    msg->Data[4] = 0xFF;
    msg->Data[5] = 0xFF;
    msg->Data[6] = 0xFF;
    msg->Data[7] = 0xFE;
    //WriteAll();
    }
void ExitMotorMode(CANMessage * msg){
    msg->Data[0] = 0xFF;
    msg->Data[1] = 0xFF;
    msg->Data[2] = 0xFF;
    msg->Data[3] = 0xFF;
    msg->Data[4] = 0xFF;
    msg->Data[5] = 0xFF;
    msg->Data[6] = 0xFF;
    msg->Data[7] = 0xFD;
    //WriteAll();
    }

//校验spi数据
uint32_t xor_checksum(uint32_t* data, int len)
{
    uint32_t t = 0;
    for(int i = 0; i < len; i++)   
        t = t ^ data[i];
    return t;
}


// *************control*************
void control(void)
{	
	
	if(((spi_command.flags[0]&0x1)==1)  && (enabled==0))
	{
		enabled = 1;
		EnterMotorMode(&a1_can);//上电
		
		EnterMotorMode(&a2_can);
		
		EnterMotorMode(&h1_can);
		
		EnterMotorMode(&h2_can);
		
		EnterMotorMode(&k1_can);
		
		EnterMotorMode(&k2_can);
			
		WriteAll();
		return;
	}
	else if(((spi_command.flags[0]&0x1)==0)  && (enabled==1))
	{
		enabled = 0;
		ExitMotorMode(&a1_can);
		
		ExitMotorMode(&a2_can);
			
		ExitMotorMode(&h1_can);
		
		ExitMotorMode(&h2_can);
		
		ExitMotorMode(&k1_can);
		
		ExitMotorMode(&k2_can);
		
		WriteAll();
		
		return;
	}


	spi_data.q_abad[0]  = l1_state.a.p;
	spi_data.q_hip[0]   = l1_state.h.p;
	spi_data.q_knee[0]  = l1_state.k.p;
	spi_data.qd_abad[0] = l1_state.a.v;
	spi_data.qd_hip[0]  = l1_state.h.v;
	spi_data.qd_knee[0] = l1_state.k.v;
	spi_data.t_abad[0]  = l1_state.a.t;
	spi_data.t_hip[0]   = l1_state.h.t;
	spi_data.t_knee[0]  = l1_state.k.t;
	
    
	spi_data.q_abad[1]  = l2_state.a.p;
	spi_data.q_hip[1]   = l2_state.h.p;
	spi_data.q_knee[1]  = l2_state.k.p;
	spi_data.qd_abad[1] = l2_state.a.v;
	spi_data.qd_hip[1]  = l2_state.h.v;
	spi_data.qd_knee[1] = l2_state.k.v;
	spi_data.t_abad[1]  = l2_state.a.t;
	spi_data.t_hip[1]   = l2_state.h.t;
	spi_data.t_knee[1]  = l2_state.k.t;
	
//	if(estop==0){
//		zero_control();
//		spi_data.flags[0] = 0xdead;
//		spi_data.flags[1] = 0xdead;
//	}
//    
//	else{     
		zero_control();
		l1_control.a.p_des  = spi_command.q_des_abad[0];//左三和右一对换
		l1_control.a.v_des  = spi_command.qd_des_abad[0];
		l1_control.a.kp     = spi_command.kp_abad[0];
		l1_control.a.kd     = spi_command.kd_abad[0];
		l1_control.a.t_ff   = spi_command.tau_abad_ff[0];
		
		l1_control.h.p_des  = spi_command.q_des_hip[0];
		l1_control.h.v_des  = spi_command.qd_des_hip[0];
		l1_control.h.kp     = spi_command.kp_hip[0];
		l1_control.h.kd     = spi_command.kd_hip[0];
		l1_control.h.t_ff   = spi_command.tau_hip_ff[0];
		
		l1_control.k.p_des  = spi_command.q_des_knee[0];
		l1_control.k.v_des  = spi_command.qd_des_knee[0];
		l1_control.k.kp     = spi_command.kp_knee[0];
		l1_control.k.kd     = spi_command.kd_knee[0];
		l1_control.k.t_ff   = spi_command.tau_knee_ff[0];
		
		l2_control.a.p_des  = spi_command.q_des_abad[1];
		l2_control.a.v_des  = spi_command.qd_des_abad[1];
		l2_control.a.kp     = spi_command.kp_abad[1];
		l2_control.a.kd     = spi_command.kd_abad[1];
		l2_control.a.t_ff   = spi_command.tau_abad_ff[1];
		
		l2_control.h.p_des  = spi_command.q_des_hip[1];
		l2_control.h.v_des  = spi_command.qd_des_hip[1];
		l2_control.h.kp     = spi_command.kp_hip[1];
		l2_control.h.kd     = spi_command.kd_hip[1];
		l2_control.h.t_ff   = spi_command.tau_hip_ff[1];
		
		l2_control.k.p_des  = spi_command.q_des_knee[1];
		l2_control.k.v_des  = spi_command.qd_des_knee[1];
		l2_control.k.kp     = spi_command.kp_knee[1];
		l2_control.k.kd     = spi_command.kd_knee[1];
		l2_control.k.t_ff   = spi_command.tau_knee_ff[1];

		spi_data.flags[0] = 1;
		spi_data.flags[1] = 1;
//		spi_data.flags[0] |= softstop_joint(l1_state.a, &l1_control.a, A_LIM_P, A_LIM_N);
//		spi_data.flags[0] |= (softstop_joint(l1_state.h, &l1_control.h, H_LIM_P, H_LIM_N))<<1;
//		spi_data.flags[1] |= softstop_joint(l2_state.a, &l2_control.a, A_LIM_P, A_LIM_N);
//		spi_data.flags[1] |= (softstop_joint(l2_state.h, &l2_control.h, H_LIM_P, H_LIM_N))<<1;	 
	//}
	spi_data.checksum = xor_checksum((uint32_t*)&spi_data,20);
	for(int i = 0; i < DATA_LEN; i++){
			tx_buff[i] = ((uint16_t*)(&spi_data))[i];
	}
}

void SPI1_IRQHandler(void)
{
	int count = 0;
	volatile uint32_t timeout;
	const uint32_t TIMEOUT_LIMIT = 10000;

	SPI_SendData(SPI1,tx_buff[0]) ;
	
	// 添加超时保护的主循环
	timeout = 0;
	while(CS == 0 && timeout < TIMEOUT_LIMIT)
	{
		timeout++;
		
		if(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY) == SET)
		{
			// 等待接收数据 - 添加超时
			uint32_t rx_timeout = 0;
			while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET && rx_timeout < TIMEOUT_LIMIT)
			{
				rx_timeout++;
			}
			if(rx_timeout >= TIMEOUT_LIMIT) break; // 超时退出
			
			rx_buff[count] = SPI_ReceiveData(SPI1); //接收数据
			count++;
			
			if(count < RX_LEN)
			{
				// 等待发送就绪 - 添加超时
				uint32_t tx_timeout = 0;
				while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET && tx_timeout < TIMEOUT_LIMIT)
				{
					tx_timeout++;
				}
				if(tx_timeout >= TIMEOUT_LIMIT) break; // 超时退出
				
				SPI_SendData(SPI1,tx_buff[count]) ;//发送数据	
			}
			else
			{
				break; // 数据接收完成，退出循环
			}
		}			
	}
	
	// 检查数据完整性
	if(count < RX_LEN)
	{
		spi_data.flags[1] = 0xBAD0; // 数据不完整
		return;
	}
	
    // 数据接收完成，解析并校验
	zero_commond();
    uint32_t calc_checksum = xor_checksum((uint32_t*)rx_buff,32);
    for(count = 0; count < CMD_LEN; count++)
    {
        ((uint16_t*)(&spi_command))[count] = rx_buff[count];
    }		
		
    // 校验数据
    if(calc_checksum != spi_command.checksum)
	{
		spi_data.flags[1] = 0xdead; // 校验失败
		return; // 校验失败不处理
	}
	else
	{
		spi_data.flags[1] = 0x600D; // 通信正常
	}
	
	// 设置数据就绪标志，让主循环处理耗时操作
	spi_data_ready = 1;
}

void printf_data(void){
	
	//printf("\r\n send:%d,%d,%d,%d,%d,%d,%d,%d \r\n",k1_can.Data[0],k1_can.Data[1],k1_can.Data[2],k1_can.Data[3],k1_can.Data[4],k1_can.Data[5],k1_can.Data[6],k1_can.Data[7]);
	//printf("\r\n\r\n");//插入换行
	printf("\r\n receive:%f,%f,%f\r\n",l1_state.k.p,l1_state.k.v,l1_state.k.t);	
	uint16_t count =0;
	for(count = 0;count<TX_LEN;count++){
		printf("%d:%d   ",count,tx_buff[count]);
	
	}	
	//printf("\r\n\r\n");//插入换行
	//printf("\r\n %f,%f,%f \r\n",l1_state.k.p,l1_state.k.v,l1_state.k.t);
	
	//printf("\r\n  %d \r\n",SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE));//插入换行
}

int main(void)
{ 
	delay_init(180);		//延时初始化 
	uart_init(115200);	//串口初始化波特率为115200
	led_init();//初始化与LED连接的硬件接口	
	
	CAN1_Mode_Init();//初始化CAN1，CAN2
	CAN2_Mode_Init();	
	
	spi_init();//spi初始化
	
	zero();
	a1_can.len = 8;                         //transmit 8 bytes
	a2_can.len = 8;                         //transmit 8 bytes
	h1_can.len = 8;
	h2_can.len = 8;
	k1_can.len = 8;
	k2_can.len = 8;
	rxMsg1.len = 8;
	rxMsg2.len = 8;

	a1_can.ID = 0x01;                        
	a2_can.ID = 0x01;                 
	h1_can.ID = 0x02;
	h2_can.ID = 0x02;
	k1_can.ID = 0x03;
	k2_can.ID = 0x03;
	
	pack_cmd(&a1_can, l1_control.a); 
	pack_cmd(&a2_can, l2_control.a); 
	pack_cmd(&h1_can, l1_control.h); 
	pack_cmd(&h2_can, l2_control.h); 
	pack_cmd(&k1_can, l1_control.k); 
	pack_cmd(&k2_can, l2_control.k); 
	
	WriteAll();
	delay_ms(2000);
	
	uint32_t count = 0;
	//zero_flag = 0;
	while(1)
	{
		count++;	
		in1 = CAN1_Receive_Msg(rxMsg1.Data);
		if (in1 != 0)
		{
			// printf("Receice CAN message!"); // 如果能收到CAN，就在串口打印
			unpack_reply(rxMsg1, &l1_state);
		}
			
		in2 = CAN2_Receive_Msg(rxMsg2.Data);
		if (in2 != 0)
			unpack_reply(rxMsg2, &l2_state);
		
		// 检查SPI数据是否就绪，执行控制逻辑（从中断移到主循环）
		if(spi_data_ready)
		{
			spi_data_ready = 0; // 清除标志
			control();   // 执行控制逻辑
			PackAll();   // 打包CAN消息
			WriteAll();  // 发送CAN消息
		}
		
		delay_us(200);



		// 串口打印和亮灯
		if(count == 2000){ // 大约0.4s
			// // 打印UP->SPIne的指令信息
			// printf("UP->SPIne: %f %f %f %f %f %hx \r\n",
			// spi_command.q_des_abad[0],
			// spi_command.qd_des_abad[0],
			// spi_command.kp_abad[0],
			// spi_command.kd_abad[0],
			// spi_command.tau_abad_ff[0],
			// spi_command.flags[0]
			// );

			// // 打印电机返回状态信息
			//  printf("motor->SPIne: %hx %hx %hx %hx %hx %hx %hx %hx %hx %hx\r\n",
			// 	rxMsg1.len,
			// 	rxMsg1.ID,
			// 	rxMsg1.Data[0],
			// 	rxMsg1.Data[1],
			// 	rxMsg1.Data[2],
			// 	rxMsg1.Data[3],
			// 	rxMsg1.Data[4],
			// 	rxMsg1.Data[5],
			// 	rxMsg1.Data[6],
			// 	rxMsg1.Data[7]
			// );

			LED1 = !LED1;
			count = 0;		
		}
//		LED1 = 0;
	}
}
