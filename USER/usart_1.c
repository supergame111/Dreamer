/*
** 蓝牙模块连接这个串口
*/
#include "global.h"

//匿名上位机通信协议
//数据分段发送
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
#define BYTE4(dwTemp)       (*((char *)(&dwTemp) + 4))
#define BYTE5(dwTemp)       (*((char *)(&dwTemp) + 5))
#define BYTE6(dwTemp)       (*((char *)(&dwTemp) + 6))
#define BYTE7(dwTemp)       (*((char *)(&dwTemp) + 7))

extern MPU6050_DATA mpu_data;  //存放获取数据
extern EULER euler_angle;

//DS1302中的变量，在串口中断函数中修改时间的时候重置
//注意格式：秒 分 时 日 月 周 年
u8 init_time[8];			 
unsigned char temp[8];
volatile unsigned char adjust_real_time_flag = 0;

unsigned char data_to_send[50] = {0};  //发送数据打包

unsigned char Tx_Buffer[256] ;
unsigned char Tx_Counter = 0;
unsigned char Tx_Counter_Temp = 0;
unsigned char Rx_Buffer[50] ;		//接受缓冲
unsigned char Rx_Counter = 0;          //接受计数变量

extern unsigned char CmdRx_Buffer[10];	   //符合格式的接收字符缓冲


static void Send_Data(uint8_t *dataToSend , uint8_t length);
static void Uart1_Put_Char(unsigned char DataToSend);
static void Uart1_Put_Buf(unsigned char *DataToSend ,unsigned char data_num);


/*----------------------------------
**函数名称：Usart_GPIO_Init
**功能描述：串口使用引脚初始化
**参数说明：无
**作者：Andrew
**日期：2018.1.24
-----------------------------------*/
void Usart1_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 				//TX
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	   	//复用推挽输出
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //配置端口速度为50M
  	GPIO_Init(GPIOA, &GPIO_InitStructure);				   	//根据参数初始化GPIOA寄存器	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 				//RX
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//浮空输入(复位状态);
  	GPIO_Init(GPIOA, &GPIO_InitStructure);				   	//根据参数初始化GPIOA寄存器	
}


/*----------------------------------
**函数名称：Usart_Configuration
**功能描述：串口参数配置，包括中断向量的配置
**参数说明：BaudRate：需要配置的波特率
**作者：Andrew
**日期：2018.1.24
-----------------------------------*/
void Usart1_Configuration(uint32_t BaudRate)
{
	USART_InitTypeDef USART_InitStructure;							    	//定义一个串口结构体
	NVIC_InitTypeDef  NVIC_InitStructure;									//中断向量结构体
	
	USART_DeInit(USART1); 														//复位串口
	USART_InitStructure.USART_BaudRate            =BaudRate ;	  			//波特率自己设置，蓝牙设置了9600
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b; 	//传输过程中使用8位数据
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;	 	//在帧结尾传输1位停止位
	USART_InitStructure.USART_Parity              = USART_Parity_No ;	 	//奇偶失能
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流失能
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx; //接收和发送模式
	USART_Init(USART1, &USART_InitStructure);								//根据参数初始化串口寄存器
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);							//使能串口中断接收
	USART_Cmd(USART1, ENABLE);     											//使能串口外设
	
	//如下语句解决第1个字节无法正确发送出去的问题 
	USART_ClearFlag(USART1, USART_FLAG_TC);     /* 清发送外城标志，Transmission Complete flag */
	
	
	//配置串口中断向量
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //中断分组1
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//设定中断源为
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//中断占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;			//副优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断
	NVIC_Init(&NVIC_InitStructure);							   	//根据参数初始化中断寄存器
	
}


/*----------------------------------
**函数名称：USART1_IRQHandler
**功能描述：串口中断服务函数
**参数说明：无
**作者：Andrew
**日期：2018.1.24
-----------------------------------*/
void USART1_IRQHandler()
{
	unsigned char i = 0;
	unsigned char j = 0;
	
	/*异常中断处理*/
	if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)	//
    {
       USART_ReceiveData(USART1);										//丢弃数据
		 USART_ClearFlag(USART1, USART_FLAG_ORE);
    }
	 
	 /*发送*/
	if(USART_GetITStatus(USART1,USART_IT_TXE) != RESET)
	{
		USART1->DR = Tx_Buffer[Tx_Counter_Temp++];    	//向DR寄存器写数据的时候，串口就会自动发送
			
		if(Tx_Counter == Tx_Counter_Temp)					//数据包发送完成
		{
		   USART_ITConfig(USART1,USART_IT_TXE,DISABLE);	//关闭TXE中断
			Tx_Counter = 0;		//发送完成计数清零
			Tx_Counter_Temp = 0;
		}
	}
	  
	 /*接受，在这里接收蓝牙发送过来的校准时间*/
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)	//读取接收中断标志位USART_IT_RXNE 
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);	//清楚中断标志位
		
		Rx_Buffer[Rx_Counter++] = USART_ReceiveData(USART1);//接收数据到缓冲区，这个函数会自动清除中断标志位
		
		if((Rx_Buffer[Rx_Counter-2] == 0x0d) && (Rx_Buffer[Rx_Counter-1] == 0x0a))//检测最后的回车换行才算命令
		{
			memcpy(CmdRx_Buffer,Rx_Buffer,Rx_Counter);	//直接使用内存拷贝，效率比较高
			
//			for(i = 0;i<Rx_Counter;i++)
//			{
//				CmdRx_Buffer[i] = Rx_Buffer[i];  //拷贝到命令缓冲区
//			}
			
			CmdRx_Buffer[Rx_Counter] = '\0';		//最后添加'\0',构成完整数组
			Rx_Counter = 0;							//计数清零
			printf("CMD OK!");						//接收成功，发送标志
		}
		
		//检测到这个固定长度固定格式的数据包，就认为是校准的时间，要求手机发送固定格式的时间数据包:秒 分 时 日 月 周 年
		//例如：{12.12.12.05.01.07.18.T}
		else if(Rx_Counter == 22 && Rx_Buffer[Rx_Counter-1] == 'T')
		{
			for(i=0; i<Rx_Counter-1; i+=3)
			{
				//数字的ascii码，十进制0为48，乘16是因为他会以16进制存储。
				temp[j] = (Rx_Buffer[i] - 48)*16 + (Rx_Buffer[i+1] - 48) ;		
				j++;
			}
			
			memcpy(init_time,temp,j);
			init_time[j] = '\0';			
			Rx_Counter = 0;
			adjust_real_time_flag = 1;		//置标志，在主循环里面检测，决定是否校准时间
			
			printf("Time have adjusted.");
		}
	}
}


/*----------------------------------
**函数名称：fputc
**功能描述：printf重定向函数
**参数说明：无
**作者：Andrew
**日期：2018.1.24
-----------------------------------*/
//重定义fputc函数，方便使用printf
int fputc(int Data, FILE *f)
{   
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));	 	//USART_GetFlagStatus：得到发送状态位
																				//USART_FLAG_TXE:发送寄存器为空 1：为空；0：忙状态
	USART_SendData(USART1,Data);						  				//发送一个字符	
	return Data;										  					//返回发送的值
}


/*中断发送一个字符*/
static void Uart1_Put_Char(unsigned char DataToSend)
{
	Tx_Buffer[Tx_Counter++] = DataToSend;  
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
}

/*中断发送方式缓冲区字符*/
static void Uart1_Put_Buf(unsigned char *DataToSend ,unsigned char data_num)
{
	uint8_t i;
	
	for(i = 0;i < data_num;i++)	//要发送的数据放到发送缓冲区
		Tx_Buffer[Tx_Counter++] = *(DataToSend+i);

	if(!(USART1->CR1 & USART_CR1_TXEIE))					//中断方式发送
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 	//只要发送寄存器空，就会一直有中断
}

/*----------------------------------
**函数名称：USART1_Send_Byte
**功能描述：使用串口发送一个字节的数据
**参数说明：Data：需要发送的数据
**作者：Andrew
**日期：2018.1.24
-----------------------------------*/
void USART1_Send_Byte(uint16_t Data)
{ 
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));	  //USART_GetFlagStatus：得到发送状态位
														  //USART_FLAG_TXE:发送寄存器为空 1：为空；0：忙状态
	USART_SendData(USART1,Data);						  //发送一个字符
}

/*----------------------------------
**函数名称：USART1_Receive_Byte
**功能描述：串口接收一个字节的数据，返回值为接收的字节数据
**参数说明：无
**作者：Andrew
**日期：2018.1.24
-----------------------------------*/
uint8_t USART1_Receive_Byte(void)
{
  while(!(USART_GetFlagStatus(USART1,USART_FLAG_RXNE))); //USART_GetFlagStatus：得到接收状态位
														   //USART_FLAG_RXNE:接收数据寄存器非空标志位 
														   //1：忙状态  0：空闲(没收到数据，等待。。。)
	return USART_ReceiveData(USART1);					   //接收一个字符
}

/*------------------
**函数名称：Send_Data
**功能描述：发送一组数据
**参数说明：*dataToSend：要发送数组的指针
            length：数组长度
**作者：Andrew
**日期：2018.1.24
-----------------*/
static void Send_Data(uint8_t *dataToSend , uint8_t length)
{
	Uart1_Put_Buf(dataToSend,length);
}


/*----------------------------------
**函数名称：Send_Senser
**功能描述：发送打包的6050传感器数据
**参数说明：无
**作者：Andrew
**日期：2018.1.24
-----------------------------------*/
void Send_Senser(void )
{
	u8 cnt = 0;
	uint16_t sum = 0;
	u8 i = 0;
	int16_t temp_angle;
		
	data_to_send[cnt++]=0xAA;	 //帧头：AAAA
	data_to_send[cnt++]=0xAA;
	data_to_send[cnt++]=0x02;	 //功能字：OXF2	与0x02一样？
	data_to_send[cnt++]=0;	     //需要发送数据的字节数，暂时给0，后面在赋值。

	data_to_send[cnt++] = BYTE1(mpu_data.acc_data.x);//取data[0]数据的高字节，
	data_to_send[cnt++] = BYTE0(mpu_data.acc_data.x);
	data_to_send[cnt++] = BYTE1(mpu_data.acc_data.y);
	data_to_send[cnt++] = BYTE0(mpu_data.acc_data.y);
	data_to_send[cnt++] = BYTE1(mpu_data.acc_data.z);
	data_to_send[cnt++] = BYTE0(mpu_data.acc_data.z);

	data_to_send[cnt++] = BYTE1(mpu_data.gyro_data.x);//这里陀螺仪的数据不需要滤波，本来就很稳定。滤波之后反而更乱
	data_to_send[cnt++] = BYTE0(mpu_data.gyro_data.x);
	data_to_send[cnt++] = BYTE1(mpu_data.gyro_data.y);
	data_to_send[cnt++] = BYTE0(mpu_data.gyro_data.y);
	data_to_send[cnt++] = BYTE1(mpu_data.gyro_data.z);
	data_to_send[cnt++] = BYTE0(mpu_data.gyro_data.z);
	
	temp_angle = (int16_t)(euler_angle.roll);
	data_to_send[cnt++] = BYTE1(temp_angle);
	data_to_send[cnt++] = BYTE0(temp_angle);
	
	temp_angle = (int16_t)(euler_angle.pitch);
	data_to_send[cnt++] = BYTE1(temp_angle);
	data_to_send[cnt++] = BYTE0(temp_angle);
	
	temp_angle = (int16_t)(euler_angle.yaw);
	data_to_send[cnt++] = BYTE1(temp_angle);
	data_to_send[cnt++] = BYTE0(temp_angle);

	data_to_send[3] = cnt-4;//计算总数据的字节数。

	for(i=0;i<cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[cnt++]=sum;  //最后一位是校验和

	Send_Data(data_to_send, cnt);   //发送一个数据包
		
}



/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
End:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
