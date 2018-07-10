
//加入按键，实验阶段，按键按下时亮屏，点亮10秒后熄灭。这个板子上，得把J-LINK的线拔了才能用按键KEY2。
//这个板子上key2 接在PA15
//当中断发生时，处理器首先会完成中断现场的保护，然后跳转到中断向量表中查找中断处理函数的入口地址，
//进而执行相应的中断处理函数
#include <stm32f10x.h>

/*----------------------------------
**函数名称：key_init
**功能描述：按键引脚初始化
**参数说明：无
**作者：Andrew
**日期：2018.1.25
-----------------------------------*/
void key_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;			//定义一个EXTI结构体变量

	//1、使能GPIO和外部中断必须是能APIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);

	//2、GPIO初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //中断设置为浮空输入
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入试试，也可以
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//3、设置EXTI线
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);//配置中断源,将GPIO与中断映射一起
	EXTI_InitStructure.EXTI_Line = EXTI_Line15 ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//中断模式为中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//下降沿出发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				//使能中断线
	EXTI_Init(&EXTI_InitStructure);							//根据参数初始化中断寄存器
	
	//4、配置中断向量
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //中断分组1
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//设定中断源为
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//中断占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//副优先级为1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断
	NVIC_Init(&NVIC_InitStructure);							   	//根据参数初始化中断寄存器
	
	//到此，中断所有的设置就配置完了，这是简单的按键的中断，其他的可能要复杂一点
	
	
}
//中断函数在IT.C中

