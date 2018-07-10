
#include "global.h"

//条件编译，调试时各模块的开关
#define NEED_MPU6050   0
#define NEED_BLUETOOTH 1


//unsigned char string_1[] = {"1.3' OLED TEST  "}; //可以不给出数组大小
//unsigned char string_2[] = {"THIS IS A TEST  "};

unsigned char CmdRx_Buffer[10] = {0};//串口接收缓冲

u8 light_screen_2;
extern volatile u8 light_screen;//按键中断函数里面设置的变量，亮屏标志。
extern volatile u8 timing_flag; //亮屏定时标志

extern volatile unsigned char adjust_real_time_flag ;	//是否校准时间标志

extern MPU6050_DATA mpu_data ;  //原始数据
extern EULER euler_angle;		  //转化为姿态角

//主函数中的大循环
void main_loop(void);
extern void key_init(void);//这个函数没有使用其他头文件声明，直接在这里外部声明吧。



/*********************主函数***********************/
int main(void) 
{  	
	OLED_Init();       		//OLED初始化
	
	i2c_GPIO_Config();		//iic协议引脚配置，它需要在6050初始化前百毫秒初始化
		
	DS1302_Init();     		//时钟模块初始化
	
	key_init();        		//按键初始化
	
	ADC_All_Init();    		//ADC初始化，开启DMA传输中断
	
	Usart1_GPIO_Init();      //串口，蓝牙或者串口转USB向上位机发送数据
	Usart1_Configuration(115200);   		//串口配置115200波特率
	
	OLED_Clear();       		//清屏一次

		
	light_screen = 0;
	light_screen_2 = 0;
	
	//结构体初始化清零
	memset(&mpu_data,0,sizeof(MPU6050_DATA));	
	memset(&euler_angle,0,sizeof(EULER));	
	
	MPU6050_Init();					//6050初始化
	while(!MPU6050ReadID());  		//等待6050读取成功
	IMU_init();
	
	time_2_init(1);			//开启定时器2，定时1ms，用于循环执行一些函数
	
	while(1)//主循环
	{		
		
		if(light_screen == 1)		//按键或者动作触发亮屏
		{
			Timing_1ms(3000);//定时3s
			OLED_Display_On(); //开显示
			
			while(timing_flag == 0) //循环检测定时器有没有到时间
			{
				if(light_screen == 1)
				{
					Display_Real_Time();//在显示时间内，时间还是在刷新的
					OLED_Show_RealTime_Battery(110,0);//110横坐标可以把电池显示在最右侧
				}
				else
					OLED_Show_Big_Time(36,0,48,2);

			}
			OLED_Clear();			//显示完之后清一次屏，防止影响下一次亮屏
			OLED_Display_Off(); //关显示
			timing_flag = !timing_flag;//定时时间到之后，标志再次取反，以备下一次定时器使用
			light_screen = !light_screen;				
			
		}
		//检测是否需要校准时间
		if(adjust_real_time_flag != 0)
		{
			adjust_real_time_flag = 0;
			adjust_real_time();			//调用函数向DS1302写入校准后的时间
		}

	}//主循环结束
		
}


/*
** 在定时器中定时循环执行所有函数
** 该函数被定时器2的中断函数调用，定时1ms调用一次
** 在这个函数中对1ms进行累计，不同函数，执行周期不同。
*/
void main_loop(void)
{
	static unsigned char time_1ms = 0, time_10ms = 0 ;
	static int16_t time_1s = 0;	//第二次动作检测标志
		
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET )		//1ms中断到来
	{	
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);    		//清除中断标志
		
  		time_1ms++;
		time_10ms++;
		time_1s++;                                                                                                                                                                                                                                                                                                                                                                                                               

		if(time_1ms == 3)//12ms
		{
			time_1ms = 0;					//清除标志	
			mpu6050_data_process();    //获取6050数据并进行四元数姿态解算
						
			#if NEED_MPU6050
			Send_Senser();             //与上位机通信接口，传输数据				
			#endif
			
			if(euler_angle.roll >= 150) //由动作亮屏
			{
				light_screen = 1;
			}
			
		}
		
		#if NEED_BLUETOOTH
		if(time_10ms == 10)
		{
			time_10ms = 0;

			if(strstr((const char *)CmdRx_Buffer,"LEDON"))   //判断蓝牙串口接收的字符串,strstr返回后面字符串在前面字符串中的第一次位置
			{
				light_screen = 1;
				memset(CmdRx_Buffer,0,sizeof(CmdRx_Buffer));		//数组清零
			}
			else if(strstr((const char *)CmdRx_Buffer,"LEDOFF"))
			{
				light_screen = 0;
				memset(CmdRx_Buffer,0,sizeof(CmdRx_Buffer));
			}
		
		}
		#endif

		if(time_1s >= 100) 
		{
			time_1s = 0;		//注意清零
			if(light_screen == 1 && euler_angle.roll >= 150)
			{
				light_screen = 0;
				light_screen_2 = 1;	//在显示第一屏也就是时间的基础上，才能显示第二幅图	
			}
		}			
	}	
}



