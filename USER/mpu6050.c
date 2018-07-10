
#include "global.h"



MPU6050_DATA mpu_data ;  		//原始数据
EULER euler_angle;				//转化为姿态角

#define ACC_FILTER_NUM 3		//加速度计滑动滤波器采样长度
#define GYRO_FILTER_NUM 3		//陀螺仪滑动滤波器采样长度


//仅在此文件内调用，声明为静态函数
//static void MPU6050ReadGyro(void);
//static void MPU6050ReadAcc(void);
static void MPU6050_ReadTemp(short*Temperature);
static void MPU6050_ReadData(u8 reg_add,unsigned char*Read,u8 num);
static void MPU6050_WriteReg(u8 reg_add,u8 reg_dat);


/*----------------------------------
**函数名称：MPU6050_WriteReg
**功能描述：6050写寄存器操作
**参数说明：reg_add：寄存器地址
						reg_dat：写入的数据
**作者：Andrew
**日期：2018.1.24
-----------------------------------*/
static void MPU6050_WriteReg(u8 reg_add,u8 reg_dat)
{
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);    //写从机地址，并配置成写模式
	i2c_WaitAck();
	i2c_SendByte(reg_add);     //写寄存器地址
	i2c_WaitAck();
	i2c_SendByte(reg_dat);     //写寄存器数据
	i2c_WaitAck();
	i2c_Stop();
}


/*----------------------------------
**函数名称：MPU6050_ReadData
**功能描述：读取6050数据
**参数说明：reg_add：寄存器地址
				* Read：存放读取的数值
				num：需要读取的次数
**作者：Andrew
**日期：2018.1.24
-----------------------------------*/
static void MPU6050_ReadData(u8 reg_add,unsigned char *Read,u8 num)
{
	unsigned char i;
	
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);
	i2c_WaitAck();
	i2c_SendByte(reg_add);
	i2c_WaitAck();
	
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS+1);   //写从机地址，并配置成读模式
	i2c_WaitAck();
	
	for(i=0;i<num;i++)
	{
		*Read=i2c_ReadByte(1);
		Read++;
	}
	*Read=i2c_ReadByte(0);
	i2c_Stop();
}

//**************至此mpu6050的读写操作函数都搞定了，下面只需要初始化就可以读取他的数据了


/*----------------------------------
**函数名称：MPU6050_Init
**功能描述：6050初始化，各种寄存器初始数据，参见寄存器参数功能表
**参数说明：无
**作者：Andrew
**日期：2018.1.24
**2018.3.19  关于加速度计参数配置。0x00->2G,0x10->8G
-----------------------------------*/
void MPU6050_Init(void)
{
  int i=0,j=0;
	
  //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
  //仿真时间为 0.28646919s - 0.21462243s = 0.072s,72ms
  for(i=0;i<1000;i++)
  {
    for(j=0;j<1000;j++)
    {
		;
    }
  } 
	MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	    //解除休眠状态
	MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x07);	    //陀螺仪采样率，1KHz
	MPU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);	        //低通滤波器的设置，截止频率是1K，带宽是5K
	MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x00);	  //配置加速度传感器工作在+-8G模式，不自检,32767/8 = 4096LSB/G
	MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);     //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s,16.4LSB/S)
}

/*----------------------------------
**函数名称：MPU6050ReadID
**功能描述：根据读取MPU6050_RA_WHO_AM_I寄存器的值确定6050是否正确初始化
**参数说明：返回值1成功，0不成功
**作者：Andrew
**日期：2018.1.24
-----------------------------------*/
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
	
    MPU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    //读器件地址
	
	if(Re != 0x68)  //这个寄存器的典型值为0x68
	{
//		printf("MPU6050 dectected error!\r\n检测不到MPU6050模块，请检查模块与开发板的接线");
		return 0;
	}
	else
	{
//		printf("MPU6050 ID = %d\r\n",Re);
		return 1;
	}

}

/*----------------------------------
**函数名称：MPU6050ReadAcc
**功能描述：读取加速度寄存器的值，并在必要的时候减去零偏值
**参数说明：无
**作者：Andrew
**日期：2018.1.24
-----------------------------------*/
void MPU6050ReadAcc(void)
{
    uint8_t buf[6];
	 int16_t accData[3];	//加速度计输出16位带符号数，其实int16_t就是 signed short int 占2个字节
	
    MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6);   //获取加速度计寄存器的数据，测量轴加速度
    accData[0] = (int16_t)((buf[0] << 8) | buf[1]);	 //先转换为16位数据，再左移8位。
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]);
	
		mpu_data.acc_data.x = accData[0];
		mpu_data.acc_data.y = accData[1];
		mpu_data.acc_data.z = accData[2];
		
}


/*----------------------------------
**函数名称：MPU6050ReadGyro
**功能描述：读取陀螺仪数值
**参数说明：无
**作者：Andrew
**日期：2018.1.24
-----------------------------------*/
void MPU6050ReadGyro(void)
{
    uint8_t buf[6];
	 int16_t gyroData[3];
	
    MPU6050_ReadData(MPU6050_GYRO_OUT,buf,6);  //获取陀螺仪寄存器的数据，测量的是角加速度
    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]);
	
		mpu_data.gyro_data.x = gyroData[0];
		mpu_data.gyro_data.y = gyroData[1];
		mpu_data.gyro_data.z = gyroData[2];
	
}


/*----------------------------------
**函数名称：MPU6050_ReadTemp
**功能描述：读取6050温度，转化成摄氏度
**参数说明：* Temperature：存放转化后的温度值
**作者：Andrew
**日期：2018.1.24
-----------------------------------*/
static void MPU6050_ReadTemp(short* Temperature)
{
	short temp3;
	uint8_t buf[2];
	
	MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
	
	temp3= (buf[0] << 8) | buf[1];
	
	*Temperature=(((double) (temp3 + 13200)) / 280)-13;

}


/*----------------------------------
**函数名称：mpu6050_data_process
**功能描述：6050原始数据的滤波
**参数说明：无
**作者：Andrew
**日期：2018.1.24
**这个函数耗时大概 7ms
-----------------------------------*/
void mpu6050_data_process(void)
{
	unsigned char i = 0,  j = 0;
	static unsigned char acc_filter_cnt = 0;//滤波次数计数
	static unsigned char gyro_filter_cnt = 0;
	
	int32_t  acc_temp[3] = {0};//三轴加速度,注意，这个不能定义为静态
	int32_t  gyro_temp[3] = {0};
	
	static int16_t  acc_x_buf[ACC_FILTER_NUM] = {0}, //滑动窗口缓存，每调用一次，里面数据多一个。
						 acc_y_buf[ACC_FILTER_NUM] = {0},
						 acc_z_buf[ACC_FILTER_NUM] = {0};

	static int16_t  gyro_x_buf[GYRO_FILTER_NUM] = {0}, //滑动窗口缓存，每调用一次，里面数据多一个。
						 gyro_y_buf[GYRO_FILTER_NUM] = {0},
   	             gyro_z_buf[GYRO_FILTER_NUM] = {0};

//	float  init_ax = 0,init_ay = 0,init_az = 0;

//结构体初始化，放在主函数初始化里面，运行前清零一次即可。
//	memset(&mpu_data,0,sizeof(MPU6050_DATA));	
//	memset(&euler_angle,0,sizeof(EULER));	

//获取原始数据						 
	MPU6050ReadAcc();   
	MPU6050ReadGyro();

//滑动滤波,加速度计数据处理
//-----------------------------------------------------
	acc_x_buf[acc_filter_cnt] = mpu_data.acc_data.x;
	acc_y_buf[acc_filter_cnt] = mpu_data.acc_data.y;
	acc_z_buf[acc_filter_cnt] = mpu_data.acc_data.z;

	for(i=0;i<ACC_FILTER_NUM;i++)	
	{
		acc_temp[0]+= acc_x_buf[i];
		acc_temp[1]+= acc_y_buf[i];
		acc_temp[2]+= acc_z_buf[i];
	}

	//---------------------------
	mpu_data.acc_filter.x = (float)acc_temp[0] / ACC_FILTER_NUM; 
	mpu_data.acc_filter.y = (float)acc_temp[1] / ACC_FILTER_NUM;
	mpu_data.acc_filter.z = (float)acc_temp[2] / ACC_FILTER_NUM;
	//--------------------------

	acc_filter_cnt++;
	if(acc_filter_cnt == ACC_FILTER_NUM)
	{
		acc_filter_cnt = 0;
	}
	

//陀螺仪数据处理
//-----------------------------------------------------------
	gyro_x_buf[gyro_filter_cnt] = mpu_data.gyro_data.x;
	gyro_y_buf[gyro_filter_cnt] = mpu_data.gyro_data.y;
	gyro_z_buf[gyro_filter_cnt] = mpu_data.gyro_data.z;

	for(j=0;j<GYRO_FILTER_NUM;j++)
	{
		gyro_temp[0]+= gyro_x_buf[j];
		gyro_temp[1]+= gyro_y_buf[j];
		gyro_temp[2]+= gyro_z_buf[j];
	}

	//-----------------------------
	mpu_data.gyro_filter.x = gyro_temp[0] / GYRO_FILTER_NUM;
	mpu_data.gyro_filter.y = gyro_temp[1] / GYRO_FILTER_NUM;
	mpu_data.gyro_filter.z = gyro_temp[2] / GYRO_FILTER_NUM;
	//--------------------------

	gyro_filter_cnt++;
	if(gyro_filter_cnt == GYRO_FILTER_NUM)
	{
		gyro_filter_cnt = 0;
	}

	//陀螺仪数据化为弧度制
	mpu_data.gyro_rad.x = (float)mpu_data.gyro_data.x * GYRO_R;
	mpu_data.gyro_rad.y = (float)mpu_data.gyro_data.y * GYRO_R;
	mpu_data.gyro_rad.z = (float)mpu_data.gyro_data.z * GYRO_R;

	
	//四元数更新，注意传入的参数与顺序。
	IMUupdate((float)mpu_data.gyro_rad.x,(float)mpu_data.gyro_rad.y,(float)mpu_data.gyro_rad.z,\
				 (float)mpu_data.acc_filter.x,(float)mpu_data.acc_filter.y,(float)mpu_data.acc_filter.z);
	
	//根据四元数计算欧拉角
	get_euler_angle();
	
}


