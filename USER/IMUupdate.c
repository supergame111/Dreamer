
#include "global.h"


//通过调节Kp，Ki两个参数，可以控制加速度计修正陀螺仪积分姿态的速度
#define Kp 3.0f       	
#define Ki 0.003f       		
#define halfT 0.004f    // 采样周期的一半

extern MPU6050_DATA mpu_data ;
extern EULER euler_angle;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // 四元数的元素，代表估计方向
float exInt = 0, eyInt = 0, ezInt = 0;         // 按比例缩小积分误差

/*
** 四元数赋初值
** 事先最好把加速度计校准，这个时候的精度取决于加速度计的精度。六面校准.
** 没有磁传感器的姿态融合下，只能暂时给偏航角 = 0，这样就只能确定水平面
** 只需在主函数初始化的时候调用一次即可.
*/
void IMU_init(void)
{
	float norm;
	float init_ax, init_ay, init_az;
	float init_roll, init_pitch, init_yaw;
	
	MPU6050ReadAcc();
	
	init_ax =  (float)mpu_data.acc_data.x / ACC_8G;	   //计算单位为G的各轴重力加速度分量,与量程有关
	init_ay =  (float)mpu_data.acc_data.y / ACC_8G;
	init_az =  (float)mpu_data.acc_data.z / ACC_8G;

	//归一化
	norm = sqrt(init_ax*init_ax + init_ay*init_ay + init_az*init_az);      
	init_ax = init_ax / norm;            
	init_ay = init_ay / norm;
	init_az = init_az / norm; 
	
	//陀螺仪x轴为前进方向
	init_roll  = atan2(init_ay, init_az);
	init_pitch = -asin(init_ax);              //init_Pitch = asin(ax / 1);   
//	init_yaw   = -atan2(init_my*cos(init_Roll) - init_mz*sin(init_Roll),init_mx*cos(init_Pitch) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Pitch)*cos(init_Roll));                       //atan2(mx, my);
   init_yaw   = 0;
 
	q0 = cos(0.5*init_roll)*cos(0.5*init_pitch)*cos(0.5*init_yaw) + sin(0.5*init_roll)*sin(0.5*init_pitch)*sin(0.5*init_yaw);  //w
	q1 = sin(0.5*init_roll)*cos(0.5*init_pitch)*cos(0.5*init_yaw) - cos(0.5*init_roll)*sin(0.5*init_pitch)*sin(0.5*init_yaw);  //x   绕x轴旋转是roll
	q2 = cos(0.5*init_roll)*sin(0.5*init_pitch)*cos(0.5*init_yaw) + sin(0.5*init_roll)*cos(0.5*init_pitch)*sin(0.5*init_yaw);  //y   绕y轴旋转是pitch
	q3 = cos(0.5*init_roll)*cos(0.5*init_pitch)*sin(0.5*init_yaw) - sin(0.5*init_roll)*sin(0.5*init_pitch)*cos(0.5*init_yaw);  //z   绕z轴旋转是yaw

 
}

/* 四元数更新算法
** 参数gx，gy，gz分别对应三个轴的角速度，单位是弧度/秒;
** 参数ax，ay，az分别对应三个轴的加速度原始数据,由于加速度的噪声较大，此处应采用滤波后的数据
*/
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
		float norm;
		float vx, vy, vz;
		float ex, ey, ez; 
	
//		float q0_last,	q1_last,q2_last,q3_last;	//这个写不写都可以，写出来只是便于理解，其实他和q0是一个值
//		 //四元数积分，求得当前姿态
//		 q0_last = q0;
//		 q1_last = q1;
//		 q2_last = q2;
//		 q3_last = q3;
	
		//加速度计的三维向量转换为单位向量
		norm = sqrt(ax*ax + ay*ay + az*az);      
		ax = ax / norm;            
		ay = ay / norm;
		az = az / norm;      

		//估计重力加速度的方向在飞行器坐标中的表示
		vx = 2*(q1*q3 - q0*q2);
		vy = 2*(q0*q1 + q2*q3);
		vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

		//加速度计读取的方向与重力加速度读取方向的差值，向量的叉乘
		ex = (ay*vz - az*vy);
		ey = (az*vx - ax*vz);
		ez = (ax*vy - ay*vx);

		//误差累计
		exInt = exInt + ex*Ki;
		eyInt = eyInt + ey*Ki;
		ezInt = ezInt + ez*Ki;

		//用叉积来做PI修正陀螺零偏，抵消陀螺仪中的偏移量
		gx = gx + Kp*ex + exInt;
		gy = gy + Kp*ey + eyInt;
		gz = gz + Kp*ez + ezInt;


		//一阶近似算法，四元数运动方程的离散化形式和积分
		q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
		q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
		q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
		q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

		//四元数规范化
		norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
		q0 = q0 / norm;
		q1 = q1 / norm;
		q2 = q2 / norm;
		q3 = q3 / norm;

}

/*
** 四元数转化为欧拉角，根据旋转轴的次序不同，公式也不同。
** 在此以x轴为旋转轴。
*/
void get_euler_angle(void)
{	
	euler_angle.roll  = atan2(2.0f * (q0*q1+q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3) * 180 / PI;
	euler_angle.pitch = asin(2.0f * (q0*q2 - q1*q3)) * 180 / PI;
	euler_angle.yaw   = atan2(2.0f * (q0*q1 + q2*q3),q0*q0 + q1*q1 - q2*q2 - q3*q3) * 180 / PI;
	
}


