
#include "filter.h"


float one_filter_angle = 0;
float kalman_filter_angle = 0, kalman_filter_angle_dot = 0;

/*----------------------------------
**函数名称：one_filter
**功能描述：一阶互补滤波
**算法说明：根据加速度计和陀螺仪的各自优缺点，分别给他们不同的权值，把他们结合到一起计算出一个姿态角
**参数说明：angle_m：为加速度得到的角度
			gyro_m：为陀螺仪得到的角速度
**作者：Andrew
**日期：2018.1.25
-----------------------------------*/
float one_filter(float angle_m,float gyro_m)
{

	float K1 =0.2;  //对加速度计的权重
	float dt=0.005;  //dt为滤波器采样时间
	
	one_filter_angle = K1 * angle_m+ (1-K1) * (one_filter_angle + gyro_m * dt);
	
	return one_filter_angle;
}

/*----------------------------------
**函数名称：kalman_filter
**功能描述：卡尔曼滤波
算法说明：根据当前时刻的观测值、上一时刻的预测值及预测误差,计算得到当前的最优量去预测下一刻的量
**参数说明：angle_m：为加速度得到的角度
			gyro_m：为陀螺仪得到的角速度
**作者：Andrew
**日期：2018.1.25
-----------------------------------*/
float kalman_filter(float angle_m,float gyro_m)
{

	float dt = 0.005;   //dt也是滤波器的采样时间
	float P[2][2]	= {{1,0},{0,1}};//一般可以修改 P Q R 三个参数。
	float Pdot[4]	= {0,0,0,0};
	float Q_angle = 0.001;
	float Q_gyro = 0.005;
	float R_angle = 0.5;
	char  C_0 = 1;
	float q_bias = 0,angle_err = 0; 
	float PCt_0 = 0,PCt_1 = 0,E = 0;
	float K_0 = 0,  K_1 = 0,  t_0 = 0,  t_1 = 0;

 	kalman_filter_angle+= (gyro_m - q_bias) * dt;

    Pdot[0]=Q_angle - P[0][1] - P[1][0];
    Pdot[1] = -P[1][1];
    Pdot[2] = -P[1][1];
    Pdot[3]=Q_gyro;

    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;  
    K_1 = PCt_1 / E;

	angle_err = angle_m - kalman_filter_angle;
    kalman_filter_angle += K_0 * angle_err;
    q_bias += K_1 * angle_err;
    kalman_filter_angle_dot = gyro_m-q_bias;

	t_0 = PCt_0;
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;

    return kalman_filter_angle;
}



