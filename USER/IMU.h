
#ifndef __IMU_H
#define __IMU_H

#include "mpu6050.h"

void IMU_init(void);


void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);

void get_euler_angle(void);

#endif

