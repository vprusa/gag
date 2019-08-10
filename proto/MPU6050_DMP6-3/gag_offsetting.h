/*
*/

#ifndef _GAG_OFFSETTING_
#define _GAG_OFFSETTING_

#include "gag.h"
#include "MPU6050_MPU9250_9Axis_MotionApps41.h"

//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int16_t ax, ay, az,gx, gy, gz;
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

void meansensors(MPU6050_MPU9250 *mpuP);
void calibration(MPU6050_MPU9250 *mpuP);
void measureOffsets(MPU6050_MPU9250 *mpuP);

#endif
