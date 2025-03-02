/**
 * 
*/
#include "definitions.h"
#include "MPU6050_MPU9150.h"

#ifndef _GAG_OFFSETTING_H_
#define _GAG_OFFSETTING_H_
// TODO move code for measuring offsets here ...
//#ifdef MEASURE_OFFSETS

//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=200;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=5;     //Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int16_t ax, ay, az,gx, gy, gz;
int16_t mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int16_t ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
uint8_t measurementsLimit = 0;
bool calibrationDone = false;

void meansensors(MPU6050_MPU9150 *mpuP);
void calibration(MPU6050_MPU9150 *mpuP, int16_t limit);
void measureOffsets(MPU6050_MPU9150 *mpuP, uint8_t i, int16_t limit);

#ifndef MASTER_SERIAL_NAME
#define MASTER_SERIAL_NAME Serial
#endif

void meansensors(MPU6050_MPU9150 *mpuP){
    MPU6050_MPU9150 accelgyro = *mpuP;
    long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

    while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
        buff_ax=buff_ax+ax;
        buff_ay=buff_ay+ay;
        buff_az=buff_az+az;
        buff_gx=buff_gx+gx;
        buff_gy=buff_gy+gy;
        buff_gz=buff_gz+gz;
    }

    if (i==(buffersize+100)){
        mean_ax=buff_ax/buffersize;
        mean_ay=buff_ay/buffersize;
        mean_az=buff_az/buffersize;
        mean_gx=buff_gx/buffersize;
        mean_gy=buff_gy/buffersize;
        mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
    }
}

void calibration(MPU6050_MPU9150 *mpuP,  int16_t limit, uint8_t i){
    MPU6050_MPU9150 accelgyro = *mpuP;
    ax_offset=-mean_ax/8;
    ay_offset=-mean_ay/8;
    az_offset=(16384-mean_az)/8;

    gx_offset=-mean_gx/4;
    gy_offset=-mean_gy/4;
    gz_offset=-mean_gz/4;
    // measurementsLimit = MEASUREMENT_LIMIT;
    measurementsLimit = limit;
    while (1){
        int ready=0;
        accelgyro.setXAccelOffset(ax_offset);
        accelgyro.setYAccelOffset(ay_offset);
        accelgyro.setZAccelOffset(az_offset);
        accelgyro.setXGyroOffset(gx_offset);
        accelgyro.setYGyroOffset(gy_offset);
        accelgyro.setZGyroOffset(gz_offset);
        meansensors(mpuP);
        MASTER_SERIAL_NAME.println("...");
        if (abs(mean_ax)<=acel_deadzone) ready++;
        else ax_offset=ax_offset-mean_ax/acel_deadzone;
        if (abs(mean_ay)<=acel_deadzone) ready++;
        else ay_offset=ay_offset-mean_ay/acel_deadzone;
        if (abs(16384-mean_az)<=acel_deadzone) ready++;
        else az_offset=az_offset+(16384-mean_az)/acel_deadzone;
        if (abs(mean_gx)<=giro_deadzone) ready++;
        else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);
        if (abs(mean_gy)<=giro_deadzone) ready++;
        else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);
        if (abs(mean_gz)<=giro_deadzone) ready++;
        else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);
        #define PRINT_VALIBRATION_VALUES
        #ifdef PRINT_VALIBRATION_VALUES
        MASTER_SERIAL_NAME.print(F("R: "));
        MASTER_SERIAL_NAME.print(ready);
        MASTER_SERIAL_NAME.print(F(" ml: "));
        MASTER_SERIAL_NAME.print(measurementsLimit);
        MASTER_SERIAL_NAME.print(F(" adz:"));
        MASTER_SERIAL_NAME.print(acel_deadzone);
        MASTER_SERIAL_NAME.print(F("  max:"));
        MASTER_SERIAL_NAME.print(mean_ax);
        MASTER_SERIAL_NAME.print(F(" axo:"));
        MASTER_SERIAL_NAME.print(ax_offset);

        MASTER_SERIAL_NAME.print(F("  may:"));
        MASTER_SERIAL_NAME.print(mean_ay);
        MASTER_SERIAL_NAME.print(F(" ayo:"));
        MASTER_SERIAL_NAME.print(ay_offset);

        MASTER_SERIAL_NAME.print(F("  maz:"));
        MASTER_SERIAL_NAME.print(mean_az);
        MASTER_SERIAL_NAME.print(F(" azo:"));
        MASTER_SERIAL_NAME.print(az_offset);

        MASTER_SERIAL_NAME.print(F("   gdz:"));
        MASTER_SERIAL_NAME.print(giro_deadzone);
        MASTER_SERIAL_NAME.print(F(" mgx:"));
        MASTER_SERIAL_NAME.print(mean_gx);
        MASTER_SERIAL_NAME.print(F(" gxo:"));
        MASTER_SERIAL_NAME.print(gx_offset);

        MASTER_SERIAL_NAME.print(F("  mgy:"));
        MASTER_SERIAL_NAME.print(mean_gy);
        MASTER_SERIAL_NAME.print(F(" gyof:"));
        MASTER_SERIAL_NAME.print(gy_offset);

        MASTER_SERIAL_NAME.print(F("  mgz:"));
        MASTER_SERIAL_NAME.print(mean_gz);
        MASTER_SERIAL_NAME.print(F(" gzof:"));
        MASTER_SERIAL_NAME.println(gz_offset);
        #endif
        if (--measurementsLimit <= 0 || ready>=6 ) break;
    }
}
extern int16_t sensorsOffsets[7][6];

void measureOffsets(MPU6050_MPU9150 *mpuP, uint8_t i, int16_t limit){
    ax=0; ay=0; az=0;gx=0; gy=0; gz=0;
    mean_ax=0;mean_ay=0;mean_az=0;mean_gx=0;mean_gy=0;mean_gz=0;state=0;
    ax_offset=0;ay_offset=0;az_offset=0;gx_offset=0;gy_offset=0;gz_offset=0;
    
  
    MPU6050_MPU9150 mpu = *mpuP;
    if(mpu.isMPU9150) {
        mpu.setXAccelOffset(0, MPU9150_RA_XA_OFFS_H);
        mpu.setYAccelOffset(0, MPU9150_RA_YA_OFFS_H);
        mpu.setZAccelOffset(0, MPU9150_RA_ZA_OFFS_H);
        mpu.setXGyroOffset(0, MPU9150_RA_XG_OFFS_USRH);
        mpu.setYGyroOffset(0, MPU9150_RA_YG_OFFS_USRH);
        mpu.setZGyroOffset(0, MPU9150_RA_ZG_OFFS_USRH);
    } else {
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    
    }
    //mpu.setXOffset(0);
    //mpu.setYGyroOffset(0);
    //mpu.setZGyroOffset(0);
    if (state==0){
        MASTER_SERIAL_NAME.println(F("\nReading sensors for first time..."));
        meansensors(mpuP);
        state++;
        delay(1000);
    }

    if (state==1) {
        MASTER_SERIAL_NAME.println(F("\nCalculating offsets..."));
        calibration(mpuP, limit, i);
        state++;
        delay(1000);
    }

    if (state==2) {
        meansensors(mpuP);
        MASTER_SERIAL_NAME.println(F("\nFINISHED!"));
        MASTER_SERIAL_NAME.print(F("\nSensor readings with offsets:\t"));
        MASTER_SERIAL_NAME.print(mean_ax); 
        MASTER_SERIAL_NAME.print(F("\t"));
        MASTER_SERIAL_NAME.print(mean_ay); 
        MASTER_SERIAL_NAME.print(F("\t"));
        MASTER_SERIAL_NAME.print(mean_az); 
        MASTER_SERIAL_NAME.print(F("\t"));
        MASTER_SERIAL_NAME.print(mean_gx); 
        MASTER_SERIAL_NAME.print(F("\t"));
        MASTER_SERIAL_NAME.print(mean_gy); 
        MASTER_SERIAL_NAME.print(F("\t"));
        MASTER_SERIAL_NAME.println(mean_gz);
        MASTER_SERIAL_NAME.print(F("Your offsets:\t"));
        MASTER_SERIAL_NAME.print(ax_offset); 
        MASTER_SERIAL_NAME.print(F("\t"));
        MASTER_SERIAL_NAME.print(ay_offset); 
        MASTER_SERIAL_NAME.print(F("\t"));
        MASTER_SERIAL_NAME.print(az_offset); 
        MASTER_SERIAL_NAME.print(F("\t"));
        MASTER_SERIAL_NAME.print(gx_offset); 
        MASTER_SERIAL_NAME.print(F("\t"));
        MASTER_SERIAL_NAME.print(gy_offset); 
        MASTER_SERIAL_NAME.print(F("\t"));
        MASTER_SERIAL_NAME.println(gz_offset); 
        MASTER_SERIAL_NAME.println(F("\nData is printed as: acelX acelY acelZ giroX giroY giroZ"));
        MASTER_SERIAL_NAME.println(F("Check that your sensor readings are close to 0 0 16384 0 0 0"));
        //MASTER_SERIAL_NAME.println(F("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)"));
        //while (1);
        MASTER_SERIAL_NAME.println(F("Setting measurmeasureed offsets..."));

        sensorsOffsets[i][0] = ax_offset;
        sensorsOffsets[i][1] = ay_offset;
        sensorsOffsets[i][2] = az_offset;
        sensorsOffsets[i][3] = gx_offset;
        sensorsOffsets[i][4] = gy_offset;
        sensorsOffsets[i][5] = gz_offset;


if(mpu.isMPU9150) {
        mpu.setXAccelOffset(sensorsOffsets[i][0], MPU9150_RA_XA_OFFS_H);
        mpu.setYAccelOffset(sensorsOffsets[i][1], MPU9150_RA_YA_OFFS_H);
        mpu.setZAccelOffset(sensorsOffsets[i][2], MPU9150_RA_ZA_OFFS_H);
        mpu.setXGyroOffset(sensorsOffsets[i][3], MPU9150_RA_XG_OFFS_USRH);
        mpu.setYGyroOffset(sensorsOffsets[i][4], MPU9150_RA_YG_OFFS_USRH);
        mpu.setZGyroOffset(sensorsOffsets[i][5], MPU9150_RA_ZG_OFFS_USRH);
        } else {        
          mpu.setXAccelOffset(sensorsOffsets[i][0]);
        mpu.setYAccelOffset(sensorsOffsets[i][1]);
        mpu.setZAccelOffset(sensorsOffsets[i][2]);   
        mpu.setXGyroOffset(sensorsOffsets[i][3]);
        mpu.setYGyroOffset(sensorsOffsets[i][4]);
        mpu.setZGyroOffset(sensorsOffsets[i][5]);
}
/*
        mpu.setXGyroOffset(gx_offset);
        mpu.setYGyroOffset(gy_offset);
        mpu.setZGyroOffset(gz_offset);
        mpu.setXAccelOffset(ax_offset);
        mpu.setYAccelOffset(ay_offset);
        mpu.setZAccelOffset(az_offset);
        */
    }
}
#endif
