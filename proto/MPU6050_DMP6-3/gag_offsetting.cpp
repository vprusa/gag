/*
*/

#include "gag_offsetting.h"
#include "gag.h"

#ifndef MASTER_SERIAL_NAME
#define MASTER_SERIAL_NAME Serial
#endif

void meansensors(MPU6050_MPU9250 *mpuP){
    MPU6050_MPU9250 accelgyro = *mpuP;
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

void calibration(MPU6050_MPU9250 *mpuP){
    MPU6050_MPU9250 accelgyro = *mpuP;
    ax_offset=-mean_ax/8;
    ay_offset=-mean_ay/8;
    az_offset=(16384-mean_az)/8;

    gx_offset=-mean_gx/4;
    gy_offset=-mean_gy/4;
    gz_offset=-mean_gz/4;

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
        MASTER_SERIAL_NAME.print(F("Ready: "));
        MASTER_SERIAL_NAME.print(ready);
        MASTER_SERIAL_NAME.print(F(" acel_deadzone:"));
        MASTER_SERIAL_NAME.print(acel_deadzone);
        MASTER_SERIAL_NAME.print(F(" mean_ax:"));
        MASTER_SERIAL_NAME.print(mean_ax);
        MASTER_SERIAL_NAME.print(F(" ax_offset:"));
        MASTER_SERIAL_NAME.print(ax_offset);

        MASTER_SERIAL_NAME.print(F(" mean_ay:"));
        MASTER_SERIAL_NAME.print(mean_ay);
        MASTER_SERIAL_NAME.print(F(" ay_offset:"));
        MASTER_SERIAL_NAME.print(ay_offset);

        MASTER_SERIAL_NAME.print(F(" mean_az:"));
        MASTER_SERIAL_NAME.print(mean_az);
        MASTER_SERIAL_NAME.print(F(" az_offset:"));
        MASTER_SERIAL_NAME.print(az_offset);

        MASTER_SERIAL_NAME.print(F(" giro_deadzone:"));
        MASTER_SERIAL_NAME.print(giro_deadzone);
        MASTER_SERIAL_NAME.print(F(" mean_gx:"));
        MASTER_SERIAL_NAME.print(mean_gx);
        MASTER_SERIAL_NAME.print(F(" gx_offset:"));
        MASTER_SERIAL_NAME.print(gx_offset);

        MASTER_SERIAL_NAME.print(F(" mean_gy:"));
        MASTER_SERIAL_NAME.print(mean_gy);
        MASTER_SERIAL_NAME.print(F(" gy_offset:"));
        MASTER_SERIAL_NAME.print(gy_offset);

        MASTER_SERIAL_NAME.print(F(" mean_gz:"));
        MASTER_SERIAL_NAME.print(mean_gz);
        MASTER_SERIAL_NAME.print(F(" gz_offset:"));
        MASTER_SERIAL_NAME.println(gz_offset);
        #endif
        if (ready==6) break;
    }
}

void measureOffsets(MPU6050_MPU9250 *mpuP){
    MPU6050_MPU9250 mpu = *mpuP;
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
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
        calibration(mpuP);
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
        MASTER_SERIAL_NAME.println(F("Setting measured offsets..."));
        mpu.setXGyroOffset(gx_offset);
        mpu.setYGyroOffset(gy_offset);
        mpu.setZGyroOffset(gz_offset);
        mpu.setXAccelOffset(ax_offset);
        mpu.setYAccelOffset(ay_offset);
        mpu.setZAccelOffset(az_offset);
    }
}