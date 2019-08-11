// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
/*
Copyright (c) 2018 Vojtěch Průša
*/
// Based on example of MPU6050 from https://github.com/jrowberg/i2cdevlib
// from 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// 
#define ESP32_RIGHT 1// master to left, slave to pc

//#define MEASURE_OFFSETS

#include "gag.h"

#ifdef MEASURE_OFFSETS
//#include "gag_offsetting.h"
#endif




void setup() {
//    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
//#ifdef MASTER_HAND
    #ifdef MASTER_HAND
    #ifdef MASTER_BT_SERIAL
    MASTER_SERIAL_NAME.begin(MASTER_BT_SERIAL_NAME);
    #else
    MASTER_SERIAL_NAME.begin(MASTER_SERIAL_BAUD);
    while (!MASTER_SERIAL_NAME)
        ; // wait for Leonardo enumeration, others continue immediately
    #endif
    #endif
    MASTER_SERIAL_NAME.println(F("USB up"));

#ifdef USE_DISPLAY
    displaySetup();
#endif

    for (int i = FIRST_SENSOR; i <= LAST_SENSOR; i++) {
        int sensorToEnable = SENSOR_PIN_OFFSET + i;
         
        #ifdef SENSOR_PIN_TU_COMPENSATION
        if (i == SENSOR_PIN_TU) {
            sensorToEnable = SENSOR_PIN_TU_COMPENSATION;
        }
        #endif
        #ifdef SENSOR_PIN_SU_COMPENSATION
        if (i == SENSOR_PIN_SU) {
            sensorToEnable = SENSOR_PIN_SU_COMPENSATION;
        }
        #endif
        #ifdef SENSOR_PIN_FU_COMPENSATION
        if (i == SENSOR_PIN_FU) {
            sensorToEnable = SENSOR_PIN_FU_COMPENSATION;
        }
        #endif
        #ifdef SENSOR_PIN_MU_COMPENSATION
        if (i == SENSOR_PIN_MU) {
            sensorToEnable = SENSOR_PIN_MU_COMPENSATION;
        }
        #endif
        #ifdef SENSOR_PIN_EU_COMPENSATION
        if (i == SENSOR_PIN_EU) {
            sensorToEnable = SENSOR_PIN_EU_COMPENSATION;
        }
        #endif
        #ifdef SENSOR_PIN_HP_COMPENSATION
        if (i == SENSOR_PIN_HP) {
            sensorToEnable = SENSOR_PIN_HP_COMPENSATION;
        }
        #endif
        pinMode(sensorToEnable, OUTPUT);
    }
#ifdef MASTER_HAND
    Wire.begin(21 , 22, 200000);
   // Wire.setTimeOut(2);
    //setTimeOut
    //Fastwire::setup(400, true);
    //Wire.begin();
    //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    //Wire.setClock(400000);
#else
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    //Fastwire::setup(400, true);
#endif
#endif
// initialize serial communication
// (9600 38400 57600 74880 115200 230400 250000 57600 38400 chosen because it is required for Teapot Demo output, but it's
// really up to you depending on your project)
#ifdef MASTER_HAND
    //TODO fix, rename
    SLAVE_SERIAL_NAME.begin(SLAVE_SERIAL_BAUD);
#endif

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    for (int i = FIRST_SENSOR; i <= LAST_SENSOR; i++) {
        selectedSensor = (Sensor)i;
        enableSingleMPU(selectedSensor);
        if(i == HP) {
            //gyros[selectedSensor].mpuM = new MPU6050(MPU6050_ADDRESS_AD0_HIGH); // MPU6050_ADDRESS_AD0_LOW / MPU6050_ADDRESS_AD0_HIGH
            //gyros[selectedSensor].mpuM = new MPU9250(MPU6050_ADDRESS_AD0_LOW); // MPU6050_ADDRESS_AD0_LOW / MPU6050_ADDRESS_AD0_HIGH
            //gyros[selectedSensor].mpuM->isMPU6050 = true;
            gyros[selectedSensor].mpu = new MPU6050_MPU9250(MPU6050_MPU9250_ADDRESS_AD0_LOW);//   0x68 / 0x69
            gyros[selectedSensor].mpu->isMPU9250 = true;
        } else {
            //gyros[selectedSensor].mpu = new MPU6050(MPU6050_ADDRESS_AD0_HIGH);//   0x68 / 0x69
            //gyros[selectedSensor].mpu = new MPU6050(MPU6050_ADDRESS_AD0_LOW);//   0x68 / 0x69
            //gyros[selectedSensor].mpu->isMPU6050 = true;
            gyros[selectedSensor].mpu = new MPU6050_MPU9250(MPU6050_MPU9250_ADDRESS_AD0_LOW);//   0x68 / 0x69
        }

        int selectorOffsettedPin = SENSOR_PIN_OFFSET + i;
        
        #ifdef SENSOR_PIN_TU_COMPENSATION
        if (i == SENSOR_PIN_TU) {
            selectorOffsettedPin = SENSOR_PIN_TU_COMPENSATION;
        }
        #endif
        #ifdef SENSOR_PIN_SU_COMPENSATION
        if (i == SENSOR_PIN_SU) {
            selectorOffsettedPin = SENSOR_PIN_SU_COMPENSATION;
        }
        #endif
        #ifdef SENSOR_PIN_FU_COMPENSATION
        if (i == SENSOR_PIN_FU) {
            selectorOffsettedPin = SENSOR_PIN_FU_COMPENSATION;
        }
        #endif
        #ifdef SENSOR_PIN_MU_COMPENSATION
        if (i == SENSOR_PIN_MU) {
            selectorOffsettedPin = SENSOR_PIN_MU_COMPENSATION;
        }
        #endif
        #ifdef SENSOR_PIN_EU_COMPENSATION
        if (i == SENSOR_PIN_EU) {
            selectorOffsettedPin = SENSOR_PIN_EU_COMPENSATION;
        }
        #endif
        // Because Non-firing contact field on right hand has broken contact on pin D8  
        #ifdef SENSOR_PIN_HP_COMPENSATION
        if (i == SENSOR_PIN_HP) {
            selectorOffsettedPin = SENSOR_PIN_HP_COMPENSATION;
        }
        #endif
        MASTER_SERIAL_NAME.print(F("Enabled on pin: "));
        MASTER_SERIAL_NAME.print(selectorOffsettedPin);
        MASTER_SERIAL_NAME.println(F(""));
        
        initMPUAndDMP(1);
    }
    timeNow = millis(); //Start counting time in milliseconds

    //delay(3000);
//ui.init();
  //ui.update();

}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    //return;
    // if programming failed, don't try to do anything
    //if (!dmpReady)
    //   return;

    // wait for MPU interrupt or extra packet(s) available
    // while (/*!mpuInterrupt &&*/ fifoCount < packetSize)
    //{
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
    // }
    //uint8_t teapotPacket[21] = {'*', 0x99, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0x00, 0x00 , '\r', '\n'};
    handSwitchPrev = timeNow;
    timePrev = timeNow; // the previous time is stored before the actual time read
    timeNow = millis(); // actual time read
    elapsedTime = (timeNow - timePrev);
 
#ifdef USE_DISPLAY

//if(elapsedTime > 10){
    /* Serial.println("elapsedTime time budget:");
    Serial.println(elapsedTime);
    Serial.println("remainingTimeBudget time budget:");
    Serial.println(remainingTimeBudget);
    */
    remainingTimeBudget = ui.update();

    if (elapsedTime - remainingTimeBudget > 0) {
    //  remainingTimeBudget = ui.update();

    // You can do some work here
    // Don't do stuff if you are below your
    // time budget.
   // Serial.println("Remaining time budget:");
    //Serial.println(remainingTimeBudget);
   // delay(remainingTimeBudget);

  }
  //return;
  //}
#endif

#ifdef MASTER_HAND
    loadSlaveHandData();
    gyros[selectedSensor].alreadySentData = false;
    //writePacket();
    loadDataAndSendPacket();
    int currentlySellectedSensor = selectedSensor;
    setOrRotateSelectedGyro(-1);
    //setOrRotateSelectedGyro(2);
    loadDataFromFIFO(true);
#endif

#ifdef SLAVE_HAND
    slaveHandDataRequestHandler();
#endif
    automaticFifoReset();
#ifdef MASTER_HAND
    loadSlaveHandData();
#endif

}