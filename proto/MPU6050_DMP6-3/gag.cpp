/*

*/

#include "gag.h"

int selectSingleMPU(int selectorOffsettedPinOrig, int i) {
    int selectorOffsettedPin = selectorOffsettedPinOrig;
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

}

void enableSingleMPU(int sensorToEnable) {
    for (int i = 0; i < SENSORS_COUNT; i++) {
        int selectorOffsettedPin = selectSingleMPU(SENSOR_PIN_OFFSET + i, i);
        if ( i != sensorToEnable ) {
            digitalWrite(selectorOffsettedPin, HIGH);
        }
    }

    for (int i = 0; i < SENSORS_COUNT; i++) {
        int selectorOffsettedPin = selectSingleMPU(SENSOR_PIN_OFFSET + i, i);
        
        if ( i == sensorToEnable ) {
            digitalWrite(selectorOffsettedPin, LOW);     
        }
    }
}



int initMPUAndDMP(int attempt) {
    if (attempt <= 0) {
        return 0;
    }
// initialize device
#ifdef MASTER_SERIAL_NAME
    MASTER_SERIAL_NAME.println(F("USB: Initializing I2C devices..."));
#endif
    if(selectedSensor == HP) {
        //MPU9250 mpu = *gyros[selectedSensor].mpuM;
        MPU6050_MPU9250 mpu = *gyros[selectedSensor].mpu;
        //mpu.initialize();
        mpu.initialize();

    //#ifdef USE_USB
        MASTER_SERIAL_NAME.println(F("Testing device connections..."));
        MASTER_SERIAL_NAME.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

        MASTER_SERIAL_NAME.println("testConnection");
        MASTER_SERIAL_NAME.println(mpu.getDeviceID());
    
        // wait for ready
        MASTER_SERIAL_NAME.println(F("Send any character to begin DMP programming and demo: "));
        // load and configure the DMP
        MASTER_SERIAL_NAME.println(F("Initializing DMP..."));
    //#endif
        //devStatus = mpu.dmpInitialize();
        //devStatus = mpu.dmpInitialize();
        MASTER_SERIAL_NAME.print(F("DMP initialized..."));

        // supply your own gyro offsets here for each mpu, scaled for min sensitivity
        // lets ignore this considering we want realtive values anyway
        //mpu.setXGyroOffset(220);
        //mpu.setYGyroOffset(76);
        //mpu.setZGyroOffset(-85);
        //mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
        #ifdef MEASURE_OFFSETS
        measureOffsets(&mpu);
        #endif
        // make sure it worked (returns 0 if so)
        //if (devStatus == 0) {
        if (true) {

    //#ifdef MASTER_SERIAL_NAME
            MASTER_SERIAL_NAME.print(F("Enabling DMP... "));
            MASTER_SERIAL_NAME.println(selectedSensor);
            mpu.setDMPEnabled(true);

            // set our DMP Ready flag so the main loop( ) function knows it's okay to use it
            MASTER_SERIAL_NAME.println(F("DMP ready! Getting packet size..."));
            //gyros[selectedSensor].dmpReady = true;
            MASTER_SERIAL_NAME.print(F("packet size: "));
            MASTER_SERIAL_NAME.print(packetSizeM);
            MASTER_SERIAL_NAME.println(F(""));
    //#endif
        } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //#ifdef USE_USB
            MASTER_SERIAL_NAME.print(F("DMP Initialization failed (code "));
            MASTER_SERIAL_NAME.print(devStatus);
            MASTER_SERIAL_NAME.println(F(")"));
    //#endif
            initMPUAndDMP(attempt - 1);
        }
    } else {
        //MPU6050 mpu = *gyros[selectedSensor].mpu;
        MPU6050_MPU9250 mpu = *gyros[selectedSensor].mpu;
        mpu.initialize();

    // TODO
    //#ifdef MASTER_SERIAL_NAME
        MASTER_SERIAL_NAME.println(F("Testing device connections..."));
        MASTER_SERIAL_NAME.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
        MASTER_SERIAL_NAME.println("testConnection");
        MASTER_SERIAL_NAME.println(mpu.getDeviceID());
        // wait for ready
        MASTER_SERIAL_NAME.println(F("\nSend any character to begin DMP programming and demo: "));
        // load and configure the DMP
        MASTER_SERIAL_NAME.println(F("Initializing DMP..."));
    //#endif
        devStatus = mpu.dmpInitialize();
        MASTER_SERIAL_NAME.print(F("DMP initialized..."));

        // supply your own gyro offsets here for each mpu, scaled for min sensitivity
        // lets ignore this considering we want realtive values anyway
        //mpu.setXGyroOffset(220);
        //mpu.setYGyroOffset(76);
        //mpu.setZGyroOffset(-85);
        //mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
        #ifdef MEASURE_OFFSETS
        measureOffsets(&mpu);
        #endif
        // make sure it worked (returns 0 if so)
        //if (devStatus == 0) {
        if (true) {
    // turn on the DMP, now that it's ready
    // initialize device
    //#ifdef MASTER_SERIAL_NAME 
            MASTER_SERIAL_NAME.print(F("Enabling DMP... "));
            MASTER_SERIAL_NAME.println(selectedSensor);
            mpu.setDMPEnabled(true);

            // set our DMP Ready flag so the main loop( ) function knows it's okay to use it
            MASTER_SERIAL_NAME.println(F("DMP ready! Getting packet size..."));
            //gyros[selectedSensor].dmpReady = true;
            MASTER_SERIAL_NAME.print(F("packet size: "));
            MASTER_SERIAL_NAME.print(packetSizeS);
            MASTER_SERIAL_NAME.println(F(""));
    //#endif
        } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    //#ifdef MASTER_SERIAL_NAME
            MASTER_SERIAL_NAME.print(F("DMP Initialization failed (code "));
            MASTER_SERIAL_NAME.print(devStatus);
            MASTER_SERIAL_NAME.println(F(")"));
    //#endif
            initMPUAndDMP(attempt - 1);
        }
    }
    return 0;
}

int setOrRotateSelectedGyro(int i) {
    if (i == -1) {
        i = selectedSensor + 1;
    }
    if (i > LAST_SENSOR) {
        i = FIRST_SENSOR;
    }
    selectedSensor = (Sensor)i;
    enableSingleMPU(selectedSensor);
    return i;
}


#ifdef OLD_RESET
void resetMPUs(int around) {
    // reset n after (TODO without current?)
    if (around > 0) {
        for (int i = 0; i < around; i++) {
            //MPU6050 mpu = *gyros[selectedSensor].mpu;
            MPU6050_MPU9250 mpu = *gyros[selectedSensor].mpu;

            mpu.resetFIFO();
            setOrRotateSelectedGyro(-1);
        }
        // rotate to start position
        for (int i = around; i < SENSORS_COUNT; i++) {
            setOrRotateSelectedGyro(-1);
        }
    }
    else
    {
        // reset n before current (TODO without current?)
        // rotate to position where reset should start
        int i = 0;
        for (; i < SENSORS_COUNT + around; i++) {
            setOrRotateSelectedGyro(-1);
        }
        for (; i < SENSORS_COUNT; i++) {
            //MPU6050 mpu = *gyros[selectedSensor].mpu;
            MPU6050_MPU9250 mpu = *gyros[selectedSensor].mpu;
            mpu.resetFIFO();
            setOrRotateSelectedGyro(-1);
        }
    }
}
#endif


void automaticFifoReset() {
    long now = millis();
    int currentlySellectedSensor = selectedSensor;

    for(int i = 0; i < SENSORS_COUNT; i++){
        if(gyros[i].lastResetTime + MIN_TIME_TO_RESET < now 
        #ifdef RIGHT_HAND
        && gyros[i].hasDataReady
        #endif
        ) {
            int selectedNow = setOrRotateSelectedGyro(i);
            if(selectedNow != 5){
                //MPU6050 mpu = *gyros[selectedNow].mpu;
                MPU6050_MPU9250 mpu = *gyros[selectedSensor].mpu;
               
                int localFifoCount = mpu.getFIFOCount();
                //Serial.println("localFifoCount");
                //Serial.println(localFifoCount);
                if(( localFifoCount >= MAX_FIFO_USAGE_FOR_RESET || 
                    ( gyros[i].lastResetTime + MAX_TIME_TO_RESET < now && 
                    localFifoCount >= MIN_FIFO_USAGE_FOR_RESET) ) ){
                    mpu.resetFIFO();
                    gyros[selectedNow].lastResetTime = now;
                }
            } else{
                //MPU9250 mpu = *gyros[selectedNow].mpuM;
                MPU6050_MPU9250 mpu = *gyros[selectedSensor].mpu;

                int localFifoCount = mpu.getFIFOCount();
                //Serial.println("localFifoCount");
                //Serial.println(localFifoCount);
                if(( localFifoCount >= MAX_FIFO_USAGE_FOR_RESET || 
                    ( gyros[i].lastResetTime + MAX_TIME_TO_RESET < now && 
                    localFifoCount >= MIN_FIFO_USAGE_FOR_RESET) ) ){
                    mpu.resetFIFO();
                    gyros[selectedNow].lastResetTime = now;
                }
            }
       }
    }
    setOrRotateSelectedGyro(currentlySellectedSensor);
}

void fifoToPacket(byte * fifoBuffer, byte * packet, int selectedSensor) {
    DEBUG_PRINTLN("fifoToPacket");
    packet[2] = selectedSensor;
    packet[3] = fifoBuffer[0];
    packet[4] = fifoBuffer[1];
    packet[5] = fifoBuffer[4];
    packet[6] = fifoBuffer[5];
    packet[7] = fifoBuffer[8];
    packet[8] = fifoBuffer[9];
    packet[9] = fifoBuffer[12];
    packet[10] = fifoBuffer[13];
}

#ifdef MASTER_HAND
void sendDataRequest(int selectedSensor) {
    SLAVE_SERIAL_NAME.write('$');
    SLAVE_SERIAL_NAME.write(selectedSensor);
    SLAVE_SERIAL_NAME.write((byte)0x00); // 0x00 fails to compile
}
#endif


void getMPU9250Data(MPU6050_MPU9250 * mpu) {
    // uint8_t buffer_m[6];
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t   mx, my, mz;
    
    //float q[4];
    uint16_t qI[4];

    //MPU6050_MPU9250 mpu = *gyros[selectedSensor].mpu;
    //MPU9250 mpu = *mpuI;
    //MPU6050_MPU9250 mpu = *mpu;
    //mpu->get
    mpu->getMotion9(&ax, &ay, &az, &gx, &gy, &gz,&mx, &my, &mz);

    // TODO fix
    // 8192 16384 32768 65536
    /*
    Gxyz[0] += (float) gx / (65536) ;
    Gxyz[1] += (float) gy / (65536) ;
    Gxyz[2] += (float) gz / (65536) ;

    */
    #define offsetDown 65536
    
    Gxyz[0] += (float) gx / (offsetDown) ;
    Gxyz[1] += (float) gy / (offsetDown) ;
    Gxyz[2] += (float) gz / (offsetDown) ;

    /*
    Gxyz[0] = (float) gx / (offsetUp) ;
    Gxyz[1] = (float) gy / (offsetUp) ;
    Gxyz[2] = (float) gz / (offsetUp) ;
    */
    #define  offsetDownQ 16

    float yaw = Gxyz[2] / offsetDownQ;
    float pitch = Gxyz[1]/ offsetDownQ;
    float roll = Gxyz[0]/ offsetDownQ;
    /*
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    */
    #define partition 0.5
    float cy = cos(yaw * partition );
    float sy = sin(yaw * partition );
    float cp = cos(pitch * partition );
    float sp = sin(pitch * partition);
    float cr = cos(roll * partition );
    float sr = sin(roll * partition );

    /*
    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
    
    #define offsetUp 1024
    
    qI[0] = (uint16_t)(q[0]*offsetUp);
    qI[1] = (uint16_t)(q[1]*offsetUp);
    qI[2] = (uint16_t)(q[2]*offsetUp);
    qI[3] = (uint16_t)(q[3]*offsetUp);
*/
    #define offsetUp 1024
    
    qI[0] = (uint16_t)((cy * cp * cr + sy * sp * sr)*offsetUp);
    qI[1] = (uint16_t)((cy * cp * sr - sy * sp * cr)*offsetUp);
    qI[2] = (uint16_t)((sy * cp * sr + cy * sp * cr)*offsetUp);
    qI[3] = (uint16_t)((sy * cp * cr - cy * sp * sr)*offsetUp);

  /*
    qI[0] += (uint16_t)(q[0]*offsetDown);
    qI[1] += (uint16_t)(q[1]*offsetDown);
    qI[2] += (uint16_t)(q[2]*offsetDown);
    qI[3] += (uint16_t)(q[3]*offsetDown);
    */
    uint8_t *fifoBuffer = gyros[selectedSensor].fifoBuffer; // FIFO storage buffer*/

    fifoBuffer[1] = qI[0] & 0xFF;
    fifoBuffer[0] = qI[0]>> 8;
    
    fifoBuffer[5] = qI[1] & 0xFF;
    fifoBuffer[4] = qI[1]>> 8;
    
    fifoBuffer[9] = qI[2] & 0xFF;
    fifoBuffer[8] = qI[2] >> 8;
    

    fifoBuffer[13] = qI[3] & 0xFF;
    fifoBuffer[12] = qI[3] >> 8;
}

bool loadDataFromFIFO(bool forceLoad) {
    if(selectedSensor != LAST_SENSOR){
        //MPU6050 mpu = *gyros[selectedSensor].mpu;
        MPU6050_MPU9250 mpu = *gyros[selectedSensor].mpu;

        if(!gyros[selectedSensor].hasDataReady || forceLoad){
            fifoCount = mpu.getFIFOCount();
           // Serial.print("fifoCount:2 ");
            ///Serial.println(fifoCount);
            uint8_t *fifoBuffer = gyros[selectedSensor].fifoBuffer; // FIFO storage buffer
            int packetSize = packetSizeS; 
            if (fifoCount >= packetSize && fifoCount <= 1024 && fifoCount != 0 ) {
                // wait for correct available data length, should be a VERY short wait
                while (fifoCount >= packetSize) {
                    mpu.getFIFOBytes(fifoBuffer, packetSize);
                    fifoCount -= packetSize;
                }
                
                /*for(int ii = 0;  ii < packetSize;  ii++) {
                    Serial.print(fifoBuffer[ii]);
                    Serial.print(" ");
                }
                Serial.print("OK");
                //mpu.resetFIFO();*/
                gyros[selectedSensor].hasDataReady=true;
                return true;
            }
        }
    } else {
        //MPU9250 mpu = *gyros[selectedSensor].mpuM;
        //MPU6050_MPU9250 mpu = *gyros[selectedSensor].mpu;

        forceLoad = true;
        if(!gyros[selectedSensor].hasDataReady || forceLoad) {
            //getMPU9250Data(gyros[selectedSensor].mpuM);
            getMPU9250Data(gyros[selectedSensor].mpu);
            //gyros[selectedSensor].mpu->dmpGet6AxisQuaternion()
            gyros[selectedSensor].hasDataReady=true;
            gyros[selectedSensor].alreadySentData=false;
            return true;
        }
    }
    return false;
}

void writePacket() {
    uint8_t *fifoBuffer = gyros[selectedSensor].fifoBuffer; // FIFO storage buffer

#ifdef MASTER_HAND
    MASTER_SERIAL_NAME.write(teapotPacket, PACKET_LENGTH);

    if(!gyros[selectedSensor].alreadySentData && gyros[selectedSensor].hasDataReady) {
        fifoToPacket(fifoBuffer, teapotPacket, selectedSensor);
        MASTER_SERIAL_NAME.write(teapotPacket, PACKET_LENGTH);
        MASTER_SERIAL_NAME.write((byte)0x00);
        gyros[selectedSensor].hasDataReady = false;
        gyros[selectedSensor].alreadySentData = true;
        teapotPacket[PACKET_COUNTER_POSITION]++; // packetCount, loops at 0xFF on purpose
    }
#else
#ifdef SLAVE_HAND
    DEBUG_PRINT("alreadySentData ");
    DEBUG_PRINT(gyros[selectedSensor].alreadySentData);
    DEBUG_PRINT(" hasDataReady ");
    DEBUG_PRINTLN(gyros[selectedSensor].hasDataReady);
    if(!gyros[selectedSensor].alreadySentData && gyros[selectedSensor].hasDataReady){
        fifoToPacket(fifoBuffer, teapotPacket, selectedSensor);
        MASTER_SERIAL_NAME.write(teapotPacket, PACKET_LENGTH);
        MASTER_SERIAL_NAME.write(0x00);
        teapotPacket[PACKET_COUNTER_POSITION]++; // packetCount, loops at 0xFF on purpose
        gyros[selectedSensor].hasDataReady = false;
        gyros[selectedSensor].alreadySentData = true;
    }
    readAlign = 0;
    readAligned = 0;
#endif
#ifdef MASTER_HAND
    if(!gyros[selectedSensor].alreadySentData && gyros[selectedSensor].hasDataReady) {
        fifoToPacket(fifoBuffer, teapotPacket, selectedSensor);
        MASTER_SERIAL_NAME.write(teapotPacket, PACKET_LENGTH);
        MASTER_SERIAL_NAME.write((byte)0x00);
        gyros[selectedSensor].hasDataReady = false;
        gyros[selectedSensor].alreadySentData = true;
        teapotPacket[PACKET_COUNTER_POSITION]++; // packetCount, loops at 0xFF on purpose
    }
#endif        
#endif

}

void loadDataAndSendPacket() {
    loadDataFromFIFO(false);
    if(gyros[selectedSensor].hasDataReady) {
/*
#ifdef USE_BT
        if (hc05.availableForWrite()) {
            for (int i = 0; i < PACKET_LENGTH; i++) {
                hc05.print((char)teapotPacket[i]);
            }
        }
#endif
*/
//#ifdef USE_USB
    writePacket();       
//#endif
    }
}

//#ifdef RIGHT_HAND_SLAVE     
#ifdef SLAVE_HAND
void slaveHandDataRequestHandler() {
    int limit = REPEAT_SLAVE_HAND_READ_LIMIT;
    readAlign = 0;
    readAligned = 0;
    DEBUG_PRINTLN("slaveHandDataRequestHandler");

    while(limit > 0) {
        int ch = MASTER_SERIAL_NAME.read();
        if (ch != -1) {
         
            if(readAlign<1) {
                if(ch == '$') {
                    readAlign=1;
                    DEBUG_PRINTLN("$ - readAlign=1");
                }
            } else {
                //if(ch >= 0 && ch < SENSORS_COUNT) {
                if((ch >= 0 && ch < SENSORS_COUNT) || (ch >= '0' && ch <= '6' )) {    
                    if(ch >= '0' && ch <= '6' ){
                        ch = ch - 48;
                    }
                    DEBUG_PRINTLN("al");
                    readAligned = 1;
                    readAlign=0;//++;
                    setOrRotateSelectedGyro(ch);

                    if(readAligned == 1){
                        gyros[selectedSensor].alreadySentData = false;
                        writePacket();
                        loadDataAndSendPacket();
                        int currentlySellectedSensor = selectedSensor;
                        setOrRotateSelectedGyro(-1);
                        loadDataFromFIFO(true);
                        // setOrRotateSelectedGyro(currentlySellectedSensor);
                    }

                    break;
                }else {
                    DEBUG_PRINTLN("readAlign=0");
                    readAlign=0;
                }
            }    
            limit--;
        }
    }
}
#endif

#ifdef MASTER_HAND
void loadSlaveHandData() {
    int limit = REPEAT_MASTER_HAND_READ_LIMIT;
    
    while(--limit>0){ 
        sendDataRequest(selectedSensor);
        handSwitchElapsed = 0;
        time2 = millis();
        timePrev2 = 0;
        int sentCharCounter = 0;
        int sentPacketCharCounter = 0;
        int align = 0;
        int aligned = 0;
        int endOfPacketAlign=0;
        
        while (true) {
            timePrev2 = time2;
            time2 = millis();
            handSwitchElapsed += (time2 - timePrev2);
            if (/*sentCharCounter > MAX_HAND_SWITCH_CHARS || */ 
                handSwitchElapsed > MAX_HAND_SWITCH_TIME ||
                sentPacketCharCounter > PACKET_LENGTH || endOfPacketAlign == 2) {
                handSwitchElapsed = 0;
                sentCharCounter = 0;
                // TODO fix failing aligning differently then with sending flag character '+' for distinguishing packet end
                MASTER_SERIAL_NAME.write((byte)0x00);
                // TODO find out how many packets are lost
                limit =-1;
                break;
            }

            int ch = SLAVE_SERIAL_NAME.read();

            if (ch != -1) {
                if(ch == '$') {
                    align++;
                    sentPacketCharCounter++;
                }
                if(ch == 0x99) {
                    align++;
                    sentPacketCharCounter++;
                }
                if(align > 1) {
                    aligned = 1;
                }
                if(sentPacketCharCounter>0) {
                    MASTER_SERIAL_NAME.write(ch);
                    sentPacketCharCounter++;
                }
                if(ch == '\r') {
                    endOfPacketAlign=1;
                }
                if(ch == '\n' && endOfPacketAlign==1) {
                    endOfPacketAlign=2;
                }
    
            }
            
        }
    }
}
#endif
