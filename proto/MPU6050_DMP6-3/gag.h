/*
*/

#ifndef _GAG_H_
#define _GAG_H_

#ifdef MEASURE_OFFSETS
#include "definitions.h"
#endif

#include "MPU6050_MPU9250_9Axis_MotionApps41.h"
#include "gag_offsetting.h"

#ifdef USE_DISPLAY
#ifndef USE_DUSPLAY_h
#define USE_DUSPLAY_h
#include "gag_display.h"
#endif
#endif

#ifdef MASTER_HAND
    #ifdef MASTER_BT_SERIAL
        #define MASTER_BT_SERIAL_NAME "GAGGM"
        #include "BluetoothSerial.h"
        #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
            #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
        #endif
        BluetoothSerial SerialBT;
        #define MASTER_SERIAL_NAME SerialBT
    #else
        #define MASTER_SERIAL_NAME Serial
    #endif

    #define MASTER_SERIAL_BAUD 115200

    #define SLAVE_SERIAL_NAME Serial2
    #define SLAVE_SERIAL_BAUD 57600

#else
    // 115200 57600
    #ifdef MASTER_HAND
        #define MASTER_SERIAL_NAME Serial2
        #define MASTER_SERIAL_BAUD 115200
    #endif
#endif

#ifdef SLAVE_HAND
    #define MASTER_SERIAL_NAME Serial
    #define MASTER_SERIAL_BAUD 57600

    #ifdef LIB_SW_SERIAL
        #include <SoftwareSerial.h>
    #endif
    #ifdef LIB_ALT_SW_SERIAL
        #include <AltSoftSerial.h>
    #endif

    //#define RX_MASTER 8
    //#define TX_MASTER 9
    // TODO check if using RX_MASTER=13 or any other SPI pin is faster? 
    #define RX_MASTER 8
    #define TX_MASTER 9
    #ifdef LIB_SW_SERIAL
        SoftwareSerial hc05Master(RX_MASTER, TX_MASTER);
    #endif
    #ifdef LIB_ALT_SW_SERIAL
        AltSoftSerial hc05Master; //(RX_MASTER, TX_MASTER);
    #endif
#endif


// MPU control/status vars
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;   // return status after each device operation (0 = success, !0 = error)
uint8_t packetSizeS = MPU6050_FIFO_PACKET_SIZE; // expected DMP packet size (default is 42 bytes)
uint8_t packetSizeM = MPU9250_FIFO_PACKET_SIZE; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;  // count of all bytes currently in FIFO

// packet structure for InvenSense teapot demo
#ifdef SLAVE_HAND
uint8_t cmdPacket[CMD_PACKET_LENGTH] = {'c', 0, 0, '\r', '\n'};

uint8_t teapotPacket[PACKET_LENGTH] = {'$', 0x99, 0, 0, 0, 0, 0, 0, 0, 0, 0,
#ifdef SEND_ACC
                                       0, 0, 0, 0, 0, 0,
#endif
                                       0x00, 0x00, '\r', '\n'};
#else
uint8_t cmdPacket[CMD_PACKET_LENGTH] = {'C', 0, 0x00, '\r', '\n'};

uint8_t teapotPacket[PACKET_LENGTH] = {'*', 0x99, 0, 0, 0, 0, 0, 0, 0, 0, 0,
#ifdef SEND_ACC
                                       0, 0, 0, 0, 0, 0,
#endif
                                       0x00, 0x00, '\r', '\n'};
#endif

enum Sensor {
    SENSOR_PIN_TU = 0,
    SENSOR_PIN_SU = 1,
    SENSOR_PIN_FU = 2,
    SENSOR_PIN_MU = 3,
    SENSOR_PIN_EU = 4,
    SENSOR_PIN_HP = 5, // hand palm
    SENSOR_PIN_NF = -1,
};

/*
char emptyArr[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
#ifdef SEND_ACC
0, 0, 0, 0, 0, 0,
#endif
0x00, 0x00, 0, 0};
*/
struct Gyro
{
    uint8_t fifoBuffer[FIFO_SIZE];                                      // FIFO storage buffer
    //int AD0_pin;
    //MPU6050 *mpu;
    //MPU6050 *mpuM;
    //MPU9250 *mpuM;
    MPU6050_MPU9250 *mpu;
    // orientation/motion vars
    Quaternion q;        // [w, x, y, z]         quaternion container
   // bool dmpReady = false; // set true if DMP init was successful
    bool hasDataReady = false;
    bool alreadySentData = false;
    long lastResetTime=0;
};

Sensor selectedSensor = SENSOR_PIN_NF;

//Variables
float elapsedTime, timeNow, timePrev, elapsedTimeToSwitch, handSwitchPrev, handSwitchElapsed; //Variables for time control
Gyro gyros[SENSORS_COUNT];

/*
void enableSingleMPU(int sensorToEnable);
int initMPUAndDMP(int attempt);
int setOrRotateSelectedGyro(int i);
*/

#ifdef OLD_RESET
void resetMPUs(int around);
#endif

//void fifoToPacket(byte * fifoBuffer, byte * packet, int selectedSensor);

#ifdef MASTER_HAND
//void sendDataRequest(int selectedSensor);
#endif

volatile int readAlign = 0;
#ifdef SLAVE_HAND 
volatile int readAligned = 0;
#endif
#ifdef MASTER_HAND 
volatile int readAligned = 1;
#endif
volatile float time2, timePrev2;

/*
//void getAccel_Data(MPU6050 * mpuI) {  
void getAccel_Data(MPU6050_MPU9250 * mpu) {
    //MPU6050 mpu = *mpuI;
   // MPU6050_MPU9250 mpu = *mpu;hasDataReady
    mpu->getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384;
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384;
}*/
float Axyz[3];
float Gxyz[3];
float Mxyz[3];

/*
void getMPU9250Data(MPU6050_MPU9250 * mpu);
bool loadDataFromFIFO(bool forceLoad);
void writePacket();
void loadDataAndSendPacket();
*/

#ifdef SLAVE_HAND
//void slaveHandDataRequestHandler();
#endif

#ifdef MASTER_HAND
//void loadSlaveHandData() {
#endif

//int selectSingleMPU(int i);

// start of code for measuring offsets 
#ifdef MEASURE_OFFSETS
//Change this 3 variables if you want to fine tune the skecth to your needs.

//extern bool calibrationDone = false;
#endif // end of code for measuring offsets 

int selectSingleMPU(int i) {
    int selectorOffsettedPin;
    switch(i){
        #ifdef SENSOR_PIN_TU_COMPENSATION
        case SENSOR_PIN_TU:
            selectorOffsettedPin = SENSOR_PIN_TU_COMPENSATION;
        break;
        #endif
        #ifdef SENSOR_PIN_SU_COMPENSATION
        case SENSOR_PIN_SU:
            selectorOffsettedPin = SENSOR_PIN_SU_COMPENSATION;
        break;
        #endif
        #ifdef SENSOR_PIN_FU_COMPENSATION
        case SENSOR_PIN_FU:
            selectorOffsettedPin = SENSOR_PIN_FU_COMPENSATION;
        break;
        #endif
        #ifdef SENSOR_PIN_MU_COMPENSATION
        case SENSOR_PIN_MU:
            selectorOffsettedPin = SENSOR_PIN_MU_COMPENSATION;
        break;
        #endif
        #ifdef SENSOR_PIN_EU_COMPENSATION
        case SENSOR_PIN_EU:
            selectorOffsettedPin = SENSOR_PIN_EU_COMPENSATION;
        break;
        #endif
        #ifdef SENSOR_PIN_HP_COMPENSATION
        case SENSOR_PIN_HP:
            selectorOffsettedPin = SENSOR_PIN_HP_COMPENSATION;
        break;
        #endif
        default:
            selectorOffsettedPin = i + SENSOR_PIN_OFFSET;
    }
    return selectorOffsettedPin;
}

/**
 * It has to happen in 2 loops for the need to avoid having multiple uncertain choices
*/
void enableSingleMPU(int sensorToEnable) {
    int i = 0;
    for (i = 0; i < SENSORS_COUNT; i++) {
        if ( i != sensorToEnable ) {
            digitalWrite(selectSingleMPU(i), HIGH);
        }
    }
    for (i = 0; i < SENSORS_COUNT; i++) {
        if ( i == sensorToEnable ) {
            digitalWrite(selectSingleMPU(i), LOW);     
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
    MPU6050_MPU9250 mpu = *gyros[selectedSensor].mpu;
    mpu.initialize();
    MASTER_SERIAL_NAME.println(F("Testing device connections..."));
    MASTER_SERIAL_NAME.println(mpu.testConnection() ? F("MPU* connection successful") : F("MPU* connection failed"));
    MASTER_SERIAL_NAME.println("testConnection");
    MASTER_SERIAL_NAME.println(mpu.getDeviceID());
    if(selectedSensor != HP) {
        MASTER_SERIAL_NAME.println(F("Initializing DMP..."));
        devStatus = mpu.dmpInitialize();
        MASTER_SERIAL_NAME.print(F("DMP initialized..."));
    } else {
        MASTER_SERIAL_NAME.print(F("Skipping initialing DMP for MPU9250..."));
    }
    // supply your own gyro offsets here for each mpu, scaled for min sensitivity
    // lets ignore this considering we want realtive values anyway
    
    #ifdef MEASURE_OFFSETS
    if(!calibrationDone) {
        measureOffsets(&mpu);
    }
    #endif
    // make sure it worked (returns 0 if so)
    MASTER_SERIAL_NAME.print(F("Enabling DMP... "));
    MASTER_SERIAL_NAME.println(selectedSensor);
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop( ) function knows it's okay to use it
    MASTER_SERIAL_NAME.println(F("DMP ready! Getting packet size..."));
    //gyros[selectedSensor].dmpReady = true;
    MASTER_SERIAL_NAME.print(F("packet size: "));

    if(selectedSensor == HP) {
        MASTER_SERIAL_NAME.print(packetSizeM);
    }else{
        MASTER_SERIAL_NAME.print(packetSizeS);
    }
    MASTER_SERIAL_NAME.println(F(""));

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

void automaticFifoReset() {
    long now = millis();
    int currentlySellectedSensor = selectedSensor;

    for(int i = 0; i < SENSORS_COUNT; i++){
        if(gyros[i].lastResetTime + MIN_TIME_TO_RESET < now 
       /* #ifdef MASTER_HAND
        && gyros[i].hasDataReady
        #endif*/
        ) {
            int selectedNow = setOrRotateSelectedGyro(i);
            if(selectedNow != HP){
                MPU6050_MPU9250 mpu = *gyros[selectedSensor].mpu;
               
                int localFifoCount = mpu.getFIFOCount();
                //GAG_DEBUG_PRINT("localFifoCount");
                //GAG_DEBUG_PRINTLN(localFifoCount);
                if(( localFifoCount >= MAX_FIFO_USAGE_FOR_RESET || 
                    ( gyros[i].lastResetTime + MAX_TIME_TO_RESET < now && 
                    localFifoCount >= MIN_FIFO_USAGE_FOR_RESET) ) ){
                    mpu.resetFIFO();
                    gyros[selectedNow].lastResetTime = now;
                }
            } else{
                MPU6050_MPU9250 mpu = *gyros[selectedSensor].mpu;
                int localFifoCount = mpu.getFIFOCount();
                //GAG_DEBUG_PRINT("localFifoCount");
                //GAG_DEBUG_PRINTLN(localFifoCount);
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
    //DEBUG_PRINTLN("fifoToPacket");
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
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t   mx, my, mz;
    
    uint16_t qI[4];

    //MPU6050_MPU9250 mpu = *mpu;
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
    if(selectedSensor != HP){
        MPU6050_MPU9250 mpu = *gyros[selectedSensor].mpu;

        if(!gyros[selectedSensor].hasDataReady || forceLoad){
            fifoCount = mpu.getFIFOCount();
            uint8_t *fifoBuffer = gyros[selectedSensor].fifoBuffer; // FIFO storage buffer
            int packetSize = packetSizeS; 
            if (fifoCount >= packetSize && fifoCount <= 1024 && fifoCount != 0 ) {
                // wait for correct available data length, should be a VERY short wait
                while (fifoCount >= packetSize) {
                    mpu.getFIFOBytes(fifoBuffer, packetSize);
                    fifoCount -= packetSize;
                }
                //mpu.resetFIFO();
                gyros[selectedSensor].hasDataReady=true;
                return true;
            }
        }
    } else {
        forceLoad = true;
        if(!gyros[selectedSensor].hasDataReady || forceLoad) {
            getMPU9250Data(gyros[selectedSensor].mpu);
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
    //???? MASTER_SERIAL_NAME.write(teapotPacket, PACKET_LENGTH);
    //DEBUG_WRITE_LEN(teapotPacket, PACKET_LENGTH);
    if(!gyros[selectedSensor].alreadySentData && gyros[selectedSensor].hasDataReady) {
        fifoToPacket(fifoBuffer, teapotPacket, selectedSensor);
        MASTER_SERIAL_NAME.write(teapotPacket, PACKET_LENGTH);
        MASTER_SERIAL_NAME.write((byte)0x00);
        //DEBUG_WRITE_LEN(teapotPacket, PACKET_LENGTH);
        //DEBUG_WRITE((byte)0x00);
        gyros[selectedSensor].hasDataReady = false;
        gyros[selectedSensor].alreadySentData = true;
        teapotPacket[PACKET_COUNTER_POSITION]++; // packetCount, loops at 0xFF on purpose
    }
#else
#ifdef SLAVE_HAND
    //DEBUG_PRINT("alreadySentData ");
    //DEBUG_PRINT(gyros[selectedSensor].alreadySentData);
    //DEBUG_PRINT(" hasDataReady ");
    //DEBUG_PRINTLN(gyros[selectedSensor].hasDataReady);
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
        writePacket();       
    }
}

#ifdef SLAVE_HAND

void execCommand(byte ch){
    // {'c', 0, 0, '\r', '\n'};
    cmdPacket[1] = ch;
    switch(ch) {
        case CMD_RESTART_WITH_CALIBRATION_AND_SEND:
        case CMD_RESTART_WITH_CALIBRATION:
            #ifdef MEASURE_OFFSETS
                calibrationDone = false;
            #endif
        case CMD_RESTART:
            setup();
            break;
        default:
            break;
    }
    if(ch == CMD_RESTART_WITH_CALIBRATION_AND_SEND) {
/*   
uint8_t teapotPacket[PACKET_LENGTH] = {'$', 0x99, 0, 0, 0, 0, 0, 0, 0, 0, 0,
#ifdef SEND_ACC
0, 0, 0, 0, 0, 0,
#endif
0x00, 0x00, '\r', '\n'};
#else
*/
        
        teapotPacket[3] = 'g';
        teapotPacket[4] = 'g';
        teapotPacket[5] = gx_offset & 0xFF;
        teapotPacket[6] = gx_offset >> 8;
        teapotPacket[7] = gy_offset & 0xFF;
        teapotPacket[8] = gy_offset >> 8;
        teapotPacket[9] = gz_offset & 0xFF;
        teapotPacket[10] = gz_offset >> 8;
        
        MASTER_SERIAL_NAME.write(teapotPacket, PACKET_LENGTH);
        MASTER_SERIAL_NAME.write((byte)0x00);    
        teapotPacket[PACKET_COUNTER_POSITION]++;
        #ifdef SEND_ACC
            teapotPacket[3] = 'g';
            teapotPacket[4] = 'g';
            teapotPacket[5] = ax_offset & 0xFF;
            teapotPacket[6] = ax_offset >> 8;
            teapotPacket[7] = ay_offset & 0xFF;
            teapotPacket[8] = ay_offset >> 8;
            teapotPacket[9] = az_offset & 0xFF;
            teapotPacket[10] = az_offset >> 8;
            
            MASTER_SERIAL_NAME.write(teapotPacket, PACKET_LENGTH);
            MASTER_SERIAL_NAME.write((byte)0x00);    
            teapotPacket[PACKET_COUNTER_POSITION]++;
        #endif
        
        return;
    }
    MASTER_SERIAL_NAME.write(cmdPacket, CMD_PACKET_LENGTH);
    MASTER_SERIAL_NAME.write((byte)0x00);
}

void slaveHandDataRequestHandler() {
    int limit = REPEAT_SLAVE_HAND_READ_LIMIT;
    readAlign = 0;
    readAligned = 0;
    //DEBUG_PRINTLN("slaveHandDataRequestHandler");

    while(limit > 0) {
        int ch = MASTER_SERIAL_NAME.read();
        if (ch != -1) {
         
            if(readAlign<1) {
                if(ch == '$') {
                    readAlign=1;
                    //DEBUG_PRINTLN("$ - readAlign=1");
                }
            } else {
                //if(ch >= 0 && ch < SENSORS_COUNT) {
                if((ch >= 0 && ch < SENSORS_COUNT) || (ch >= '0' && ch <= '6' )) {    
                    if(ch >= '0' && ch <= '6' ){
                        ch = ch - 48;
                    }
                    //DEBUG_PRINTLN("al");
                    readAligned = 1;
                    readAlign=0;
                    setOrRotateSelectedGyro(ch);

                    gyros[selectedSensor].alreadySentData = false;
                    writePacket();
                    loadDataAndSendPacket();
                    int currentlySellectedSensor = selectedSensor;
                    setOrRotateSelectedGyro(-1);
                    loadDataFromFIFO(true);
                    break;
                } else if (ch == 'c') {
                    // deal with command packet ..
                    readAligned = 1;
                    readAlign=0;
                    while(limit > 0) {
                        int chc = MASTER_SERIAL_NAME.read();
                        if (chc != -1) {
                            execCommand((char)chc);
                        }
                        limit--;
                     }
                    break;
                } else {
                    //DEBUG_PRINTLN("readAlign=0");
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

#endif