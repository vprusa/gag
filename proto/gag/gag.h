/**
 * TODO's for this file  
 * - slip to gag_master.h and gag_agent.h
 * - split custom command execution
 * - refactor MASTER_SERIAL, GAG_DEBUG_PRINT[F|LN], GAG_DEBUG_PRINT[F|LN]
 * - 
*/

#ifndef _GAG_H_
#define _GAG_H_
#include "definitions.h"

#define GAG_DEBUG_PRINT(x) Serial.print(x)
#define GAG_DEBUG_PRINTF(x, y) Serial.print(x, y)
#define GAG_DEBUG_PRINTLN(x) Serial.println(x)
#define GAG_DEBUG_PRINTLNF(x, y) Serial.println(x, y)

#include "MPU6050_MPU9150_9Axis_MotionApps41.h"
// #include "MPU6050_9Axis_MotionApps41.h"
#ifdef MEASURE_OFFSETS
#include "gag_offsetting.h"
#endif
// #define USE_BT_GATT_SERIAL

#ifdef USE_DISPLAY
#include "gag_display.h"
#endif

#ifdef MASTER_HAND
    #ifdef MASTER_BT_SERIAL
        #define MASTER_BT_SERIAL_NAME "GAGGM"
        #ifdef USE_BT_GATT_SERIAL
            #include "gag_bt_gatt_serial.h"
            #define MASTER_SERIAL_NAME SerialBT
            BluetoothSerial SerialBT;
        #else
            #include "BluetoothSerial.h"
            #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
                #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
            #endif
            BluetoothSerial SerialBT;
            #define MASTER_SERIAL_NAME SerialBT
        #endif
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

    // #define LIB_SW_SERIAL
    // #define LIB_ALT_SW_SERIAL

    #ifdef LIB_SW_SERIAL
        #include <SoftwareSerial.h>
    #endif
    #ifdef LIB_ALT_SW_SERIAL
        #include <AltSoftSerial.h>
    #endif
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

// TODO pass const* cmdPacket as argument because cmdPacket is a singleton per device?
void execCommand();
void setupSensors();
void enableSingleMPU(uint8_t sensorToEnable);
uint8_t selectSingleMPU(uint8_t sensorToEnable);
uint8_t initMPUAndDMP(uint8_t attempt, uint8_t i);

// MPU control/status vars
uint8_t devStatus;   // return status after each device operation (0 = success, !0 = error)
uint8_t packetSizeS = MPU6050_FIFO_PACKET_SIZE; // expected DMP packet size (default is 42 bytes)
uint8_t packetSizeM = MPU9150_FIFO_PACKET_SIZE; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;  // count of all bytes currently in FIFO

#define SEND_ACC
#ifdef SLAVE_HAND
uint8_t cmdPacket[CMD_PACKET_LENGTH] = {'c', 0, 0, 0, 0, 0, 0, '\r', '\n'};

uint8_t dataPacket[PACKET_LENGTH] = {'$', 0x99, 0, 0, 0, 0, 0, 0, 0, 0, 0,
#ifdef SEND_ACC
                                       0, 0, 0, 0, 0, 0,
#endif
                                       0x00, 0x00, '\r', '\n'};
#else
uint8_t cmdPacket[CMD_PACKET_LENGTH] = {'C', 0, 0, 0, 0, 0, 0, '\r', '\n'};

uint8_t dataPacket[PACKET_LENGTH] = {'*', 0x99, 0,  0, 0,  0, 0,  0, 0,  0, 0,
#ifdef SEND_ACC
                                       0, 0, 0, 0, 0, 0,
#endif
                                       0x00, 0x00, '\r', '\n'};
#endif



#ifdef USE_BT_GATT_SERIAL
ServerCallbacks::ServerCallbacks(void){
    GAG_DEBUG_PRINTLN("ServerCallbacks");
    //MASTER_SERIAL_NAME.deviceConnected = true;
}

ServerCallbacks::~ServerCallbacks(void){
    GAG_DEBUG_PRINTLN("~ServerCallbacks");
}
extern bool deviceConnected;
void ServerCallbacks::onConnect(BLEServer* pServer) {
    GAG_DEBUG_PRINTLN("onConnect");
    //MASTER_SERIAL_NAME.deviceConnected = true;

    deviceConnected = true;
};

void ServerCallbacks::onDisconnect(BLEServer* pServer) {
    GAG_DEBUG_PRINTLN("onDisconnect");
    // MASTER_SERIAL_NAME.deviceConnected = false;
    deviceConnected = false;
    // Try to restart advertising to allow reconnection
    // Restart advertising to allow reconnection
    // BLEAdvertising *pAdvertising = pServer->getAdvertising();
    // pAdvertising->start();

    //BLEDevice::deinit();  // Forcefully reset BLE
    //delay(100);           // Small delay before restarting
    //BLEDevice::init("GAGGM");

    BLEDevice::deinit();
    delay(100);
    BLEDevice::init("GAGGM");

    pServer->getAdvertising()->start();
    GAG_DEBUG_PRINTLN("Restarting BLE Advertising...");
}


CharCallbacks::CharCallbacks(void){
    GAG_DEBUG_PRINTLN("CharCallbacks");
}

CharCallbacks::~CharCallbacks(void){
    GAG_DEBUG_PRINTLN("~CharCallbacks");
}

// because this is called assynchrnously i created buffer for storing cmdPackets
void CharCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    GAG_DEBUG_PRINTLN("onWrite");
    std::string rxValue = pCharacteristic->getValue();
    // timeNow = micros();
    
    if(rxValue.length() > 0) {
        const char * rxStr = rxValue.c_str(); 
        GAG_DEBUG_PRINTLN(rxStr);
        // TODO check if already in use?
        // cmdPacket[1] = rxStr[1]; // cmd
        // cmdPacket[2] = rxStr[2]; // type 
        // cmdPacket[3] = rxStr[3]; // specification
        // cmdPacket[4] = rxStr[4]; // value1
        // cmdPacket[5] = rxStr[5]; // value2

        GAG_DEBUG_PRINT("SerialBT.serialBufferWriteIndex ");
        GAG_DEBUG_PRINTLN(SerialBT.serialBufferWriteIndex);

        GAG_DEBUG_PRINT("SerialBT.serialBufferReadIndex ");
        GAG_DEBUG_PRINTLN(SerialBT.serialBufferReadIndex);

        for(char i = 0; i < CMD_PACKET_LENGTH; i++) {
            SerialBT.serialBuffer[SerialBT.serialBufferWriteIndex][i]=rxStr[i];
            if(SerialBT.serialBuffer[SerialBT.serialBufferWriteIndex][i] == -1){
                SerialBT.serialBuffer[SerialBT.serialBufferWriteIndex][i] = 0;
            }
        }
        // GAG_DEBUG_PRINTLN(SerialBT.serialBuffer[SerialBT.serialBufferWriteIndex]);
        GAG_DEBUG_PRINTLN(SerialBT.serialBuffer[SerialBT.serialBufferWriteIndex][0]);
        // GAG_DEBUG_PRINTLN(SerialBT.serialBuffer[SerialBT.serialBufferWriteIndex][0]);

        if(++SerialBT.serialBufferWriteIndex >= BT_SERIAL_BUFFER_SIZE){
            SerialBT.serialBufferWriteIndex=0;}


        //if(SerialBT.serialBufferWriteIndex == SerialBT.serialBufferReadIndex) {
            // if(++SerialBT.serialBufferWriteIndex >= BT_SERIAL_BUFFER_SIZE){
                // SerialBT.serialBufferWriteIndex=0;}
        //}

        GAG_DEBUG_PRINTLN("Buffer: ");
        for(char i =0; i<BT_SERIAL_BUFFER_SIZE; i++ ) {
            for(char ii =0; ii<CMD_PACKET_LENGTH; ii++ ) {
                GAG_DEBUG_PRINT(SerialBT.serialBuffer[i][ii]);
                GAG_DEBUG_PRINT(" ");
            }
            GAG_DEBUG_PRINTLN("");
        }
        //execCommand();
        // for (int i = 0; i < rxValue.length(); i++) {
            // SLAVE_SERIAL_NAME.print(rxValue[i]);
            // GAG_DEBUG_PRINT(reinterpret_cast<uint8_t>([i]));
        // }
    }
};
#endif

enum Sensor {
    SENSOR_PIN_TU = 0,
    SENSOR_PIN_SU = 1,
    SENSOR_PIN_FU = 2,
    SENSOR_PIN_MU = 3,
    SENSOR_PIN_EU = 4,
    SENSOR_PIN_HG = 5, // hand palm
    SENSOR_PIN_HP = 6, // hand palm MPU6050
    SENSOR_PIN_NF = -1,
};

struct Gyro {
    uint8_t fifoBuffer[FIFO_SIZE]; // FIFO storage buffer
    MPU6050_MPU9150 *mpu;
    // orientation/motion vars
    Quaternion *q;        // [w, x, y, z]         quaternion container
    //#define TEST_CALIB
    #ifdef TEST_CALIB
    #define TEST_CALIB_QUAT_BUFFER 10
    Quaternion queue[TEST_CALIB_QUAT_BUFFER];
    uint8_t queueIndex = 0;
    #endif
    bool debugDiff = false;
    bool hasDataReady = false;
    bool alreadySentData = false;
    long lastResetTime=0;
};

Sensor selectedSensor = SENSOR_PIN_NF;

//Variables
long elapsedTime, timeNow, timePrev, elapsedTimeToSwitch, handSwitchPrev, handSwitchElapsed; //Variables for time control
Gyro gyros[SENSORS_COUNT];

int8_t readAlign = 0;
volatile long time2, timePrev2;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

#ifdef MASTER_HAND
    extern bool useSlaveHand;
#endif

#ifdef SET_OFFSETS
    // cmd for setting sensors 3 gyro's x-axis offset O3gx-1750
    // exec calibration for sensor a3..50
    // ax ay az gx gy gz
    int16_t sensorsOffsets[7][6] = {
        // T
        // {-2772,479,2063,119,-49,120},
        {-2772,479,2063,119,-49,120},
        // I
        // {-3678,1461,1496,46,-43,-3},
        {-3678,1461,1496,46,-43,-3},
        // M
        // {507,1346,1743,20,26,-19},
        {507,1346,1743,20,26,-19},
        // L
        //{-1397,525,919,-1295,2057,-509},
        //{-1397,525,919,-3000,2057,-509},
        //{-1448, 453, 1111, -1379,1980,-514},
        //{-4140,444,1180,-1208,1774,-1000}, // 469
        // {1578,356,1180,-1630,1975, -200}, // 469
        //{-1584, 495, 1004, -1326, 2093, -495},
        // !!! One sensors's axis is broken... TODO replace sensor ...
        // Sensor readings with offsets:	-190	195	16692	-5	5	-1
        //Your offsets:	-1680	1199	1546	24	-11	0
        // {199, -379, 1234, -168, -264, -637},

        // Sensor readings with offsets:	-744	1000	12955	0	0	49
        // Your offsets:	199	-379	1234	-168	-264	-637
        // {-1680, 1199, 1546, -24, 11, -0},
        // Sensor readings with offsets:	-520	-384	16725	-5	5	1
        // Your offsets:	-3208	-1979	1487	47	-22	0

        {-3208, -1979, 1487, 47, -22, 0},
        // R
        //        {-1611, 466, 1081, -1468, 2020, -497},
        //  {-1650,1000,-500,0,0,0}, // 469
        //{19440,-13312,2176,280,-144,5144}, // 469
        //{0,0,0,0,0,0},
        {-1802,1287,1493,35,-19,6},

        // WG unknown        
        // {-107, -183, 539, -13, -72, -588},
        // Sensor readings with offsets:	287	701	17104	-22	10	5
// Your offsets:	1680	1653	1113	62	-23	-1
        // {1680,1653, 1113, 62, -23, -1},
// Sensor readings with offsets:	280	741	17068	-22	10	5
// Your offsets:	1659	1620	1108	62	-23	-1
        {1659, 1620, 1108, 62, -23, -1},


        // W
        //{0,0,0,0,0,0}};
        // {500,500,500,0,0,0}};
        //{0,0,0,0,0,0}
        //{ -158, -471, 1159, -181, -719, -926}
        //{-27, -423, 1154, -227, -504, -1113}
        // {-366, -1411, 4095, -1225, -2542, -3466}
        // {-127, -263, 1133, -511, -429, -1933}
        // {-107, -183, 539, -13, -72, -588}
        // {-123, -224, 707, -70, -85, -1441}
        // Sensor readings with offsets:	679	67	10904	-33	3	1
// Your offsets:	-104	-6	849	8	0	-32
        // {-104,-6,849,8,0,-32}

// Sensor readings with offsets:	771	483	12423	-33	4	1
// Your offsets:	-130	-53	808	60	-2	0
        {-130, -53, 808, 60, -2, 0}


        // Your offsets:	-123	-224	707	-70	-85	-1441
    };
#endif

// start of code for measuring offsets 
#ifdef MEASURE_OFFSETS
//Change this 3 variables if you want to fine tune the skecth to your needs.

// extern bool calibrationDone = false;calibrationDone
#endif // end of code for measuring offsets 

uint8_t selectSingleMPU(uint8_t i) {
    uint8_t selectorOffsettedPin;
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
        #ifdef SENSOR_PIN_HG_COMPENSATION
        case SENSOR_PIN_HG:
            selectorOffsettedPin = SENSOR_PIN_HG_COMPENSATION;
        break;
        #endif
        
        default:
            selectorOffsettedPin = i + SENSOR_PIN_OFFSET;
    }
    return selectorOffsettedPin;
}


void setupSensors(){
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.
    
    // TODO skip initial calibration
    // calibrationDone = true;

    for (int i = FIRST_SENSOR; i <= LAST_SENSOR; i++) {
        selectedSensor = (Sensor) i;
        enableSingleMPU(selectedSensor);
        gyros[selectedSensor].mpu = new MPU6050_MPU9150(MPU6050_MPU9150_ADDRESS_AD0_LOW);//   0x68 / 0x69
        if(selectedSensor == HP) {
            gyros[selectedSensor].mpu->isMPU9150 = true;
        }

        int selectorOffsettedPin = selectSingleMPU(i);

        MASTER_SERIAL_NAME.print(F("selectedSensor: "));
        MASTER_SERIAL_NAME.print((int)selectedSensor);
        MASTER_SERIAL_NAME.println(F(""));

        MASTER_SERIAL_NAME.print(F("Enabled on pin: "));
        MASTER_SERIAL_NAME.print(selectorOffsettedPin);
        MASTER_SERIAL_NAME.println(F(""));
        
        initMPUAndDMP(1, i);

//          if(i == HG) {
//                Serial.println(" !!!!! ");
//     Serial.println("Quaternion: ");
//           //  gyros[selectedSensor].mpu->dmpGetGyroSensor()
//            MPU6050_MPU9150 mpu = *gyros[selectedSensor].mpu;
//             fifoCount = mpu.getFIFOCount();
//             uint8_t *fifoBuffer = gyros[selectedSensor].fifoBuffer; // FIFO storage buffer
//             int packetSize = packetSizeS; 
// mpu.setFIFOEnabled(true);
//    DEBUG_PRINTLN(F("Waiting for FIRO count >= 46..."));
//             while ((fifoCount = mpu.getFIFOCount()) < 46);
//             DEBUG_PRINTLN(F("Reading FIFO..."));
//             // getFIFOBytes(fifoBuffer, (fifoCount < 128) ? fifoCount : 128); // safeguard only 128 bytes
//             // DEBUG_PRINTLN(F("Reading interrupt status..."));
//             // getIntStatus();

//             if (fifoCount >= packetSize && fifoCount <= 1024 && fifoCount != 0 ) {
//                 // wait for correct available data length, should be a VERY short wait
//                 while (fifoCount >= packetSize) {
//                     mpu.getFIFOBytes(fifoBuffer, packetSize);
//                     fifoCount -= packetSize;
//                 }
//                 Quaternion *newQ = new Quaternion();
//                 uint8_t res = mpu.dmpGetQuaternion(newQ,fifoBuffer);
//                 // GAG_DEBUG_PRINT("res: ");
//                 // GAG_DEBUG_PRINTLN(res);
//                 //  fmod(2.*acos(A.dot(B)),2.*PI);
//                 newQ->normalize();
//                 // GAG_DEBUG_PRINTLN("DBL: ");

//     // Print quaternion values
//     Serial.println(" !!!!! ");
//     Serial.println("Quaternion: ");
//     Serial.print(newQ->w, 6); Serial.print(", ");
//     Serial.print(newQ->x, 6); Serial.print(", ");
//     Serial.print(newQ->y, 6); Serial.print(", ");
//     Serial.println(newQ->z, 6);
//                 gyros[selectedSensor].q = newQ;
//                 //mpu.resetFIFO();
//                 gyros[selectedSensor].hasDataReady=true;
//             }
//         }

        
        MASTER_SERIAL_NAME.println(F("\n\n"));
    }
}

/**
 * It has to happen in 2 loops because of the need to avoid having multiple uncertain choices
*/
void enableSingleMPU(uint8_t sensorToEnable) {
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

uint8_t initMPUAndDMP(uint8_t attempt, uint8_t i) {
    if (attempt <= 0) {
        return 0;
    }
    // initialize device
#ifdef MASTER_SERIAL_NAME
    MASTER_SERIAL_NAME.println(F("USB: Initializing I2C devices..."));
#endif
    MASTER_SERIAL_NAME.print(F("Enabling DMP... "));
    MASTER_SERIAL_NAME.println(selectedSensor);
    MPU6050_MPU9150 mpu = *gyros[selectedSensor].mpu;
    mpu.initialize();
    MASTER_SERIAL_NAME.println(F("Testing device connections..."));
    MASTER_SERIAL_NAME.println(mpu.testConnection() ? F("MPU* connection successful") : F("MPU* connection failed"));
    MASTER_SERIAL_NAME.println("testConnection");
    MASTER_SERIAL_NAME.println(mpu.getDeviceID());
    if(selectedSensor != HP) {
        MASTER_SERIAL_NAME.println(F("dmpInitialize MPU6050..."));
        MASTER_SERIAL_NAME.println(F("Initializing DMP..."));
        devStatus = mpu.dmpInitialize();
        MASTER_SERIAL_NAME.print(F("DMP initialized..."));
    } else {
        MASTER_SERIAL_NAME.println(F("dmpInitialize MPU9150..."));
        mpu.isMPU9150 = true;
        devStatus = mpu.dmpInitialize2();
        //devStatus = mpu.dmpInitialize();
        MASTER_SERIAL_NAME.print(F("Skipping initialing DMP for MPU9150..."));
    }
    // supply your own gyro offsets here for each mpu, scaled for min sensitivity
    // lets ignore this considering we want realtive values anyway
    
    #ifdef MEASURE_OFFSETS
    // TODO fix measuring offsets for HP
    // if(!calibrationDone && i != HP) {
    // if(!calibrationDone && i == 3) {
    if(selectedSensor == HP || selectedSensor == HG) {
        // measureOffsets(&mpu, i, 10);
    }
    #endif
    #ifdef SET_OFFSETS
    if(selectedSensor == HP){
        // mpu.setXAccelOffset(sensorsOffsets[i][0], MPU9150_RA_XA_OFFS_H);
        // mpu.setYAccelOffset(sensorsOffsets[i][1], MPU9150_RA_YA_OFFS_H);
        // mpu.setZAccelOffset(sensorsOffsets[i][2], MPU9150_RA_ZA_OFFS_H);
        mpu.setXAccelOffset(sensorsOffsets[i][0]);
        mpu.setYAccelOffset(sensorsOffsets[i][1]);
        mpu.setZAccelOffset(sensorsOffsets[i][2]);

    }else{
        mpu.setXAccelOffset(sensorsOffsets[i][0]);
        mpu.setYAccelOffset(sensorsOffsets[i][1]);
        mpu.setZAccelOffset(sensorsOffsets[i][2]);
    }
    mpu.setXGyroOffset(sensorsOffsets[i][3]);
    mpu.setYGyroOffset(sensorsOffsets[i][4]);
    mpu.setZGyroOffset(sensorsOffsets[i][5]);
        
    #endif

    // make sure it worked (returns 0 if so)

    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop( ) function knows it's okay to use it
    MASTER_SERIAL_NAME.println(F("DMP ready! Getting packet size..."));
    MASTER_SERIAL_NAME.print(F("packet size: "));

    if(selectedSensor == HP) {
        MASTER_SERIAL_NAME.print(packetSizeM);
        mpu.setFIFOEnabled(true);

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
    selectedSensor = (Sensor) i;
    enableSingleMPU(selectedSensor);
    return i;
}

void automaticFifoReset() {
    long now = millis();
    static long lastTime = 0;
    int currentlySellectedSensor = selectedSensor;

    for(int i = 0; i < SENSORS_COUNT; i++){
        if(gyros[i].lastResetTime + MIN_TIME_TO_RESET < now) {
            int selectedNow = setOrRotateSelectedGyro(i);
            MPU6050_MPU9150 mpu = *gyros[selectedSensor].mpu;
            int localFifoCount = mpu.getFIFOCount();
            if(( localFifoCount >= MAX_FIFO_USAGE_FOR_RESET || 
                ( gyros[i].lastResetTime + MAX_TIME_TO_RESET < now && 
                localFifoCount >= MIN_FIFO_USAGE_FOR_RESET) ) ){
                mpu.resetFIFO();
                gyros[selectedNow].lastResetTime = now;
            }
        }
        // idk, offsets registeres do not work ...
        /*
        if(i == HP && now - lastTime > 1000){
            MPU6050_MPU9150 mpu = *gyros[selectedSensor].mpu;
            // mpu.setXGyroOffset(35);
            // mpu.setYGyroOffset(-35);
            // mpu.setZGyroOffset(40); // 12?
            
            lastTime=now;
        }*/
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

void getMPU9150Data(MPU6050_MPU9150 * mpu) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
    
    uint16_t qI[4];

    mpu->getMotion9(&ax, &ay, &az, &gx, &gy, &gz,&mx, &my, &mz);
    // Note: Some magic is used here that divides and mupltiplies floats from uint16_t ...
    // It is done to avoid unnecessary variables when dealing with (re)intereting 
    // uint16_t to floating point (S, M, E) calculating stuff with minimal loss of precision
    // and interpreting it back to uint16_t
    // TODO use bit shifts, etc.
    // 8192 16384 32768 65536

    #define offsetDown 16384
    float Gxyz_prev[3];
    Gxyz_prev[0] = Gxyz[0];
    Gxyz_prev[1] = Gxyz[1];
    Gxyz_prev[2] = Gxyz[2];

    Gxyz[0] += (((float) gx / (offsetDown)) * 2);
    Gxyz[1] += (((float) gy / (offsetDown)) * 2);
    Gxyz[2] += (((float) gz / (offsetDown)) * 2);
   
    // Gxyz[0] -= -0.020630f;
    // Gxyz[1] -= 0.014893f;
    // Gxyz[2] -= -0.0185f;
    
    // TODO static diff
    float Gxyz_diff[3];
    Gxyz_diff[0] = Gxyz[0] - Gxyz_prev[0];
    Gxyz_diff[1] = Gxyz[1] - Gxyz_prev[1];
    Gxyz_diff[2] = Gxyz[2] - Gxyz_prev[2];

    // MPU9150_DEBUG_PRINT(F(" Gxyz_diff x: "));    
    // MPU9150_DEBUG_PRINTF(Gxyz_diff[0], 6);    
    // MPU9150_DEBUG_PRINT(F(" y: "));    
    // MPU9150_DEBUG_PRINTF(Gxyz_diff[1], 6);    
    // MPU9150_DEBUG_PRINT(F(" z: "));    
    // MPU9150_DEBUG_PRINTF(Gxyz_diff[2], 6); 
   
    /*
    if(abs(Gxyz_diff[0]) < 0.03f) {
        Gxyz[0] = Gxyz_prev[0];
    }else{
        Gxyz[0] -= -0.020630f;
    }
    if(abs(Gxyz_diff[1]) < 0.03f) {
        Gxyz[1] = Gxyz_prev[1];
    }else{
        Gxyz[1] -= 0.014893f;
    }
    if(abs(Gxyz_diff[2]) < 0.03f) {
        Gxyz[2] = Gxyz_prev[2];
    }else{
        Gxyz[2] -= -0.0185f;
    }
    */

    // MPU9150_DEBUG_PRINT(F(" Gxyz x: "));    
    // MPU9150_DEBUG_PRINTF(Gxyz[0], 6);    
    // MPU9150_DEBUG_PRINT(F(" y: "));    
    // MPU9150_DEBUG_PRINTF(Gxyz[1], 6);    
    // MPU9150_DEBUG_PRINT(F(" z: "));    
    // MPU9150_DEBUG_PRINTF(Gxyz[2], 6); 
    #define offsetDownQ 16

    float yaw = Gxyz[2] / offsetDownQ;
    float pitch = Gxyz[1] / offsetDownQ;
    float roll = Gxyz[0] / offsetDownQ;

    // MPU9150_DEBUG_PRINT(F(" ypr y: "));    
    // MPU9150_DEBUG_PRINTF(yaw, 6);    
    // MPU9150_DEBUG_PRINT(F(" p: "));    
    // MPU9150_DEBUG_PRINTF(pitch, 6);    
    // MPU9150_DEBUG_PRINT(F(" r: "));    
    // MPU9150_DEBUG_PRINTF(roll, 6);    

    #define partition 0.5
    float cy = cos(yaw * partition );
    float sy = sin(yaw * partition );
    float cp = cos(pitch * partition );
    float sp = sin(pitch * partition );
    float cr = cos(roll * partition );
    float sr = sin(roll * partition );

    //#define offsetUp 1024
    // how to make this faster...?
    // https://www.qtcentre.org/threads/60473-How-to-convert-float-to-uint32
    #define offsetUp 2048
    qI[0] = (uint16_t)((cy * cp * cr + sy * sp * sr) * offsetUp);
    qI[1] = (uint16_t)((cy * cp * sr - sy * sp * cr) * offsetUp);
    qI[2] = (uint16_t)((sy * cp * sr + cy * sp * cr) * offsetUp);
    qI[3] = (uint16_t)((sy * cp * cr - cy * sp * sr) * offsetUp);

    // MPU9150_DEBUG_PRINT(F(" qI w: "));    
    // MPU9150_DEBUG_PRINT(qI[0]);    
    // MPU9150_DEBUG_PRINT(F(" x: "));    
    // MPU9150_DEBUG_PRINT(qI[1]);    
    // MPU9150_DEBUG_PRINT(F(" y: "));    
    // MPU9150_DEBUG_PRINT(qI[2]);    
    // MPU9150_DEBUG_PRINT(F(" z: "));    
    // MPU9150_DEBUG_PRINTLN(qI[3]);    

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

// struct Quaternion {
//     float w, x, y, z;
// };
#define GYRO_SCALE 131.0f  // 1 LSB = 1/131 deg/s
#define DEG_TO_RAD 0.0174533f  // (π / 180)
Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};  // Identity quaternion
float dt = 0.5;  // Adjust based on actual loop rate
uint16_t qI[4];   // Integer quaternion for FIFO storage
unsigned long lastTime = 0;  // Store last time (in microseconds)
int16_t gx_offset2 = 0;
int16_t gy_offset2 = 0;
int16_t gz_offset2 = 0;
void correctDriftWithAccel(int16_t ax, int16_t ay, int16_t az) {
    float accelNorm = sqrt(ax * ax + ay * ay + az * az);
    float ax_n = ax / accelNorm;
    float ay_n = ay / accelNorm;
    float az_n = az / accelNorm;

    // Expected gravity direction from quaternion
    float gx_n = 2.0f * (q.x * q.z - q.w * q.y);
    float gy_n = 2.0f * (q.w * q.x + q.y * q.z);
    float gz_n = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;

    // Compute error (difference between expected gravity and measured gravity)
    float ex = (ay_n * gz_n - az_n * gy_n);
    float ey = (az_n * gx_n - ax_n * gz_n);
    float ez = (ax_n * gy_n - ay_n * gx_n);

    // Apply correction (simple proportional feedback)
    float driftGain = 0.01f;
    gx_offset2 += ex * driftGain;
    gy_offset2 += ey * driftGain;
    gz_offset2 += ez * driftGain;
}

void updateQuaternion(int16_t gx, int16_t gy, int16_t gz) {
   gx -= gx_offset2;
    gy -= gy_offset2;
    gz -= gz_offset2;

    // Convert gyro readings from degrees/s to radians/s
    // float gyroScale = 131.0;  // Scale factor for ±250°/s
    float gyroScale = 131.0;  // Scale factor for ±250°/s
    // float gx_r = (gx / gyroScale) * (3.14159265359f / 180.0f);
    // float gy_r = (gy / gyroScale) * (3.14159265359f / 180.0f);
    // float gz_r = (gz / gyroScale) * (3.14159265359f / 180.0f);
  unsigned long currentTime = micros();  // Current time in microseconds
    float dt = (currentTime - lastTime) / 1e6;  // Convert to seconds
    lastTime = currentTime;  // Update time for next iteration

    // ✅ Convert gyro data to radians per second
    float gx_r = (gx / GYRO_SCALE) * DEG_TO_RAD;
    float gy_r = (gy / GYRO_SCALE) * DEG_TO_RAD;
    float gz_r = (gz / GYRO_SCALE) * DEG_TO_RAD;


    // Compute quaternion derivative
    Quaternion qDot;
    qDot.w = -0.5f * (q.x * gx_r + q.y * gy_r + q.z * gz_r);
    qDot.x =  0.5f * (q.w * gx_r + q.y * gz_r - q.z * gy_r);
    qDot.y =  0.5f * (q.w * gy_r - q.x * gz_r + q.z * gx_r);
    qDot.z =  0.5f * (q.w * gz_r + q.x * gy_r - q.y * gx_r);

    // Integrate to update quaternion
    q.w += qDot.w * dt;
    q.x += qDot.x * dt;
    q.y += qDot.y * dt;
    q.z += qDot.z * dt;

    // Normalize the quaternion
    // float norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    // q.w /= norm;
    // q.x /= norm;
    // q.y /= norm;
    // q.z /= norm;
    q.normalize();
}


void storeQuaternionInFIFO() {
    int16_t qI[4];

    // ✅ Use correct scaling (16384 like dmpGetQuaternion expects)
    qI[0] = (int16_t)(q.w * 16384.0f);
    qI[1] = (int16_t)(q.x * 16384.0f);
    qI[2] = (int16_t)(q.y * 16384.0f);
    qI[3] = (int16_t)(q.z * 16384.0f);

    // ✅ Store in FIFO using correct order (Big Endian)
    uint8_t *fifoBuffer = gyros[selectedSensor].fifoBuffer;

    fifoBuffer[0]  = (qI[0] >> 8) & 0xFF;  // High Byte
    fifoBuffer[1]  = qI[0] & 0xFF;         // Low Byte
    
    fifoBuffer[4]  = (qI[1] >> 8) & 0xFF;
    fifoBuffer[5]  = qI[1] & 0xFF;
    
    fifoBuffer[8]  = (qI[2] >> 8) & 0xFF;
    fifoBuffer[9]  = qI[2] & 0xFF;

    fifoBuffer[12] = (qI[3] >> 8) & 0xFF;
    fifoBuffer[13] = qI[3] & 0xFF;
}


// Kalman filter state (4D state for quaternion)
float Q_angle = 0.001;  // Process noise covariance
float Q_bias = 0.003;   // Process noise covariance for bias
float R_measure = 0.03; // Measurement noise covariance

// Kalman state variables
float P[4][4] = {0}; // Error covariance matrix
float K[4] = {0};    // Kalman gain
float q_est[4] = {1, 0, 0, 0}; // Estimated quaternion
float bias[3] = {0,0,0};
void predictQuaternion(float gx, float gy, float gz, float dt) {
    // Apply bias correction
    gx -= bias[0];
    gy -= bias[1];
    gz -= bias[2];

    // Convert gyro rates to radians
    gx *= (PI / 180.0);
    gy *= (PI / 180.0);
    gz *= (PI / 180.0);

    // Compute quaternion derivative
    float q_dot[4] = {
        0.5 * (-q_est[1] * gx - q_est[2] * gy - q_est[3] * gz),
        0.5 * ( q_est[0] * gx + q_est[2] * gz - q_est[3] * gy),
        0.5 * ( q_est[0] * gy - q_est[1] * gz + q_est[3] * gx),
        0.5 * ( q_est[0] * gz + q_est[1] * gy - q_est[2] * gx)
    };

    // Update quaternion estimate
    q_est[0] += q_dot[0] * dt;
    q_est[1] += q_dot[1] * dt;
    q_est[2] += q_dot[2] * dt;
    q_est[3] += q_dot[3] * dt;

    // Normalize quaternion
    float norm = sqrt(q_est[0] * q_est[0] + q_est[1] * q_est[1] +
                      q_est[2] * q_est[2] + q_est[3] * q_est[3]);
    q_est[0] /= norm;
    q_est[1] /= norm;
    q_est[2] /= norm;
    q_est[3] /= norm;

    // Update error covariance matrix (adding process noise)
    for (int i = 0; i < 4; i++) {
        P[i][i] += Q_angle * dt;
    }
}



// void updateQuaternion2(float ax, float ay, float az, float mx, float my, float mz) {
//     // Normalize accelerometer data
//     float norm = sqrt(ax * ax + ay * ay + az * az);
//     ax /= norm;
//     ay /= norm;
//     az /= norm;

//     // Normalize magnetometer data
//     norm = sqrt(mx * mx + my * my + mz * mz);
//     mx /= norm;
//     my /= norm;
//     mz /= norm;

//     // Compute reference quaternion from accelerometer & magnetometer
//     float ref_q[4];
//     ref_q[0] = sqrt(0.5f * (1.0f + ax));
//     ref_q[1] = ay / (2.0f * ref_q[0]);
//     ref_q[2] = az / (2.0f * ref_q[0]);
//     ref_q[3] = 0.0f;

//     // Compute quaternion difference (error)
//     float error[4] = {
//         q_est[0] - ref_q[0],
//         q_est[1] - ref_q[1],
//         q_est[2] - ref_q[2],
//         q_est[3] - ref_q[3]
//     };

//     // Kalman gain calculation
//     for (int i = 0; i < 4; i++) {
//         K[i] = P[i][i] / (P[i][i] + R_measure);
//     }

//     // Correct quaternion estimate
//     for (int i = 0; i < 4; i++) {
//         q_est[i] -= K[i] * error[i];
//     }

//     // Normalize quaternion
//     norm = sqrt(q_est[0] * q_est[0] + q_est[1] * q_est[1] +
//                 q_est[2] * q_est[2] + q_est[3] * q_est[3]);
//     q_est[0] /= norm;
//     q_est[1] /= norm;
//     q_est[2] /= norm;
//     q_est[3] /= norm;
// }

void predictQuaternion2(float gx, float gy, float gz, float dt) {
    // Apply bias correction
    gx -= bias[0];
    gy -= bias[1];
    gz -= bias[2];

    // Convert gyro rates to radians
    gx *= (PI / 180.0);
    gy *= (PI / 180.0);
    gz *= (PI / 180.0);

    // Compute quaternion derivative
    float q_dot[4] = {
        0.5 * (-q_est[1] * gx - q_est[2] * gy - q_est[3] * gz),
        0.5 * ( q_est[0] * gx + q_est[2] * gz - q_est[3] * gy),
        0.5 * ( q_est[0] * gy - q_est[1] * gz + q_est[3] * gx),
        0.5 * ( q_est[0] * gz + q_est[1] * gy - q_est[2] * gx)
    };

    // Update quaternion estimate
    q_est[0] += q_dot[0] * dt;
    q_est[1] += q_dot[1] * dt;
    q_est[2] += q_dot[2] * dt;
    q_est[3] += q_dot[3] * dt;

    // Normalize quaternion
    float norm = sqrt(q_est[0] * q_est[0] + q_est[1] * q_est[1] +
                      q_est[2] * q_est[2] + q_est[3] * q_est[3]);
    q_est[0] /= norm;
    q_est[1] /= norm;
    q_est[2] /= norm;
    q_est[3] /= norm;

    // Update error covariance matrix (adding process noise)
    for (int i = 0; i < 4; i++) {
        P[i][i] += Q_angle * dt;
    }
}


// // Kalman filter state variables
// float P[4][4] = {0};  // Error covariance matrix
// float Q_angle = 0.001; // Process noise covariance for quaternion
// float Q_bias = 0.003;  // Process noise covariance for gyroscope bias
// float R_measure = 0.03; // Measurement noise covariance

// float bias[3] = {0, 0, 0}; // Gyroscope bias correction (X, Y, Z)

// float q_est[4] = {1, 0, 0, 0}; // Quaternion estimate (initialized to identity)

// // Function to predict quaternion using gyroscope
// void predictQuaternion(float gx, float gy, float gz, float dt) {
//     // Apply bias correction
//     gx -= bias[0];
//     gy -= bias[1];
//     gz -= bias[2];

//     // Convert gyro rates to radians
//     gx *= (PI / 180.0);
//     gy *= (PI / 180.0);
//     gz *= (PI / 180.0);

//     // Compute quaternion derivative
//     float q_dot[4] = {
//         0.5 * (-q_est[1] * gx - q_est[2] * gy - q_est[3] * gz),
//         0.5 * ( q_est[0] * gx + q_est[2] * gz - q_est[3] * gy),
//         0.5 * ( q_est[0] * gy - q_est[1] * gz + q_est[3] * gx),
//         0.5 * ( q_est[0] * gz + q_est[1] * gy - q_est[2] * gx)
//     };

//     // Update quaternion estimate
//     q_est[0] += q_dot[0] * dt;
//     q_est[1] += q_dot[1] * dt;
//     q_est[2] += q_dot[2] * dt;
//     q_est[3] += q_dot[3] * dt;

//     // Normalize quaternion
//     float norm = sqrt(q_est[0] * q_est[0] + q_est[1] * q_est[1] +
//                       q_est[2] * q_est[2] + q_est[3] * q_est[3]);
//     q_est[0] /= norm;
//     q_est[1] /= norm;
//     q_est[2] /= norm;
//     q_est[3] /= norm;

//     // Update error covariance matrix
//     for (int i = 0; i < 4; i++) {
//         P[i][i] += Q_angle * dt;
//     }
// }

// Function to update quaternion using accelerometer and magnetometer
void updateQuaternion2(float ax, float ay, float az, float mx, float my, float mz) {
    // Normalize accelerometer
    float norm = sqrt(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Normalize magnetometer
    norm = sqrt(mx * mx + my * my + mz * mz);
    mx /= norm;
    my /= norm;
    mz /= norm;

    // Compute reference quaternion from accelerometer & magnetometer
    float ref_q[4];
    ref_q[0] = sqrt(0.5f * (1.0f + ax));
    ref_q[1] = ay / (2.0f * ref_q[0]);
    ref_q[2] = az / (2.0f * ref_q[0]);
    ref_q[3] = 0.0f;

    // Compute quaternion error
    float error[4] = {
        q_est[0] - ref_q[0],
        q_est[1] - ref_q[1],
        q_est[2] - ref_q[2],
        q_est[3] - ref_q[3]
    };

    // Compute Kalman gain
    float S = P[0][0] + R_measure;
    float K[4];
    for (int i = 0; i < 4; i++) {
        K[i] = P[i][i] / S;
    }

    // Update quaternion
    for (int i = 0; i < 4; i++) {
        q_est[i] -= K[i] * error[i];
    }

    // Normalize quaternion
    norm = sqrt(q_est[0] * q_est[0] + q_est[1] * q_est[1] +
                q_est[2] * q_est[2] + q_est[3] * q_est[3]);
    q_est[0] /= norm;
    q_est[1] /= norm;
    q_est[2] /= norm;
    q_est[3] /= norm;

    // Update bias estimate
    for (int i = 0; i < 3; i++) {
        bias[i] += Q_bias * K[i] * error[i];
    }

    // Update error covariance
    for (int i = 0; i < 4; i++) {
        P[i][i] *= (1 - K[i]);
    }
}


void loadMPU9150Data(MPU6050_MPU9150 *mpu) {
      int16_t ax, ay, az, gx, gy, gz, mx, my, mz;

 // Get sensor data
    // mpu->getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // Update quaternion from gyro data
    // updateQuaternion(gx, gy, gz);
    // updateQuaternion(gx, gy, gz);
        // correctDriftWithAccel(ax, ay, az);
          // Read raw sensor data


    mpu->getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    // Convert raw data
    gx /= 131.0f; gy /= 131.0f; gz /= 131.0f;
    ax /= 16384.0f; ay /= 16384.0f; az /= 16384.0f;

    // Compute time delta
    static unsigned long lastTime = 0;
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    // Predict quaternion using gyroscope
    predictQuaternion2(gx, gy, gz, dt);

    // Correct drift using accelerometer and magnetometer
    updateQuaternion2(ax, ay, az, mx, my, mz);

    // Print quaternion
    Serial.print("Quaternion: ");
    Serial.print(q_est[0]); Serial.print(", ");
    Serial.print(q_est[1]); Serial.print(", ");
    Serial.print(q_est[2]); Serial.print(", ");
    Serial.println(q_est[3]);

    // delay(10);


        storeQuaternionInFIFO();

    // // Print quaternion values
    // Serial.print("Quaternion: ");
    // Serial.print(q.w, 6); Serial.print(", ");
    // Serial.print(q.x, 6); Serial.print(", ");
    // Serial.print(q.y, 6); Serial.print(", ");
    // Serial.println(q.z, 6);

}


void loadMPU60500Data(MPU6050_MPU9150 *mpu) {
      int16_t ax, ay, az, gx, gy, gz, mx, my, mz;

 // Get sensor data
    mpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Update quaternion from gyro data
    // updateQuaternion(gx, gy, gz);
    updateQuaternion(gx, gy, gz);
        correctDriftWithAccel(ax, ay, az);
        storeQuaternionInFIFO();

    // // Print quaternion values
    // Serial.print("Quaternion: ");
    // Serial.print(q.w, 6); Serial.print(", ");
    // Serial.print(q.x, 6); Serial.print(", ");
    // Serial.print(q.y, 6); Serial.print(", ");
    // Serial.println(q.z, 6);

}


void loadMPU9150Data2(MPU6050_MPU9150 *mpu) {
}


bool loadDataFromFIFO(bool forceLoad) {
    if (selectedSensor == HG) {
      forceLoad = true;
            MPU6050_MPU9150 mpu = *gyros[selectedSensor].mpu;
//  if(!gyros[selectedSensor].hasDataReady || forceLoad) {
//             // getMPU9150Data(gyros[selectedSensor].mpu);
//             loadMPU60500Data(gyros[selectedSensor].mpu);
//             // loadMPU9150Data2(gyros[selectedSensor].mpu);
//             gyros[selectedSensor].hasDataReady=true;
//      }

//             gyros[selectedSensor].alreadySentData=false;
//             return true;
      // mpu.dmpInitialize3();

            fifoCount = mpu.getFIFOCount();
            uint8_t *fifoBuffer = gyros[selectedSensor].fifoBuffer; // FIFO storage buffer
            int packetSize = packetSizeS; 
mpu.setFIFOEnabled(true);
  //  DEBUG_PRINTLN(F("Waiting for FIRO count >= 46..."));
   
            // while ((fifoCount = mpu.getFIFOCount()) < 46);
            // DEBUG_PRINTLN(F("Reading FIFO..."));
            // getFIFOBytes(fifoBuffer, (fifoCount < 128) ? fifoCount : 128); // safeguard only 128 bytes
            // DEBUG_PRINTLN(F("Reading interrupt status..."));
            // getIntStatus();

            // if (fifoCount >= packetSize && fifoCount <= 1024 && fifoCount != 0 ) {
                // wait for correct available data length, should be a VERY short wait
                while (fifoCount >= packetSize) {
                    mpu.getFIFOBytes(fifoBuffer, packetSize);
                    fifoCount -= packetSize;
                }
                Quaternion *newQ = new Quaternion();
                uint8_t res = mpu.dmpGetQuaternion(newQ,fifoBuffer);
                // GAG_DEBUG_PRINT("res: ");
                // GAG_DEBUG_PRINTLN(res);
                //  fmod(2.*acos(A.dot(B)),2.*PI);
                newQ->normalize();
                // GAG_DEBUG_PRINTLN("DBL: ");

    // Print quaternion values
    // Serial.println(" !!!!! ");
    // Serial.println("Quaternion: ");
    // Serial.print(newQ->w, 6); Serial.print(", ");
    // Serial.print(newQ->x, 6); Serial.print(", ");
    // Serial.print(newQ->y, 6); Serial.print(", ");
    // Serial.println(newQ->z, 6);
                gyros[selectedSensor].q = newQ;
                //mpu.resetFIFO();
                gyros[selectedSensor].hasDataReady=true;
        // }

    }
    // else if(selectedSensor != HP) {
    if(selectedSensor != HP) {
    // if (true) {
        MPU6050_MPU9150 mpu = *gyros[selectedSensor].mpu;


if (selectedSensor == HG) {
          //      Serial.println(" !!!!! ");
          //  Serial.println("Quaternion: ");
          // //  gyros[selectedSensor].mpu->dmpGetGyroSensor()
          //  MPU6050_MPU9150 mpu = *gyros[selectedSensor].mpu;
          //  mpu.reset();
          // //  mpu.setDMPEnabled(true);
          // //  mpu.setFIFOEnabled(true);
          //  mpu.resetFIFO();
          //   mpu.getIntStatus();

          //   fifoCount = mpu.getFIFOCount();
          //   uint8_t *fifoBuffer = gyros[selectedSensor].fifoBuffer; // FIFO storage buffer
          //   int packetSize = packetSizeS; 

          //   DEBUG_PRINTLN(F("Waiting for FIRO count >= 46..."));
          //   while ((fifoCount = mpu.getFIFOCount()) < 42);
          //   DEBUG_PRINTLN(F("Reading FIFO..."));
          //   // getFIFOBytes(fifoBuffer, (fifoCount < 128) ? fifoCount : 128); // safeguard only 128 bytes
          //   // DEBUG_PRINTLN(F("Reading interrupt status..."));
          //   // getIntStatus();
                  // initMPUAndDMP(1, i);
//  MASTER_SERIAL_NAME.println(selectedSensor);
    // mpu.initialize();
    // MASTER_SERIAL_NAME.println(mpu.testConnection() ? F("MPU* connection successful") : F("MPU* connection failed"));
        // devStatus = mpu.dmpInitialize();
        // mpu.resetFIFO();

}

        if(!gyros[selectedSensor].hasDataReady || forceLoad) {
            fifoCount = mpu.getFIFOCount();
            uint8_t *fifoBuffer = gyros[selectedSensor].fifoBuffer; // FIFO storage buffer
            int packetSize = packetSizeS; 


            if (fifoCount >= packetSize && fifoCount <= 1024 && fifoCount != 0 ) {
                // wait for correct available data length, should be a VERY short wait
                while (fifoCount >= packetSize) {
                    mpu.getFIFOBytes(fifoBuffer, packetSize);
                    fifoCount -= packetSize;
                }
                Quaternion *newQ = new Quaternion();
                uint8_t res = mpu.dmpGetQuaternion(newQ,fifoBuffer);
                // GAG_DEBUG_PRINT("res: ");
                // GAG_DEBUG_PRINTLN(res);
                //  fmod(2.*acos(A.dot(B)),2.*PI);
                newQ->normalize();
                // GAG_DEBUG_PRINTLN("DBL: ");

                if(gyros[selectedSensor].debugDiff && gyros[selectedSensor].q != nullptr) {
                // if(gyros[selectedSensor].q != nullptr) {
                    double dbl = fmod(2.*acos(newQ->dot(gyros[selectedSensor].q)),2.*PI);
                    #ifdef TEST_CALIB
                    #endif

                    GAG_DEBUG_PRINT("DBL: ");
                    GAG_DEBUG_PRINT(selectedSensor);
                    GAG_DEBUG_PRINT(" ");
                    GAG_DEBUG_PRINTF(dbl, 6);
                    /*
                    GAG_DEBUG_PRINT(" w:");
                    GAG_DEBUG_PRINTF(newQ->w, 6);
                    GAG_DEBUG_PRINT(" x:");
                    GAG_DEBUG_PRINTF(newQ->x, 6);
                    GAG_DEBUG_PRINT(" y:");
                    GAG_DEBUG_PRINTF(newQ->y, 6);
                    GAG_DEBUG_PRINT(" z:");
                    GAG_DEBUG_PRINTF(newQ->z, 6);
                    GAG_DEBUG_PRINT(" prev w:");
                    GAG_DEBUG_PRINTF(gyros[selectedSensor].q->w, 6);
                    GAG_DEBUG_PRINT(" x:");
                    GAG_DEBUG_PRINTF(gyros[selectedSensor].q->x, 6);
                    GAG_DEBUG_PRINT(" y:");
                    GAG_DEBUG_PRINTF(gyros[selectedSensor].q->y, 6);
                    GAG_DEBUG_PRINT(" z:");
                    GAG_DEBUG_PRINTLNF(gyros[selectedSensor].q->z, 6);
                    */
                    GAG_DEBUG_PRINT(" diff w:");
                    if(newQ->w - gyros[selectedSensor].q->w >= 0){GAG_DEBUG_PRINT(" ");};
                    GAG_DEBUG_PRINTF(newQ->w - gyros[selectedSensor].q->w, 6);
                    GAG_DEBUG_PRINT(" x:");
                    if(newQ->x - gyros[selectedSensor].q->x >= 0){GAG_DEBUG_PRINT(" ");};
                    GAG_DEBUG_PRINTF(newQ->x - gyros[selectedSensor].q->x, 6);
                    GAG_DEBUG_PRINT(" y:");
                    if(newQ->y - gyros[selectedSensor].q->y >= 0){GAG_DEBUG_PRINT(" ");};
                    GAG_DEBUG_PRINTF(newQ->y - gyros[selectedSensor].q->y, 6);
                    GAG_DEBUG_PRINT(" z:");
                    if(newQ->z - gyros[selectedSensor].q->z >= 0){GAG_DEBUG_PRINT(" ");};
                    GAG_DEBUG_PRINTLNF(newQ->z - gyros[selectedSensor].q->z, 6);
                }

    // Print quaternion values
    // Serial.print("Quaternion: ");
    // Serial.print(newQ->w, 6); Serial.print(", ");
    // Serial.print(newQ->x, 6); Serial.print(", ");
    // Serial.print(newQ->y, 6); Serial.print(", ");
    // Serial.println(newQ->z, 6);
                gyros[selectedSensor].q = newQ;
                //mpu.resetFIFO();
                gyros[selectedSensor].hasDataReady=true;
                return true;
            }
        }
    } else {
        forceLoad = true;
        // Serial.println("HP");
        MPU6050_MPU9150 mpu = *gyros[selectedSensor].mpu;
        if(!gyros[selectedSensor].hasDataReady || forceLoad) {
            // getMPU9150Data(gyros[selectedSensor].mpu);
            loadMPU9150Data(gyros[selectedSensor].mpu);
            // loadMPU9150Data2(gyros[selectedSensor].mpu);
            gyros[selectedSensor].hasDataReady=true;


            // fifoCount = mpu.getFIFOCount2();
            // uint8_t *fifoBuffer = gyros[selectedSensor].fifoBuffer; // FIFO storage buffer
            // int packetSize = packetSizeM; 
            // // Serial.println("fifoCount: ");
            // // GAG_DEBUG_PRINT("fifoCount: ");
            // // GAG_DEBUG_PRINTLN(fifoCount);


            //     mpu.resetFIFO();

            //     // DEBUG_PRINTLN(F("Enabling FIFO..."));
            //     // mpu.setFIFOEnabled(true);

            //     // DEBUG_PRINTLN(F("Enabling DMP..."));
            //     // mpu.setDMPEnabled(true);

            //     // DEBUG_PRINTLN(F("Resetting DMP..."));
            //     // mpu.resetDMP();
            
            //     // DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
            //     // while ((fifoCount = mpu.getFIFOCount()) < 3);
            //     while ((fifoCount = mpu.getFIFOCount()) < packetSizeM);
                
            // if (fifoCount >= packetSize /*&& fifoCount <= 1024*/ && fifoCount != 0 ) {
            //     // wait for correct available data length, should be a VERY short wait
            //     while (fifoCount >= packetSize) {
            //         mpu.getFIFOBytes(fifoBuffer, packetSize);
            //         fifoCount -= packetSize;
            //     }
            //     Quaternion *newQ = new Quaternion();
            //     uint8_t res = mpu.dmpGetQuaternion(newQ,fifoBuffer);
            //     // GAG_DEBUG_PRINT("res: ");
            //     // GAG_DEBUG_PRINTLN(res);
            //     //  fmod(2.*acos(A.dot(B)),2.*PI);
            //     newQ->normalize();
            //     // GAG_DEBUG_PRINTLN("DBL: ");

            //     // if(gyros[selectedSensor].debugDiff && gyros[selectedSensor].q != nullptr) {
            //     // // if(gyros[selectedSensor].q != nullptr) {
            //     //     double dbl = fmod(2.*acos(newQ->dot(gyros[selectedSensor].q)),2.*PI);
            //     //     #ifdef TEST_CALIB
            //     //     #endif

            //     //     GAG_DEBUG_PRINT("DBL: ");
            //     //     GAG_DEBUG_PRINT(selectedSensor);
            //     //     GAG_DEBUG_PRINT(" ");
            //     //     GAG_DEBUG_PRINTF(dbl, 6);
                    
            //     //     GAG_DEBUG_PRINT(" w:");
            //     //     GAG_DEBUG_PRINTF(newQ->w, 6);
            //     //     GAG_DEBUG_PRINT(" x:");
            //     //     GAG_DEBUG_PRINTF(newQ->x, 6);
            //     //     GAG_DEBUG_PRINT(" y:");
            //     //     GAG_DEBUG_PRINTF(newQ->y, 6);
            //     //     GAG_DEBUG_PRINT(" z:");
            //     //     GAG_DEBUG_PRINTF(newQ->z, 6);
            //     //     GAG_DEBUG_PRINT(" prev w:");
            //     //     GAG_DEBUG_PRINTF(gyros[selectedSensor].q->w, 6);
            //     //     GAG_DEBUG_PRINT(" x:");
            //     //     GAG_DEBUG_PRINTF(gyros[selectedSensor].q->x, 6);
            //     //     GAG_DEBUG_PRINT(" y:");
            //     //     GAG_DEBUG_PRINTF(gyros[selectedSensor].q->y, 6);
            //     //     GAG_DEBUG_PRINT(" z:");
            //     //     GAG_DEBUG_PRINTLNF(gyros[selectedSensor].q->z, 6);
                   
            //     //     GAG_DEBUG_PRINT(" diff w:");
            //     //     if(newQ->w - gyros[selectedSensor].q->w >= 0){GAG_DEBUG_PRINT(" ");};
            //     //     GAG_DEBUG_PRINTF(newQ->w - gyros[selectedSensor].q->w, 6);
            //     //     GAG_DEBUG_PRINT(" x:");
            //     //     if(newQ->x - gyros[selectedSensor].q->x >= 0){GAG_DEBUG_PRINT(" ");};
            //     //     GAG_DEBUG_PRINTF(newQ->x - gyros[selectedSensor].q->x, 6);
            //     //     GAG_DEBUG_PRINT(" y:");
            //     //     if(newQ->y - gyros[selectedSensor].q->y >= 0){GAG_DEBUG_PRINT(" ");};
            //     //     GAG_DEBUG_PRINTF(newQ->y - gyros[selectedSensor].q->y, 6);
            //     //     GAG_DEBUG_PRINT(" z:");
            //     //     if(newQ->z - gyros[selectedSensor].q->z >= 0){GAG_DEBUG_PRINT(" ");};
            //     //     GAG_DEBUG_PRINTLNF(newQ->z - gyros[selectedSensor].q->z, 6);
            //     // }
            //     gyros[selectedSensor].q = newQ;
            //     mpu.resetFIFO();
            //     gyros[selectedSensor].hasDataReady=true;
            //     // return true;
            // }




            
            gyros[selectedSensor].alreadySentData=false;
            return true;
        }
    }
    return false;
}

void writePacket() {
    uint8_t *fifoBuffer = gyros[selectedSensor].fifoBuffer; // FIFO storage buffer
#ifdef MASTER_HAND
    if(!gyros[selectedSensor].alreadySentData && gyros[selectedSensor].hasDataReady) {
        fifoToPacket(fifoBuffer, dataPacket, selectedSensor);
        // hotfix of broken sensor's z-axis
        // if(selectedSensor == 3) { // TODO fix names
            // dataPacket[9] = gyros[4].fifoBuffer[12];
            // dataPacket[10] = gyros[4].fifoBuffer[13];
        // }
        #ifdef USE_BT_GATT_SERIAL
            MASTER_SERIAL_NAME.write(dataPacket, PACKET_LENGTH);
            //#define SEND_DATA_ALSO_OVER_SERIAL
            #ifdef SEND_DATA_ALSO_OVER_SERIAL
                Serial.write(dataPacket, PACKET_LENGTH);
                Serial.write((byte)0x00);
            #endif
        #else
            MASTER_SERIAL_NAME.write(dataPacket, PACKET_LENGTH);
            MASTER_SERIAL_NAME.write((byte)0x00);
        #endif
        gyros[selectedSensor].hasDataReady = false;
        gyros[selectedSensor].alreadySentData = true;
        dataPacket[PACKET_COUNTER_POSITION]++; // packetCount, loops at 0xFF on purpose
    }
#else
#ifdef SLAVE_HAND
    if(!gyros[selectedSensor].alreadySentData && gyros[selectedSensor].hasDataReady){
        fifoToPacket(fifoBuffer, dataPacket, selectedSensor);
        MASTER_SERIAL_NAME.write(dataPacket, PACKET_LENGTH);
        MASTER_SERIAL_NAME.write(0x00);
        dataPacket[PACKET_COUNTER_POSITION]++; // packetCount, loops at 0xFF on purpose
        gyros[selectedSensor].hasDataReady = false;
        gyros[selectedSensor].alreadySentData = true;
    }
    readAlign = 0;
#endif
#ifdef MASTER_HAND
    if(!gyros[selectedSensor].alreadySentData && gyros[selectedSensor].hasDataReady) {
        fifoToPacket(fifoBuffer, dataPacket, selectedSensor);
        MASTER_SERIAL_NAME.write(dataPacket, PACKET_LENGTH);
        MASTER_SERIAL_NAME.write((byte)0x00);
        gyros[selectedSensor].hasDataReady = false;
        gyros[selectedSensor].alreadySentData = true;
        dataPacket[PACKET_COUNTER_POSITION]++; // packetCount, loops at 0xFF on purpose
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

/**
 * Exec cmd based on: 
 * - hand command set (enabled by compilation)
 * 
 * example cmds:
 * set 
 * - O1gx9
 * get
 * - C1gx0
*/
void execCommand(/*const byte * ch */) {
    // {'c', 0, 0, 0, 0, 0, 0, '\r', '\n'};
    // {'C', 0, 0, 0, 0, 0, 0, '\r', '\n'};
    // calibration
    // {'C', 'a', 0, 0, 0, 0, 0, '\r', '\n'};

    //cmdPacket[1] = *ch;
    GAG_DEBUG_PRINT("execCommand: ");
    for(char i =0; i < CMD_PACKET_LENGTH; i++) {
        GAG_DEBUG_PRINT((char)cmdPacket[i]);
    }

    switch(cmdPacket[1]) {
        case CMD_TEST_REPLY:
            GAG_DEBUG_PRINTLN("CMD_TEST_REPLY");
            break;
        case CMD_GET_CURRENT_OFFSET:
        case CMD_SET_OFFSET: {
            // TODO const access..?
            uint8_t sensorIndex = cmdPacket[2];
            // agent hand sensorIndex > 5 else slave hand
            if(sensorIndex >= 48) {sensorIndex-=48;}
            if(sensorIndex > 5) {sensorIndex -=5;}
            int8_t selectedSensorBkp = selectedSensor;
            GAG_DEBUG_PRINT("Selecting ");
            GAG_DEBUG_PRINTLN(sensorIndex);

            enableSingleMPU(sensorIndex);
            MPU6050_MPU9150 * currentMpu = gyros[sensorIndex].mpu;

            switch(cmdPacket[1]) {
                case CMD_GET_CURRENT_OFFSET: {
                    // int16_t offset = 0;
                    int16_t offset = 0;
                    switch(cmdPacket[3]) {
                        case 'g': 
                            switch(cmdPacket[4]) {
                                case 'x':
                                    offset = (currentMpu->getXGyroOffset());
                                break;
                                case 'y':
                                    offset = (currentMpu->getYGyroOffset());
                                break;
                                case 'z':
                                    offset = (currentMpu->getZGyroOffset());
                                break;
                            }
                            break;
                        case 'a': 
                            switch(cmdPacket[4]) {
                                case 'x':
                                    offset = currentMpu->getXAccelOffset();
                                break;
                                case 'y':
                                    offset = currentMpu->getYAccelOffset();
                                break;
                                case 'z':
                                    offset = currentMpu->getZAccelOffset();
                                break;
                            }
                            break;
                            // case 'm': break; // so far magnetometer does not support setting offsets 
                        // default:
                    }
                    GAG_DEBUG_PRINTLN("CMD_GET_CURRENT_OFFSET");
                    GAG_DEBUG_PRINTLN(offset);
                }
                break;
                case CMD_SET_OFFSET:
                    GAG_DEBUG_PRINT("CMD_SET_OFFSET");
                    GAG_DEBUG_PRINT((char)cmdPacket[2]);
                    GAG_DEBUG_PRINT(" ");
                    GAG_DEBUG_PRINT((char)cmdPacket[3]);
                    GAG_DEBUG_PRINT(" ");
                    GAG_DEBUG_PRINT((char)cmdPacket[4]);
                    GAG_DEBUG_PRINT(" ");
                    GAG_DEBUG_PRINT((char)cmdPacket[5]);
                    GAG_DEBUG_PRINT(" ");
                    GAG_DEBUG_PRINTLN(cmdPacket[5]);
                    //delay(100);
                    //currentMpu->initialize();
                    int16_t offset = 0;
                    if(cmdPacket[5]!=0){
                        offset = ((static_cast<uint16_t>(cmdPacket[5])) << 8) | cmdPacket[6];
                    }
                    GAG_DEBUG_PRINT("offset ");
                    GAG_DEBUG_PRINTLN(offset);

                    switch(cmdPacket[3]) {
                        case 'g':
                            switch(cmdPacket[4]){
                                case 'x':
                                    currentMpu->setXGyroOffset(offset);
                                    sensorsOffsets[sensorIndex][3] = offset;
                                    //currentMpu->reset();
                                    //initMPUAndDMP(1,sensorIndex);
                                break;
                                case 'y':
                                    currentMpu->setYGyroOffset(offset);
                                    sensorsOffsets[sensorIndex][4] = offset;
                                break;
                                case 'z':
                                    currentMpu->setZGyroOffset(offset);
                                    sensorsOffsets[sensorIndex][5] = offset;
                                break;
                            }
                            break;
                        case 'a':
                            // TODO use 2 bytes convert them to 2 uint8_t and merge them to uint16_t
                            switch(cmdPacket[4]){
                                case 'x':
                                    currentMpu->setXAccelOffset(offset);
                                    sensorsOffsets[sensorIndex][0] = offset;
                                break;
                                case 'y':
                                    currentMpu->setYAccelOffset(offset);
                                    sensorsOffsets[sensorIndex][1] = offset;
                                break;
                                case 'z':
                                    currentMpu->setZAccelOffset(offset);
                                    sensorsOffsets[sensorIndex][2] = offset;
                                break;
                            }
                            break;
                            // case 'm': break;
                        // default:
                    }
                    // delay(100);
                break;
                }
                enableSingleMPU(selectedSensorBkp);
            }
            break;
        case CMD_SET_SENSOR_DEBUG_FLAG:{
                uint8_t sensorIndex = cmdPacket[2];
                if(sensorIndex >= 48) {sensorIndex-=48;}
                if(sensorIndex > 5) {sensorIndex -=5;}
                
                if(cmdPacket[3] == 'T') {
                    GAG_DEBUG_PRINT("En");
                    gyros[sensorIndex].debugDiff = true;
                } else {
                    GAG_DEBUG_PRINT("Dis");
                    gyros[sensorIndex].debugDiff = false;
                }
                GAG_DEBUG_PRINT("able debug for sensor: ");
                GAG_DEBUG_PRINTLN(sensorIndex);
            }
            break;
        // case CMD_RESTART_WITH_CALIBRATION_AND_SEND:
        case CMD_CALIBRATION:{
                #ifdef MEASURE_OFFSETS
                    calibrationDone = false;
                #endif
                uint8_t sensorIndex = cmdPacket[2];
                uint8_t selectedSensorBkp = selectedSensor;
                if(sensorIndex >= 48) {sensorIndex-=48;}
                if(sensorIndex > 5) {sensorIndex = sensorIndex % 6;}
                enableSingleMPU(sensorIndex);
                GAG_DEBUG_PRINT("Selected: ");
                GAG_DEBUG_PRINTLN(sensorIndex);
                int16_t limit =  ((static_cast<uint16_t>(cmdPacket[6])) << 8) | cmdPacket[5];
                GAG_DEBUG_PRINT("limit: ");
                GAG_DEBUG_PRINTLN(limit);
                #ifdef MEASURE_OFFSETS
                measureOffsets(gyros[selectedSensor].mpu, sensorIndex, limit);
                #endif
                enableSingleMPU(selectedSensorBkp);
                setupSensors();
            }
            break;
        case CMD_RESTART:
            if(cmdPacket[2] == CMD_RESTART || cmdPacket[3] == CMD_RESTART) {
                setup();
            }
            break;
        #ifdef MASTER_HAND
            case CMD_READ_SLAVE:
                useSlaveHand = true;
                break;
            case CMD_DONT_READ_SLAVE:
                useSlaveHand = false;
                break;
        #endif
        default:
            break;
    }
    #ifdef MEASURE_OFFSETS
    if(cmdPacket[1] == CMD_RESTART_WITH_CALIBRATION_AND_SEND) {
/*
this is just a note for better code readability ... 
on the other hand substituting comments for readable code seems like a bad practice.. 
TODO own strutc for packet?   
uint8_t dataPacket[PACKET_LENGTH] = {'$', 0x99, 0, 0, 0, 0, 0, 0, 0, 0, 0,
#ifdef SEND_ACC
0, 0, 0, 0, 0, 0,
#endif
0x00, 0x00, '\r', '\n'};
#else
*/
        dataPacket[3] = 'g';
        dataPacket[4] = 'g';
        dataPacket[5] = gx_offset & 0xFF;
        dataPacket[6] = gx_offset >> 8;
        dataPacket[7] = gy_offset & 0xFF;
        dataPacket[8] = gy_offset >> 8;
        dataPacket[9] = gz_offset & 0xFF;
        dataPacket[10] = gz_offset >> 8;
        
        // MASTER_SERIAL_NAME.write(dataPacket, PACKET_LENGTH);
        // MASTER_SERIAL_NAME.write((byte)0x00);    
        dataPacket[PACKET_COUNTER_POSITION]++;
        #ifdef SEND_ACC
            dataPacket[3] = 'g';
            dataPacket[4] = 'g';
            dataPacket[5] = ax_offset & 0xFF;
            dataPacket[6] = ax_offset >> 8;
            dataPacket[7] = ay_offset & 0xFF;
            dataPacket[8] = ay_offset >> 8;
            dataPacket[9] = az_offset & 0xFF;
            dataPacket[10] = az_offset >> 8;
            
            MASTER_SERIAL_NAME.write(dataPacket, PACKET_LENGTH);
            MASTER_SERIAL_NAME.write((byte)0x00);    
            // dataPacket[PACKET_COUNTER_POSITION]++;
        #endif
        
        return;
    }
    #endif
    
    //MASTER_SERIAL_NAME.write(cmdPacket, CMD_PACKET_LENGTH);
    //MASTER_SERIAL_NAME.write((byte)0x00);
}

#ifdef SLAVE_HAND

void slaveHandDataRequestHandler() {
    int limit = REPEAT_SLAVE_HAND_READ_LIMIT;
    readAlign = 0;
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
                    if(ch >= '0' && ch <= '6'){
                        ch = ch - 48;
                    }
                    //DEBUG_PRINTLN("al");
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
                    //DEBUG_PRINTLN("exec Command");
                    // deal with command packet ..
                    readAlign=0;
                    while(limit > 0) {
                        int chc = MASTER_SERIAL_NAME.read();
                        //DEBUG_PRINTLN(chc);
                        if (chc != -1) {
                            //exec Command((char)chc);
                            // TODO fix exec Command's cmdPacket 
                            execCommand();
                            limit = -1;
                            break;
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

boolean masterHandCommandRequestHandler(
        uint8_t* limit,
        uint8_t* endOfPacketAlignIn,
        int8_t* readAlign,
        bool* sendToSlaveIn,
        uint8_t* sentPacketCharCounterIn,
        uint8_t* alignIn, 
        int8_t * ch
    ){
    //GAG_DEBUG_PRINTLN("slaveHandDataRequestHandler");
    // TODO save memory?
    uint8_t endOfPacketAlign = *endOfPacketAlignIn;
    bool sendToSlave = *sendToSlaveIn;
    uint8_t sentPacketCharCounter = *sentPacketCharCounterIn;
    uint8_t align = *alignIn;

    //int ch = MASTER_SERIAL_NAME.read();
    *ch = (int8_t) MASTER_SERIAL_NAME.read();
    // int ch2 = Serial.read();
    // if (ch2 != -1) {
        // ch = ch2;
    // }
  
    if (*ch != -1) {
        GAG_DEBUG_PRINT(*ch);
        if(*readAlign<=0) {
            if(*ch == 'C' || *ch == 'c') {
                GAG_DEBUG_PRINT("received - C|c: ");
                GAG_DEBUG_PRINTLN((char)(*ch));
                if(*ch == 'c') {
                    sendToSlave = true;
                }
                // GAG_DEBUG_PRINT("parsing cmdPacket1 ");
                // GAG_DEBUG_PRINT((int) *readAlign);
                // GAG_DEBUG_PRINT(" ");
                // GAG_DEBUG_PRINT((int) *limit);
                // GAG_DEBUG_PRINT(" ");
                // GAG_DEBUG_PRINTLN((char)cmdPacket[*readAlign]);
                (*readAlign)++;
                (*limit) += CMD_PACKET_LENGTH+1;
                // GAG_DEBUG_PRINT("parsing cmdPacket2 ");
                // GAG_DEBUG_PRINT((int) *readAlign);
                // GAG_DEBUG_PRINT(" ");
                // GAG_DEBUG_PRINT((int) *limit);
                // GAG_DEBUG_PRINT(" ");
                // GAG_DEBUG_PRINTLN((char)cmdPacket[*readAlign]);
            } else {
                *readAlign = 0;
            }
        } /*else {
            if(*ch == '\r') {
                *readAlign=254;
            }
            if(*ch == '\n' && *readAlign == 254) {
                *readAlign=255;
            }
        }*/
        if(*readAlign > 0) {
            // GAG_DEBUG_PRINTLN("*readAlign > 0");
            if(sendToSlave) {
                SLAVE_SERIAL_NAME.write(*ch);
            } else {
                // GAG_DEBUG_PRINTLN("parsing cmdPacket");
                *readAlign=0;
                cmdPacket[0]=*ch;
                // GAG_DEBUG_PRINT("Limit ");
                // GAG_DEBUG_PRINTLN((int) *limit);
                while((*limit) > 0) {
                    int chc = MASTER_SERIAL_NAME.read();
                    if (chc != -1) {
                        (*readAlign)++;
                        cmdPacket[*readAlign]=chc;
                        // GAG_DEBUG_PRINT("parsing cmdPacket ");
                        // GAG_DEBUG_PRINT((char)chc);
                        // GAG_DEBUG_PRINT(" ");
                        // GAG_DEBUG_PRINT((int) *readAlign);
                        // GAG_DEBUG_PRINT(" ");
                        // GAG_DEBUG_PRINT((int) *limit);
                        // GAG_DEBUG_PRINT(" ");
                        // GAG_DEBUG_PRINTLN((char)cmdPacket[*readAlign]);
                        //if(chc == '\n' && *readAlign==(CMD_PACKET_LENGTH-1) && cmdPacket[CMD_PACKET_LENGTH-2] == '\r') {
                        //if(chc == '\n' && *readAlign==(CMD_PACKET_LENGTH-1)) {
                        if(chc == '\n') {
                            execCommand();
                            (*readAlign) = 0;
                            (*limit) = 0;
                            return true;
                        }
                        if(*readAlign>CMD_PACKET_LENGTH){
                            // GAG_DEBUG_PRINT("*readAlign>CMD_PACKET_LENGTH ");
                            // *limit = -1;
                        }
                    }
                    (*limit)--;
                }
                return true;
            }
        }
        // if(*readAlign == 255){
            // return true;
        // }
    }
    (*limit)--;
    return true;
}

void masterHandDataRequestHandler() {
    //GAG_DEBUG_PRINTLN("slaveHandDataRequestHandler");
    // TODO save memory?
    uint8_t limit = REPEAT_MASTER_HAND_READ_LIMIT;
    uint8_t endOfPacketAlign = 0;
    readAlign = 0;
    bool sendToSlave = false;
    uint8_t sentPacketCharCounter = 0;
    uint8_t align = 0;
    int8_t ch = 0;

    while(limit > 0) {
        if(masterHandCommandRequestHandler(
            &limit,
            &endOfPacketAlign,
            &readAlign,
            &sendToSlave,
            &sentPacketCharCounter,
            &align,
            &ch)) { break; }

        if (handSwitchElapsed > MAX_HAND_SWITCH_TIME ||
            sentPacketCharCounter > PACKET_LENGTH || endOfPacketAlign == 2) {
            handSwitchElapsed = 0;
            // TODO fix failing aligning differently then with sending flag character '+' for distinguishing packet end
            MASTER_SERIAL_NAME.write((byte)0x00);
            // TODO find out how many packets are lost
            limit =-1;
            break;
        }
     
        if (ch != -1) {
            if(ch == '$') {
                align++;
                sentPacketCharCounter++;
            }
            if(ch == 0x99) {
                align++;
                sentPacketCharCounter++;
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
        limit--;
    }
}

void loadSlaveHandData() {
    uint8_t limit = REPEAT_MASTER_HAND_READ_LIMIT;

    while(--limit>0){ 
        sendDataRequest(selectedSensor);
        handSwitchElapsed = 0;
        time2 = millis();
        timePrev2 = 0;
        uint8_t sentCharCounter = 0;
        uint8_t sentPacketCharCounter = 0;
        uint8_t align = 0;
        uint8_t endOfPacketAlign=0;
        
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