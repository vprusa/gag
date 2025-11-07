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

#define HP 10000

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
        GAG_DEBUG_PRINTLN(SerialBT.serialBuffer[SerialBT.serialBufferWriteIndex][0]);

        if(++SerialBT.serialBufferWriteIndex >= BT_SERIAL_BUFFER_SIZE){
            SerialBT.serialBufferWriteIndex=0;}

        GAG_DEBUG_PRINTLN("Buffer: ");
        for(char i =0; i<BT_SERIAL_BUFFER_SIZE; i++ ) {
            for(char ii =0; ii<CMD_PACKET_LENGTH; ii++ ) {
                GAG_DEBUG_PRINT(SerialBT.serialBuffer[i][ii]);
                GAG_DEBUG_PRINT(" ");
            }
            GAG_DEBUG_PRINTLN("");
        }
    }
};
#endif

enum Sensor {
    SENSOR_PIN_TU = 0, //5,//0,
    SENSOR_PIN_SU = 1,
    SENSOR_PIN_FU = 2,
    SENSOR_PIN_MU = 3,
    SENSOR_PIN_EU = 4,
    SENSOR_PIN_HG = 5, //0, //5, // hand palm
    // SENSOR_PIN_HP = 6, // hand palm MPU6050
    SENSOR_PIN_NF = -1,
};

struct Gyro {
    // uint8_t fifoBuffer[FIFO_SIZE]; // FIFO storage buffer
    uint8_t* fifoBuffer; // FIFO storage buffer
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
        {-2772,479,2063,119,-49,120},
        // I
        {-3678,1461,1496,46,-43,-3},
        // M
        {507,1346,1743,20,26,-19},
        // L
        {-3208, -1979, 1487, 47, -22, 0},
        // R
        {-1802,1287,1493,35,-19,6},

        // WG unknown        
        {1659, 1620, 1108, 62, -23, -1},

        // W
        {-147, -84, 737, 41, -3, -4}

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
        // #ifdef SENSOR_PIN_HP_COMPENSATION
        // case SENSOR_PIN_HP:
            // selectorOffsettedPin = SENSOR_PIN_HP_COMPENSATION;
        // break;
        // #endif
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

// Global variables to store gyro offsets
float gx_offset3 = 0, gy_offset3 = 0, gz_offset3 = 0;
float ax_offset3 = 0, ay_offset3 = 0, az_offset3 = 0;
float mx_offset3 = 0, my_offset3 = 0, mz_offset3 = 0;
bool offsets_calculated = false;

// Function to calculate gyro offsets
// void calibrateGyro(MPU6050_MPU9150 *mpu, int numSamples = 20) {
void calibrateGyro2(MPU6050_MPU9150 *mpu, int numSamples = 100, bool debug = false) {
      int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;

    for (int i = 0; i < numSamples; i++) {
        mpu->getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;

        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;

        if (debug) {
            Serial.print("Sample ");
            Serial.print(i + 1);
            Serial.print(" | gx: "); Serial.print(gx);
            Serial.print(", gy: "); Serial.print(gy);
            Serial.print(", gz: "); Serial.println(gz);
        }

        delay(15); // Stable readings with shorter delay
    }

    // Compute average offsets
    gx_offset3 = gx_sum / numSamples;
    gy_offset3 = gy_sum / numSamples;
    gz_offset3 = gz_sum / numSamples;

    ax_offset3 = ax_sum / numSamples;
    ay_offset3 = ay_sum / numSamples;
    az_offset3 = az_sum / numSamples;

    offsets_calculated = true;

    // Print final calculated offsets
    Serial.println("Calibration complete:");
    Serial.print("Gyro Offsets -> gx: "); Serial.print(gx_offset3);
    Serial.print(", gy: "); Serial.print(gy_offset3);
    Serial.print(", gz: "); Serial.println(gz_offset3);

    Serial.print("Accel Offsets -> ax: "); Serial.print(ax_offset3);
    Serial.print(", ay: "); Serial.print(ay_offset3);
    Serial.print(", az: "); Serial.println(az_offset3);
    

    // Print calculated offsets
    Serial.print("");
    Serial.print("gx -= "); Serial.print(gx_offset3);
    Serial.print("; gy -= "); Serial.print(gy_offset3);
    Serial.print("; gz -= "); Serial.print(gz_offset3);
    Serial.print("; ax -= "); Serial.print(ax_offset3);
    Serial.print("; ay -= "); Serial.print(ay_offset3);
    Serial.print("; az -= "); Serial.print(az_offset3);
 

    offsets_calculated = true; // Mark that calibration is complete
}

void calibrateGyro(MPU6050_MPU9150 *mpu, int numSamples = 500, int magCalibrationTimeSec = 15, bool debug = true) {
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;

    // Gyro and Accelerometer calibration
    for (int i = 0; i < numSamples; i++) {
        mpu->getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

        if ((gx == 0 || gy == 0 || gz == 0 || gx == -1 || gy == -1 || gz == -1 || ax == -1 || ay == -1 || az == -1 || ax == 0 || ay == 0 || az == 0)) {
          continue;
        }
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;

        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;

        if (debug) {
            Serial.print("Sample ");
            Serial.print(i + 1);
            Serial.print(" | gx: "); Serial.print(gx);
            Serial.print(", gy: "); Serial.print(gy);
            Serial.print(", gz: "); Serial.print(gz);
            Serial.print(" | ax: "); Serial.print(ax);
            Serial.print(", ay: "); Serial.print(ay);
            Serial.print(", az: "); Serial.println(az);
        }

        delay(100);
    }

    gx_offset3 = gx_sum / numSamples;
    gy_offset3 = gy_sum / numSamples;
    gz_offset3 = gz_sum / numSamples;

    ax_offset3 = ax_sum / numSamples;
    ay_offset3 = ay_sum / numSamples;
    az_offset3 = az_sum / numSamples;

    Serial.println("Gyro and Accelerometer calibration complete:");
    Serial.print("Gyro Offsets -> gx: "); Serial.print(gx_offset3);
    Serial.print(", gy: "); Serial.print(gy_offset3);
    Serial.print(", gz: "); Serial.println(gz_offset3);

    Serial.print("Accel Offsets -> ax: "); Serial.print(ax_offset3);
    Serial.print(", ay: "); Serial.print(ay_offset3);
    Serial.print(", az: "); Serial.println(az_offset3);

    // Magnetometer calibration
    int16_t mx_min = INT16_MAX, my_min = INT16_MAX, mz_min = INT16_MAX;
    int16_t mx_max = INT16_MIN, my_max = INT16_MIN, mz_max = INT16_MIN;

    Serial.println("Rotate sensor slowly in all directions for magnetometer calibration...");
    unsigned long startTime = millis();

    while ((millis() - startTime) < magCalibrationTimeSec * 1000) {
        mpu->getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

        mx_min = min(mx_min, mx);
        my_min = min(my_min, my);
        mz_min = min(mz_min, mz);

        mx_max = max(mx_max, mx);
        my_max = max(my_max, my);
        mz_max = max(mz_max, mz);

        delay(50); // Slightly longer delay to allow full rotation
    }

    mx_offset3 = (mx_max + mx_min) / 2;
    my_offset3 = (my_max + my_min) / 2;
    mz_offset3 = (mz_max + mz_min) / 2;

    Serial.println("Magnetometer calibration complete:");
    Serial.print("Offsets -> mx: "); Serial.print(mx_offset3);
    Serial.print(", my: "); Serial.print(my_offset3);
    Serial.print(", mz: "); Serial.println(mz_offset3);

  // Print calculated offsets
    Serial.print("");
    Serial.print("gx -= "); Serial.print(gx_offset3);
    Serial.print("; gy -= "); Serial.print(gy_offset3);
    Serial.print("; gz -= "); Serial.print(gz_offset3);
    Serial.print("; ax -= "); Serial.print(ax_offset3);
    Serial.print("; ay -= "); Serial.print(ay_offset3);
    Serial.print("; az -= "); Serial.print(az_offset3);
    Serial.print("; mx -= "); Serial.print(mx_offset3);
    Serial.print("; my -= "); Serial.print(my_offset3);
    Serial.print("; mz -= "); Serial.print(mz_offset3);
    Serial.println(";");
    
    offsets_calculated = true;
}


void rotateQuaternionZ180(Quaternion &quat);
void loadHGData(int selectedSensor);
Quaternion q = {0.0f, 0.0f, 0.0f, 1.0f};  // Identity quaternion

void setupSensors() {

    // Rotate by 90 degrees around Z-axis once at initialization
    // rotateQuaternionZ180(q);
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
            gyros[selectedSensor].fifoBuffer = new uint8_t[FIFO_SIZE_MPU9250];
            gyros[selectedSensor].mpu->isMPU9150 = true;
        } else{
          gyros[selectedSensor].fifoBuffer = new uint8_t[FIFO_SIZE_MPU6050];
        }

        int selectorOffsettedPin = selectSingleMPU(i);

        MASTER_SERIAL_NAME.print(F("selectedSensor: "));
        MASTER_SERIAL_NAME.print((int)selectedSensor);
        MASTER_SERIAL_NAME.println(F(""));

        MASTER_SERIAL_NAME.print(F("Enabled on pin: "));
        MASTER_SERIAL_NAME.print(selectorOffsettedPin);
        MASTER_SERIAL_NAME.println(F(""));
        
        initMPUAndDMP(1, i);

        if(i == HG) {
          loadHGData(HG);
        }
         
        MASTER_SERIAL_NAME.println(F("\n\n"));
    }
}

void loadHGData(int selectedSensor) {
   Serial.println(" !!!!! ");
    Serial.println("Quaternion: ");
          //  gyros[selectedSensor].mpu->dmpGetGyroSensor()
           MPU6050_MPU9150 mpu = *gyros[selectedSensor].mpu;
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
                newQ->normalize();

                gyros[selectedSensor].q = newQ;
                gyros[selectedSensor].hasDataReady=true;
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
    // delay(1);
    for (i = 0; i < SENSORS_COUNT; i++) {
        if ( i == sensorToEnable ) {
            digitalWrite(selectSingleMPU(i), LOW);     
        }
    }
    // delay(1);
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
        Serial.println("devStatus");
        Serial.println(devStatus);
        mpu.setDMPEnabled(true);
        mpu.setFIFOEnabled(true);
        mpu.resetFIFO();
        //devStatus = mpu.dmpInitialize();
        MASTER_SERIAL_NAME.print(F("Skipping initialing DMP for MPU9150..."));
    }
    // supply your own gyro offsets here for each mpu, scaled for min sensitivity
    // lets ignore this considering we want realtive values anyway
    
    #ifdef SET_OFFSETS
    if(selectedSensor == HP){
        mpu.setXAccelOffset(sensorsOffsets[i][0], MPU9150_RA_XA_OFFS_H);
        mpu.setYAccelOffset(sensorsOffsets[i][1], MPU9150_RA_YA_OFFS_H);
        mpu.setZAccelOffset(sensorsOffsets[i][2], MPU9150_RA_ZA_OFFS_H);
        mpu.setXGyroOffset(sensorsOffsets[i][3], MPU9150_RA_XG_OFFS_USRH);
        mpu.setYGyroOffset(sensorsOffsets[i][4], MPU9150_RA_YG_OFFS_USRH);
        mpu.setZGyroOffset(sensorsOffsets[i][5], MPU9150_RA_ZG_OFFS_USRH);
    }else{
        mpu.setXAccelOffset(sensorsOffsets[i][0]);
        mpu.setYAccelOffset(sensorsOffsets[i][1]);
        mpu.setZAccelOffset(sensorsOffsets[i][2]);
        mpu.setXGyroOffset(sensorsOffsets[i][3]);
        mpu.setYGyroOffset(sensorsOffsets[i][4]);
        mpu.setZGyroOffset(sensorsOffsets[i][5]);
    }
        
    #endif

    // make sure it worked (returns 0 if so)

    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop( ) function knows it's okay to use it
    MASTER_SERIAL_NAME.println(F("DMP ready! Getting packet size..."));
    MASTER_SERIAL_NAME.print(F("packet size: "));

    if(selectedSensor == HP) {
        MASTER_SERIAL_NAME.print(packetSizeM);
        mpu.setFIFOEnabled(true);
        // calibrateQuaternionOffset(&mpu);
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
    float driftGain = 0.1f;
    gx_offset2 += ex * driftGain;
    gy_offset2 += ey * driftGain;
    gz_offset2 += ez * driftGain;
}


// Function to correct rotation using magnetometer data
void correctRotationWithMag(int16_t mx, int16_t my, int16_t mz) {
    // Normalize magnetometer data
    float magNorm = sqrt(mx * mx + my * my + mz * mz);
    float mx_n = mx / magNorm;
    float my_n = my / magNorm;
    float mz_n = mz / magNorm;

    // Define reference magnetic vector (e.g., magnetic north along the x-axis)
    const float ref_mx = 1.0f, ref_my = 0.0f, ref_mz = 0.0f;

    // Compute error (cross product between measured and reference vectors)
    float ex = (my_n * ref_mz - mz_n * ref_my);
    float ey = (mz_n * ref_mx - mx_n * ref_mz);
    float ez = (mx_n * ref_my - my_n * ref_mx);

    // Apply correction (proportional feedback)
    const float magGain = 0.05f;
    gx_offset2 += ex * magGain;
    gy_offset2 += ey * magGain;
    gz_offset2 += ez * magGain;
}

void updateQuaternion(int16_t gx, int16_t gy, int16_t gz) {
    // gx -= gx_offset2 + gx_offset3;
    // gy -= gy_offset2 + gy_offset3;
    // gz -= gz_offset2 + gz_offset3;
    gx -= gx_offset2 ;
    gy -= gy_offset2 ;
    gz -= gz_offset2 ;

    
    // Convert gyro readings from degrees/s to radians/s
    float gyroScale = 131.0;  // Scale factor for ±250°/s
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0f;  // Convert ms to seconds
    // Serial.println(dt);
    lastTime = currentTime;  // Update time for next iteration
        if (dt <= 0.00) return; 
    const float gyro_scale = 131.0f;  // ±250°/s scale factor
    const float deg_to_rad = 3.14159265359f / 180.0f;
    
    // Fix sensor rotation by rotating raw gyro values around Z-axis by -90°
    int16_t new_gx = gy;  // Rotated -90°: gx' = -gy
    int16_t new_gy = -gx;   // Rotated -90°: gy' = gx
    int16_t new_gz = gz;   // Z remains the same


    float gx_r = (new_gx / gyro_scale) * deg_to_rad;
    float gy_r = (new_gy / gyro_scale) * deg_to_rad;
    float gz_r = (new_gz / gyro_scale) * deg_to_rad;

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
    q.normalize();
}


void storeQuaternionInFIFO() {
    int16_t qI[4];

    // Use correct scaling (16384 like dmpGetQuaternion expects)
    qI[0] = (int16_t)(q.w * 16384.0f);
    qI[1] = (int16_t)(q.x * 16384.0f);
    qI[2] = (int16_t)(q.y * 16384.0f);
    qI[3] = (int16_t)(q.z * 16384.0f);

    // Store in FIFO using correct order (Big Endian)
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

// Define quaternion offset
float q_offset_w = 0.0f, q_offset_x = 0.0f, q_offset_y = 0.0f, q_offset_z = 0.0f;

// Number of samples for calibration
#define CALIBRATION_SAMPLES 100

int16_t ax_c, ay_c, az_c, gx_c, gy_c, gz_c, mx, my, mz;

void rotateQuaternionZ90(Quaternion &quat) {
    // Rotation quaternion for 90 degrees (π/2 radians) around Z-axis
    const float angle = PI / 2.0f;
    Quaternion rotation;
    rotation.w = cos(angle / 2.0f);
    rotation.x = 0.0f;
    rotation.y = 0.0f;
    rotation.z = sin(angle / 2.0f);

    // Perform quaternion multiplication: quat = rotation * quat
    Quaternion result;
    result.w = rotation.w * quat.w - rotation.x * quat.x - rotation.y * quat.y - rotation.z * quat.z;
    result.x = rotation.w * quat.x + rotation.x * quat.w + rotation.y * quat.z - rotation.z * quat.y;
    result.y = rotation.w * quat.y - rotation.x * quat.z + rotation.y * quat.w + rotation.z * quat.x;
    result.z = rotation.w * quat.z + rotation.x * quat.y - rotation.y * quat.x + rotation.z * quat.w;

    quat = result;
}


void rotateQuaternionZ180(Quaternion &quat) {
    Quaternion rotation;
    rotation.w = 0.0f;
    rotation.x = 0.0f;
    rotation.y = 0.0f;
    rotation.z = 1.0f;

    Quaternion result;
    result.w = rotation.w * quat.w - rotation.x * quat.x - rotation.y * quat.y - rotation.z * quat.z;
    result.x = rotation.w * quat.x + rotation.x * quat.w + rotation.y * quat.z - rotation.z * quat.y;
    result.y = rotation.w * quat.y - rotation.x * quat.z + rotation.y * quat.w + rotation.z * quat.x;
    result.z = rotation.w * quat.z + rotation.x * quat.y - rotation.y * quat.x + rotation.z * quat.w;

    quat = result;
}

void loadMPU9150Data(MPU6050_MPU9150 *mpu) {
      int16_t ax, ay, az, gx, gy, gz, mx, my, mz;

 // Get sensor data
     mpu->getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);



    gx -= -148;
    gy -= 84;
    gz -= -150; 

    ax -= 266;
    ay -= -1104;
    az -= -320; 

    mx -= 28;
    my -= 650;
    mz -= 0; 

    // Update quaternion from gyro data
    updateQuaternion(gx, gy, gz);
    correctDriftWithAccel(ax, ay, az);
    correctRotationWithMag(mx, my, mz);
    storeQuaternionInFIFO();

}



void loadMPU60500Data(MPU6050_MPU9150 *mpu) {
  int16_t ax, ay, az, gx, gy, gz, mx, my, mz;

  // Get sensor data
  mpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Update quaternion from gyro data
  updateQuaternion(gx, gy, gz);
  correctDriftWithAccel(ax, ay, az);

  storeQuaternionInFIFO();

}

bool loadDataFromFIFO(bool forceLoad) {
    if(selectedSensor != HP) {
        MPU6050_MPU9150 * mpu = gyros[selectedSensor].mpu;


        if(!gyros[selectedSensor].hasDataReady || forceLoad) {
            fifoCount = mpu->getFIFOCount();
            uint8_t *fifoBuffer = gyros[selectedSensor].fifoBuffer; // FIFO storage buffer
            int packetSize = packetSizeS; 


            if (fifoCount >= packetSize && fifoCount <= 1024 && fifoCount != 0 ) {
                // wait for correct available data length, should be a VERY short wait
                while (fifoCount >= packetSize) {
                    mpu->getFIFOBytes(fifoBuffer, packetSize);
                    fifoCount -= packetSize;
                }
                Quaternion sensorQ;
                uint8_t res = mpu->dmpGetQuaternion(&sensorQ,fifoBuffer);
                sensorQ.normalize();
                if (selectedSensor == HG) {

                  Quaternion rotatedQ = Quaternion(sensorQ.w, -sensorQ.y, -sensorQ.x, sensorQ.z);
                  rotatedQ.normalize();

                  // Save rotated quaternion
                  gyros[selectedSensor].q = new Quaternion(rotatedQ);
                  q.w = gyros[selectedSensor].q->w;
                  q.x = gyros[selectedSensor].q->x;
                  q.y = gyros[selectedSensor].q->y;
                  q.z = gyros[selectedSensor].q->z;
                  storeQuaternionInFIFO();

                } else {
                  gyros[selectedSensor].q = new Quaternion(sensorQ);
                }

                //mpu.resetFIFO();
                gyros[selectedSensor].hasDataReady=true;
                return true;
            }
        }
    } else {
        forceLoad = true;
     
          loadMPU9150Data(gyros[selectedSensor].mpu);
          gyros[selectedSensor].hasDataReady=true;
          gyros[selectedSensor].alreadySentData=false;
          return true;
        // }
    }
    return false;
}

void writePacket() {
    uint8_t *fifoBuffer = gyros[selectedSensor].fifoBuffer; // FIFO storage buffer
#ifdef MASTER_HAND
    if(!gyros[selectedSensor].alreadySentData && gyros[selectedSensor].hasDataReady) {
        fifoToPacket(fifoBuffer, dataPacket, selectedSensor);
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
                (*readAlign)++;
                (*limit) += CMD_PACKET_LENGTH+1;
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
                *readAlign=0;
                cmdPacket[0]=*ch;
                while((*limit) > 0) {
                    int chc = MASTER_SERIAL_NAME.read();
                    if (chc != -1) {
                        (*readAlign)++;
                        cmdPacket[*readAlign]=chc;
                        if(chc == '\n') {
                            execCommand();
                            (*readAlign) = 0;
                            (*limit) = 0;
                            return true;
                        }
                    }
                    (*limit)--;
                }
                return true;
            }
        }
    }
    (*limit)--;
    return true;
}

void masterHandDataRequestHandler() {
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