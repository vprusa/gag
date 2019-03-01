// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
/*
Copyright (c) 2018 Vojtěch Průša
*/
// Based on example of MPU6050 from https://github.com/jrowberg/i2cdevlib
// from 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// 

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low =e0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using MASTER_SERIAL_NAME.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// TODO switch usage usage of pins for BT_BAUD and USB_BAUD so communication between hands would go over usb and communication to pc/ntb would use AltSoftSerial


// Hand Switch: 
// uncomment for using left hand or comment for using right hand
#define LEFT_HAND

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
#define OUTPUT_TEAPOT
#define OUTPUT_TEAPOT_REALACCEL

//#define SEND_ACC

#ifdef SEND_ACC
#define PACKET_LENGTH 21
#define PACKET_COUNTER_POSITION 18
#else
#define PACKET_LENGTH 15
#define PACKET_COUNTER_POSITION 10
#endif

#define FINGERS_COUNT 6
// fingers <0,4>
#define FIRST_FINGER 0
// or set LAST_FINGER to 4
#define LAST_FINGER 5
// time for internal interrupt to trigger in loop - working up to 50 ms but freezes may occure - so reset MPU's FIFO more ften (20ms each?)
#define SWITCH_FINGERS_MS 0
#define FINGER_PIN_TU TU
#define FINGER_PIN_SU SU
#define FINGER_PIN_FU FU
#define FINGER_PIN_MU MU
#define FINGER_PIN_EU EU
#define FINGER_PIN_HP HP
#define FINGER_PIN_HP_COMPENSATION 13
#define FINGER_PIN_NF NF

//#define FINGER_PIN_OFFSET 3
#define FINGER_PIN_OFFSET 3
// 57600 115200
#define BT_BAUD 57600
// TODO do not use this at all
#define MAX_HAND_SWITCH_TIME 25
// do not forget for ShanonKotelnik theorem where MAX_HAND_SWITCH_CHARS >= PACKET_LENGTH*2
#define MAX_HAND_SWITCH_CHARS 35
// TODO make left/right switch

// TODO test if 2 should is enough
#define REPEAT_RIGHT_HAND_READ_LIMIT 10
#define REPEAT_LEFT_HAND_READ_LIMIT 10

// (9600 38400 57600 74880 115200 230400 250000 57600 38400 chosen because it is required for Teapot Demo output, but it's

#ifdef LEFT_HAND
#define MAX_TIME_TO_RESET 50
#define MIN_TIME_TO_RESET 30
#define MAX_FIFO_USAGE_FOR_RESET 900
#define MIN_FIFO_USAGE_FOR_RESET 500
#define FIFO_PACKET_SIZE 42
#define FIFO_SIZE 42
//#define USE_BT
#define USE_USB
#define USE_BT_MASTER
#else
#define RIGHT_HAND
#define RIGHT_HAND_SLAVE
#define RIGHT_HAND_TO_PC

#define MAX_TIME_TO_RESET 50
#define MIN_TIME_TO_RESET 20
#define MAX_FIFO_USAGE_FOR_RESET 900
#define MIN_FIFO_USAGE_FOR_RESET 500
#define FIFO_PACKET_SIZE 42
#define FIFO_SIZE 42
#endif
//#define USE_BT

#ifdef RIGHT_HAND
//#define USE_BT
#define USE_USB
#endif
//#define USE_BT

//#define LIB_SW_SERIAL
#define LIB_ALT_SW_SERIAL 1

// 115200 57600
#ifdef LEFT_HAND
#define USB_BAUD 57600
#define BT_BAUD 115200
#define MASTER_SERIAL_NAME hc05Master
#define PC_SERIAL_NAME Serial
#else
#define USB_BAUD 115200
//#define BT_BAUD 57600
#define MASTER_SERIAL_NAME Serial
//#define PC_SERIAL_NAME hc05Master
#endif

#ifdef LEFT_HAND
#ifdef USE_BT_MASTER
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
#endif

// MPU control/status vars
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;   // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize = FIFO_PACKET_SIZE; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;  // count of all bytes currently in FIFO

// packet structure for InvenSense teapot demo
#ifdef RIGHT_HAND
uint8_t teapotPacket[PACKET_LENGTH] = {'$', 0x99, 0, 0, 0, 0, 0, 0, 0, 0, 0,
#ifdef SEND_ACC
                                       0, 0, 0, 0, 0, 0,
#endif
                                       0x00, 0x00, '\r', '\n'};
#else
uint8_t teapotPacket[PACKET_LENGTH] = {'*', 0x99, 0, 0, 0, 0, 0, 0, 0, 0, 0,
#ifdef SEND_ACC
                                       0, 0, 0, 0, 0, 0,
#endif
                                       0x00, 0x00, '\r', '\n'};
#endif

enum Finger
{
    FINGER_PIN_TU = 0,
    FINGER_PIN_SU = 1,
    FINGER_PIN_FU = 2,
    FINGER_PIN_MU = 3,
    FINGER_PIN_EU = 4,
    FINGER_PIN_HP = 5, // hadn palm
    FINGER_PIN_NF=-1,
    
};
char emptyArr[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
#ifdef SEND_ACC
0, 0, 0, 0, 0, 0,
#endif
0x00, 0x00, 0, 0};
struct Gyro
{
    uint8_t fifoBuffer[FIFO_SIZE];                                      // FIFO storage buffer
    int AD0_pin;
    MPU6050 *mpu;
    // orientation/motion vars
    Quaternion q;        // [w, x, y, z]         quaternion container
    bool dmpReady = false; // set true if DMP init was successful
    bool hasDataReady = false;
    bool alreadySentData = false;
    long lastResetTime=0;
};

Finger selectedFinger = FINGER_PIN_NF;

//Variables
float elapsedTime, time, timePrev, elapsedTimeToSwitch, handSwitchPrev, handSwitchElapsed; //Variables for time control
Gyro gyros[FINGERS_COUNT];

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void enableSingleMPU(int fingerToEnable)
{
    for (int i = 0; i < FINGERS_COUNT; i++)
    {
        int fingerToEnableOffsetted = FINGER_PIN_OFFSET + i;
        // Because Non-firing contact field on right hand has broken contact on pin D8  
        if (i == FINGER_PIN_HP) {
            fingerToEnableOffsetted = FINGER_PIN_HP_COMPENSATION;
        }
        if (i == fingerToEnable)
        {
            digitalWrite(fingerToEnableOffsetted, LOW);
        }
        else
        {
            digitalWrite(fingerToEnableOffsetted, HIGH);
        }
    }
}

void setup()
{
    for (int i = FIRST_FINGER; i <= LAST_FINGER; i++)
    {
        int fingerToEnable = FINGER_PIN_OFFSET + i;
        if (i == FINGER_PIN_HP) {
            fingerToEnable = FINGER_PIN_HP_COMPENSATION;
        }
        pinMode(fingerToEnable, OUTPUT);
    }

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

// initialize serial communication
// (9600 38400 57600 74880 115200 230400 250000 57600 38400 chosen because it is required for Teapot Demo output, but it's
// really up to you depending on your project)
#ifdef USE_BT_MASTER
    PC_SERIAL_NAME.begin(BT_BAUD);
    // while (!hc05.available()){}
    // hc05Master.println(F("BT up"));
#endif
#ifdef USE_USB
    MASTER_SERIAL_NAME.begin(USB_BAUD);
    #ifdef RIGHT_HAND
    while (!MASTER_SERIAL_NAME)
        ; // wait for Leonardo enumeration, others continue immediately
    #endif
    //Serial.write('');
    MASTER_SERIAL_NAME.println(F("USB up"));
#endif

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    for (int i = FIRST_FINGER; i <= LAST_FINGER; i++)
    {
        selectedFinger = (Finger)i;
        enableSingleMPU(selectedFinger);
        gyros[selectedFinger].mpu = new MPU6050();
        initMPUAndDMP(1);
    }

    time = millis(); //Start counting time in milliseconds
}

int initMPUAndDMP(int attempt)
{
    if (attempt <= 0)
    {
        return 0;
    }
// initialize device
#ifdef USE_BT
    hc05.println(F("BT: Initializing I2C devices..."));
#endif
#ifdef USE_USB
    MASTER_SERIAL_NAME.println(F("USB: Initializing I2C devices..."));
#endif
    MPU6050 mpu = *gyros[selectedFinger].mpu;
    mpu.initialize();
#ifdef USE_USB
    MASTER_SERIAL_NAME.println(F("Testing device connections..."));
    MASTER_SERIAL_NAME.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    MASTER_SERIAL_NAME.println(F("\nSend any character to begin DMP programming and demo: "));
    // load and configure the DMP
    MASTER_SERIAL_NAME.println(F("Initializing DMP..."));
#endif
    devStatus = mpu.dmpInitialize();
    MASTER_SERIAL_NAME.print(F("DMP initialized..."));

    // supply your own gyro offsets here for each mpu, scaled for min sensitivity
    // lets ignore this considering we want realtive values anyway
    //mpu.setXGyroOffset(220);
    //mpu.setYGyroOffset(76);
    //mpu.setZGyroOffset(-85);
    //mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    //if (devStatus == 0) {
    if (true) {
// turn on the DMP, now that it's ready
// initialize device
#ifdef USE_BT
        hc05.print(F("Enabling DMP... "));
        hc05.println(selectedFinger);
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        hc05.println(F("DMP ready! Getting packet size..."));
        gyros[selectedFinger].dmpReady = true;
        // get expected DMP packet size for later comparison
        hc05.print(F("packet size: "));
        hc05.print(packetSize);
        hc05.println(F(""));
#endif
#ifdef USE_USB
        MASTER_SERIAL_NAME.print(F("Enabling DMP... "));
        MASTER_SERIAL_NAME.println(selectedFinger);
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        MASTER_SERIAL_NAME.println(F("DMP ready! Getting packet size..."));
        gyros[selectedFinger].dmpReady = true;
        MASTER_SERIAL_NAME.print(F("packet size: "));
        MASTER_SERIAL_NAME.print(packetSize);
        MASTER_SERIAL_NAME.println(F(""));
#endif
    }
    else
    {
// ERROR!
// 1 = initial memory load failed
// 2 = DMP configuration updates failed
// (if it's going to break, usually the code will be 1)
#ifdef USE_BT
        hc05.print(F("DMP Initialization failed (code "));
        hc05.print(devStatus);
        hc05.println(F(")"));
#endif
#ifdef USE_USB
        MASTER_SERIAL_NAME.print(F("DMP Initialization failed (code "));
        MASTER_SERIAL_NAME.print(devStatus);
        MASTER_SERIAL_NAME.println(F(")"));
#endif
        initMPUAndDMP(attempt - 1);
    }
    return 0;
}

int setOrRotateSelectedGyro(int i)
{
    if (i == -1) {
        i = selectedFinger + 1;
    }
    if (i > LAST_FINGER) {
        i = FIRST_FINGER;
    }
    selectedFinger = (Finger)i;
    enableSingleMPU(selectedFinger);
    return i;
}

#ifdef OLD_RESET
void resetMPUs(int around)
{
    // reset n after (TODO without current?)
    if (around > 0) {
        for (int i = 0; i < around; i++) {
            MPU6050 mpu = *gyros[selectedFinger].mpu;
            mpu.resetFIFO();
            setOrRotateSelectedGyro(-1);
        }
        // rotate to start position
        for (int i = around; i < FINGERS_COUNT; i++) {
            setOrRotateSelectedGyro(-1);
        }
    }
    else
    {
        // reset n before current (TODO without current?)
        // rotate to position where reset should start
        int i = 0;
        for (; i < FINGERS_COUNT + around; i++) {
            setOrRotateSelectedGyro(-1);
        }
        for (; i < FINGERS_COUNT; i++) {
            MPU6050 mpu = *gyros[selectedFinger].mpu;
            mpu.resetFIFO();
            setOrRotateSelectedGyro(-1);
        }
    }
}
#endif

void automaticFifoReset(){
    long now = millis();
    int currentlySellectedSensor = selectedFinger;
    for(int i = 0; i < FINGERS_COUNT; i++){
        if(gyros[i].lastResetTime + MIN_TIME_TO_RESET < now 
        #ifdef RIGHT_HAND
        && gyros[i].hasDataReady
        #endif
        ) {
            int selectedNow = setOrRotateSelectedGyro(i);
            MPU6050 mpu = *gyros[selectedNow].mpu;
                
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

void fifoToPacket(byte * fifoBuffer, byte * packet, int selectedFinger){
    packet[2] = selectedFinger;
    packet[3] = fifoBuffer[0];
    packet[4] = fifoBuffer[1];
    packet[5] = fifoBuffer[4];
    packet[6] = fifoBuffer[5];
    packet[7] = fifoBuffer[8];
    packet[8] = fifoBuffer[9];
    packet[9] = fifoBuffer[12];
    packet[10] = fifoBuffer[13];
}

#ifdef USE_BT_MASTER
void sendDataRequest(int selectedFinger){
    PC_SERIAL_NAME.write('$');
    PC_SERIAL_NAME.write(selectedFinger);
    PC_SERIAL_NAME.write((byte)0x00); // 0x00 fails to compile
}
#endif

volatile int readAlign = 0;
#ifdef RIGHT_HAND_SLAVE 
volatile int readAligned = 0;
#endif
#ifdef LEFT_HAND 
volatile int readAligned = 1;
#endif
volatile float time2, timePrev2;

bool loadDataFromFIFO(int forceLoad = false){
    MPU6050 mpu = *gyros[selectedFinger].mpu;
    if(!gyros[selectedFinger].hasDataReady || forceLoad){
        fifoCount = mpu.getFIFOCount();
        uint8_t *fifoBuffer = gyros[selectedFinger].fifoBuffer; // FIFO storage buffer
        
        if (fifoCount >= packetSize && fifoCount <= 1024 && fifoCount != 0 ) // if (mpuIntStatus & 0x02) 
        {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount >= packetSize/* && _BV(MPU6050_INTERRUPT_DMP_INT_BIT)*/)
            {
                mpu.getFIFOBytes(fifoBuffer, packetSize);
                fifoCount -= packetSize;
            }
            gyros[selectedFinger].hasDataReady=true;
            return true;
        }
    }

    return false;
}

void writePacket(){
    uint8_t *fifoBuffer = gyros[selectedFinger].fifoBuffer; // FIFO storage buffer
#ifdef RIGHT_HAND
    if(!gyros[selectedFinger].alreadySentData && gyros[selectedFinger].hasDataReady){
        fifoToPacket(fifoBuffer, teapotPacket, selectedFinger);
        MASTER_SERIAL_NAME.write(teapotPacket, PACKET_LENGTH);
        MASTER_SERIAL_NAME.write(0x00);
        teapotPacket[PACKET_COUNTER_POSITION]++; // packetCount, loops at 0xFF on purpose
        gyros[selectedFinger].hasDataReady = false;
        gyros[selectedFinger].alreadySentData = true;
    }
    readAlign = 0;
    readAligned = 0;
#endif
#ifdef LEFT_HAND
    if(!gyros[selectedFinger].alreadySentData && gyros[selectedFinger].hasDataReady) {
        fifoToPacket(fifoBuffer, teapotPacket, selectedFinger);
        MASTER_SERIAL_NAME.write(teapotPacket, PACKET_LENGTH);
        MASTER_SERIAL_NAME.write((byte)0x00);
        gyros[selectedFinger].hasDataReady = false;
        gyros[selectedFinger].alreadySentData = true;
        teapotPacket[PACKET_COUNTER_POSITION]++; // packetCount, loops at 0xFF on purpose
    }
#endif        
}

void loadDataAndSendPacket() {
    loadDataFromFIFO(false);
    if(gyros[selectedFinger].hasDataReady) {

#ifdef USE_BT
        if (hc05.availableForWrite()) {
            for (int i = 0; i < PACKET_LENGTH; i++) {
                hc05.print((char)teapotPacket[i]);
            }
        }
#endif
#ifdef USE_USB
    writePacket();       
#endif
    }
}

#ifdef RIGHT_HAND_SLAVE     
void rightHandDataRequestHandler(){
   int limit = REPEAT_LEFT_HAND_READ_LIMIT;
    readAlign = 0;
    readAligned = 0;
    while(limit > 0) {
        int ch = MASTER_SERIAL_NAME.read();
        if (ch != -1) {
            if(readAlign<1) {
                if(ch == '$') {
                    readAlign=1;
                }
            }else {
                if(ch >= 0 && ch < FINGERS_COUNT) {
                    readAligned = 1;
                    readAlign=0;//++;
                    setOrRotateSelectedGyro(ch);

                    if(readAligned == 1){
                        gyros[selectedFinger].alreadySentData = false;
                        writePacket();
                        loadDataAndSendPacket();
                        int currentlySellectedSensor = selectedFinger;
                        setOrRotateSelectedGyro(-1);
                        loadDataFromFIFO(true);
                    // setOrRotateSelectedGyro(currentlySellectedSensor);
                    }

                    break;
                }else {
                    readAlign=0;
                }
            }    
            limit--;
        }
    }
}
#endif

#ifdef USE_BT_MASTER
void loadRightHandData(){
    int limit = REPEAT_RIGHT_HAND_READ_LIMIT;
    
    while(--limit>0){ 
        sendDataRequest(selectedFinger);
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

            int ch = PC_SERIAL_NAME.read();

            if (ch != -1) {
                if(ch == '$'){
                    align++;
                    sentPacketCharCounter++;
                }
                if(ch == 0x99){
                    align++;
                    sentPacketCharCounter++;
                }
                if(align > 1){
                    aligned = 1;
                }
                if(sentPacketCharCounter>0){
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

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{

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
    handSwitchPrev = time;
    timePrev = time; // the previous time is stored before the actual time read
    time = millis(); // actual time read
    elapsedTime = (time - timePrev);
 
    // reset interrupt flag and get INT_STATUS byte
    
#ifdef USE_BT_MASTER
        loadRightHandData();
#endif

#ifdef RIGHT_HAND_SLAVE 
    rightHandDataRequestHandler();
#endif
   
#ifdef LEFT_HAND
        gyros[selectedFinger].alreadySentData = false;
        //writePacket();
        loadDataAndSendPacket();
        int currentlySellectedSensor = selectedFinger;
        setOrRotateSelectedGyro(-1);
        loadDataFromFIFO(true);
        //resetMPUs(-3);
        //automaticFifoReset();

#endif
    automaticFifoReset();
#ifdef USE_BT_MASTER
    loadRightHandData();
#endif

}
