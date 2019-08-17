
/*
*/

#ifndef _GAG_H_
#define _GAG_H_

#include "MPU6050_MPU9250_9Axis_MotionApps41.h"
//#include "gag_display.h"
//#include "gag_offsetting.h"

#ifdef ESP32_RIGHT
#define MASTER_HAND
#define USE_DISPLAY 1
#else
#define SLAVE_HAND
#endif

//#define GAG_DEBUG
#ifdef GAG_DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif

#ifdef USE_DISPLAY
#include "gag_display.h"
#endif


#define MPU6050_FIFO_PACKET_SIZE 42
#define MPU9250_FIFO_PACKET_SIZE 48
// TODO fix..
#define FIFO_SIZE 48


#ifndef ESP32_RIGHT
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#endif

// Hand Switch: 
// uncomment for using left hand or comment for using right hand
#ifdef  ESP32_RIGHT
#else
#define LEFT_HAND
#endif

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
#ifdef ESP32_RIGHT

#define SENSORS_COUNT 6
// SENSORs <0,5>
#define FIRST_SENSOR 0
// or set LAST_SENSOR to 5
#define LAST_SENSOR 5
// time for internal interrupt to trigger in loop - working up to 50 ms but freezes may occure - so reset MPU's FIFO more ften (20ms each?)
#define SWITCH_SENSORS_MS 0

// https://randomnerdtutorials.com/esp32-pinout-reference-gpios/

#define SENSOR_PIN_TU TU
#define SENSOR_PIN_TU_COMPENSATION 14 //4
#define SENSOR_PIN_SU SU
#define SENSOR_PIN_SU_COMPENSATION 13 //14
#define SENSOR_PIN_FU FU
#define SENSOR_PIN_FU_COMPENSATION 12
#define SENSOR_PIN_MU MU
#define SENSOR_PIN_MU_COMPENSATION 4 //13
#define SENSOR_PIN_EU EU
#define SENSOR_PIN_EU_COMPENSATION 2
#define SENSOR_PIN_HP HP // HAND PALM
#define SENSOR_PIN_HP_COMPENSATION 15
#define SENSOR_PIN_NF NF

//#define SENSOR_PIN_OFFSET 3
#define SENSOR_PIN_OFFSET 0
// 57600 115200
//#define BT_BAUD 57600

#else
#define SENSORS_COUNT 6
// SENSORs <0,4>
#define FIRST_SENSOR 0
// or set LAST_SENSOR to 4
#define LAST_SENSOR 5
// time for internal interrupt to trigger in loop - working up to 50 ms but freezes may occure - so reset MPU's FIFO more ften (20ms each?)
#define SWITCH_SENSORS_MS 0

#define SENSOR_PIN_TU TU
#define SENSOR_PIN_TU_COMPENSATION 4 
#define SENSOR_PIN_SU SU
#define SENSOR_PIN_SU_COMPENSATION 7
#define SENSOR_PIN_FU FU
#define SENSOR_PIN_FU_COMPENSATION 10
#define SENSOR_PIN_MU MU
#define SENSOR_PIN_MU_COMPENSATION 11
#define SENSOR_PIN_EU EU
#define SENSOR_PIN_EU_COMPENSATION 12
#define SENSOR_PIN_HP HP // HAND PALM
#define SENSOR_PIN_HP_COMPENSATION 13
#define SENSOR_PIN_NF NF

//#define SENSOR_PIN_OFFSET 3
#define SENSOR_PIN_OFFSET 0
// 57600 115200
//#define PC_SERIAL_BAUD 115200

#endif

// TODO do not use this at all
#define MAX_HAND_SWITCH_TIME 25
// do not forget for ShanonKotelnik theorem where MAX_HAND_SWITCH_CHARS >= PACKET_LENGTH*2
#define MAX_HAND_SWITCH_CHARS 35
// TODO make left/right switch

// TODO test if 2 should is enough
#define REPEAT_MASTER_HAND_READ_LIMIT 10
#define REPEAT_SLAVE_HAND_READ_LIMIT 10

// (9600 38400 57600 74880 115200 230400 250000 57600 38400 chosen because it is required for Teapot Demo output, but it's

#ifdef SLAVE_HAND
#define MAX_TIME_TO_RESET 80
#define MIN_TIME_TO_RESET 20
#define MAX_FIFO_USAGE_FOR_RESET 300
#define MIN_FIFO_USAGE_FOR_RESET 150
//#define FIFO_PACKET_SIZE FIFO_APCKET_GLOBAL_SIZE
//#define FIFO_SIZE FIFO_APCKET_GLOBAL_SIZE
#else

#ifdef MASTER_HAND
// values for ESP32 400KHz I2C
// comparing to nano MPU6050 needs to be reset more often
#define MAX_TIME_TO_RESET 15
#define MIN_TIME_TO_RESET 5
#define MAX_FIFO_USAGE_FOR_RESET 100
#define MIN_FIFO_USAGE_FOR_RESET 50
#else
// values for arduino nano using 400KHz I2C
#define MAX_TIME_TO_RESET 50
#define MIN_TIME_TO_RESET 20
#define MAX_FIFO_USAGE_FOR_RESET 300
#define MIN_FIFO_USAGE_FOR_RESET 200
#endif
//#define FIFO_PACKET_SIZE FIFO_APCKET_GLOBAL_SIZE
//#define FIFO_M_PACKET_SIZE FIFO_APCKET_GLOBAL_SIZE
//#define FIFO_SIZE FIFO_APCKET_GLOBAL_SIZE
#endif
//#define USE_BT


//#define ESP_BT_BAUD 115200
//#include "BluetoothSerial.h"
//#define MASTER_SERIAL_NAME Serial1

//#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
//#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
//#endif

//BluetoothSerial btSerial;

#ifdef  MASTER_HAND

#define MASTER_BT_SERIAL
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
#define SLAVE_SERIAL_BAUD 115200

//#define PC_SERIAL_NAME hc05Master

#else

//#define LIB_SW_SERIAL
//#define LIB_ALT_SW_SERIAL 1

// 115200 57600
#ifdef MASTER_HAND
#define MASTER_SERIAL_NAME Serial2
#define PC_SERIAL_NAME Serial
#define MASTER_SERIAL_BAUD 115200
//#define PC_SERIAL_BAUD 115200
#endif
#endif
// TODO remove so many ifs ...

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

enum Sensor
{
    SENSOR_PIN_TU = 0,
    SENSOR_PIN_SU = 1,
    SENSOR_PIN_FU = 2,
    SENSOR_PIN_MU = 3,
    SENSOR_PIN_EU = 4,
    SENSOR_PIN_HP = 5, // hadn palm
    SENSOR_PIN_NF=-1,
    
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

void enableSingleMPU(int sensorToEnable);
int initMPUAndDMP(int attempt);
int setOrRotateSelectedGyro(int i);


#ifdef OLD_RESET
void resetMPUs(int around);
#endif

void fifoToPacket(byte * fifoBuffer, byte * packet, int selectedSensor);

#ifdef MASTER_HAND
void sendDataRequest(int selectedSensor);
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
   // MPU6050_MPU9250 mpu = *mpu;
    mpu->getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384;
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384;
}*/
float Axyz[3];
float Gxyz[3];
float Mxyz[3];


void getMPU9250Data(MPU6050_MPU9250 * mpu);
bool loadDataFromFIFO(bool forceLoad);
void writePacket();
void loadDataAndSendPacket();

#ifdef SLAVE_HAND
void slaveHandDataRequestHandler();
#endif

#ifdef MASTER_HAND
void loadSlaveHandData() {
#endif

int selectSingleMPU(int selectorOffsettedPinOrig, int i);


// start of code for measuring offsets 
#ifdef MEASURE_OFFSETS

//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int16_t ax, ay, az,gx, gy, gz;
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

void meansensors(MPU6050_MPU9250 *mpuP);
void calibration(MPU6050_MPU9250 *mpuP);
void measureOffsets(MPU6050_MPU9250 *mpuP);

#endif // end of code for measuring offsets 

#endif