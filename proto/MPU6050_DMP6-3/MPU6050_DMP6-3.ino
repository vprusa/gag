// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
/*
Copyright (c) 2018 Vojtěch Průša
*/
// Based on example of MPU6050 from https://github.com/jrowberg/i2cdevlib
// from 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// 
//#define ESP32_RIGHT 1// master to left, slave to pc

#include "MPU6050_MPU9250_9Axis_MotionApps41.h"


#include "MPU6050_MPU9250_9Axis_MotionApps41.h"


#ifdef ESP32_RIGHT
#define USE_DISPLAY 1
#include "Wire.h"

#include <TimeLib.h>
#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`
//#include "OLEDDisplay.h"
#include "OLEDDisplayUi.h"

#endif


#ifdef USE_DISPLAY

const unsigned char activeSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00011000,
    B00100100,
    B01000010,
    B01000010,
    B00100100,
    B00011000
};

const unsigned char inactiveSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000
};

// Initialize the OLED display using Wire library
//SSD1306Wire  display(0x3c, D3, D5);
SSD1306Wire display(0x3c, 18, 19);

// SH1106 display(0x3c, D3, D5);

OLEDDisplayUi ui ( &display );

int screenW = 128;
int screenH = 64;
int clockCenterX = screenW/2;
int clockCenterY = ((screenH-16)/2)+16;   // top yellow part is 16 px height
int clockRadius = 23;

// utility function for digital clock display: prints leading 0
String twoDigits(int digits){
  if(digits < 10) {
    String i = '0'+String(digits);
    return i;
  }
  else {
    return String(digits);
  }
}

void clockOverlay(OLEDDisplay *display, OLEDDisplayUiState* state) {
}


void analogClockFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  //  ui.disableIndicator();
  // Draw the clock face
  //  display->drawCircle(clockCenterX + x, clockCenterY + y, clockRadius);
  display->drawCircle(clockCenterX + x, clockCenterY + y, 2);
  //
  //hour ticks
  for( int z=0; z < 360;z= z + 30 ){
  //Begin at 0° and stop at 360°
    float angle = z ;
    angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
    int x2 = ( clockCenterX + ( sin(angle) * clockRadius ) );
    int y2 = ( clockCenterY - ( cos(angle) * clockRadius ) );
    int x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 8 ) ) ) );
    int y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 8 ) ) ) );
    display->drawLine( x2 + x , y2 + y , x3 + x , y3 + y);
  }

  // display second hand
  float angle = second() * 6 ;
  angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
  int x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 5 ) ) ) );
  int y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 5 ) ) ) );
  display->drawLine( clockCenterX + x , clockCenterY + y , x3 + x , y3 + y);
  //
  // display minute hand
  angle = minute() * 6 ;
  angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
  x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 4 ) ) ) );
  y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 4 ) ) ) );
  display->drawLine( clockCenterX + x , clockCenterY + y , x3 + x , y3 + y);
  //
  // display hour hand
  angle = hour() * 30 + int( ( minute() / 12 ) * 6 )   ;
  angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
  x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 2 ) ) ) );
  y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 2 ) ) ) );
  display->drawLine( clockCenterX + x , clockCenterY + y , x3 + x , y3 + y);
}

void digitalClockFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  String timenow = String(hour())+":"+twoDigits(minute())+":"+twoDigits(second());
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_24);
  display->drawString(clockCenterX + x , clockCenterY + y, timenow );
}

// This array keeps function pointers to all frames
// frames are the single views that slide in
FrameCallback frames[] = { analogClockFrame, digitalClockFrame };

// how many frames are there?
int frameCount = 2;

// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback overlays[] = { clockOverlay };
int overlaysCount = 1;
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

//#define CONFIG_DISABLE_HAL_LOCKS

//#define ARDUHAL_LOG_LEVEL ARDUHAL_LOG_LEVEL_ERROR
//#define CORE_DEBUG_LEVEL
//#define ARDUHAL_LOG_LEVEL (1)
//#define CORE_DEBUG_LEVEL 

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
// SENSORs <0,4>
#define FIRST_SENSOR 0
// or set LAST_SENSOR to 4
#define LAST_SENSOR 5
// time for internal interrupt to trigger in loop - working up to 50 ms but freezes may occure - so reset MPU's FIFO more ften (20ms each?)
#define SWITCH_SENSORS_MS 0

/*#define SENSOR_PIN_TU TU
#define SENSOR_PIN_TU_COMPENSATION 34 
#define SENSOR_PIN_SU SU
#define SENSOR_PIN_SU_COMPENSATION 35
#define SENSOR_PIN_FU FU
#define SENSOR_PIN_FU_COMPENSATION 32
#define SENSOR_PIN_MU MU
#define SENSOR_PIN_MU_COMPENSATION 33
#define SENSOR_PIN_EU EU
#define SENSOR_PIN_EU_COMPENSATION 25
#define SENSOR_PIN_HP HP // HAND PALM
//#define SENSOR_PIN_HP_COMPENSATION 13
#define SENSOR_PIN_NF NF
*/

// https://randomnerdtutorials.com/esp32-pinout-reference-gpios/

#define SENSOR_PIN_TU TU
#define SENSOR_PIN_TU_COMPENSATION 4
#define SENSOR_PIN_SU SU
#define SENSOR_PIN_SU_COMPENSATION 15
#define SENSOR_PIN_FU FU
#define SENSOR_PIN_FU_COMPENSATION 12
#define SENSOR_PIN_MU MU
#define SENSOR_PIN_MU_COMPENSATION 13
#define SENSOR_PIN_EU EU
#define SENSOR_PIN_EU_COMPENSATION 14
#define SENSOR_PIN_HP HP // HAND PALM
#define SENSOR_PIN_HP_COMPENSATION 2
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
/*
#define SENSOR_PIN_TU TU
#define SENSOR_PIN_SU SU
#define SENSOR_PIN_FU FU
#define SENSOR_PIN_MU MU
#define SENSOR_PIN_EU EU
#define SENSOR_PIN_HP HP
#define SENSOR_PIN_HP_COMPENSATION 13
#define SENSOR_PIN_NF NF

//#define SENSOR_PIN_OFFSET 3
#define SENSOR_PIN_OFFSET 3
*/
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
#define BT_BAUD 115200

#endif

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
#define MAX_TIME_TO_RESET 80
#define MIN_TIME_TO_RESET 20
#define MAX_FIFO_USAGE_FOR_RESET 300
#define MIN_FIFO_USAGE_FOR_RESET 150
//#define FIFO_PACKET_SIZE FIFO_APCKET_GLOBAL_SIZE
//#define FIFO_SIZE FIFO_APCKET_GLOBAL_SIZE
//#define USE_BT
#define USE_USB
#define USE_BT_MASTER
#else
#define RIGHT_HAND
#define RIGHT_HAND_SLAVE
#define RIGHT_HAND_TO_PC

#ifdef ESP32_RIGHT
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

#ifdef  ESP32_RIGHT

//#define BT_BAUD 57600
#define MASTER_SERIAL_NAME Serial
#define USB_BAUD 115200
#define USE_USB

//#define PC_SERIAL_NAME hc05Master

#else
#ifdef RIGHT_HAND
//#define USE_BT
#define USE_USB
#endif

//#define USE_BT

//#define LIB_SW_SERIAL
//#define LIB_ALT_SW_SERIAL 1

// 115200 57600

#ifdef LEFT_HAND
#define USB_BAUD 115200
#define BT_BAUD 115200
#define MASTER_SERIAL_NAME Serial
#define PC_SERIAL_NAME Serial
#endif
#endif
// TODO remove so many ifs ...

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
uint8_t packetSizeS = MPU6050_FIFO_PACKET_SIZE; // expected DMP packet size (default is 42 bytes)
uint8_t packetSizeM = MPU9250_FIFO_PACKET_SIZE; // expected DMP packet size (default is 42 bytes)
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

enum SENSOR
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

SENSOR selectedSENSOR = SENSOR_PIN_NF;

//Variables
float elapsedTime, timeNow, timePrev, elapsedTimeToSwitch, handSwitchPrev, handSwitchElapsed; //Variables for time control
Gyro gyros[SENSORS_COUNT];

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void enableSingleMPU(int SENSORToEnable) {

    //Wire.flush();
    for (int i = 0; i < SENSORS_COUNT; i++) {
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
        
        if ( i != SENSORToEnable ) {
            digitalWrite(selectorOffsettedPin, HIGH);
        }
     

    }

  for (int i = 0; i < SENSORS_COUNT; i++) {
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
        
        if ( i == SENSORToEnable ) {
            digitalWrite(selectorOffsettedPin, LOW);     
        }
    }
}


void setup() {
//    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
#ifdef ESP32_RIGHT
    MASTER_SERIAL_NAME.begin(USB_BAUD);
    #ifdef RIGHT_HAND
    while (!MASTER_SERIAL_NAME)
        ; // wait for Leonardo enumeration, others continue immediately
    #endif
    MASTER_SERIAL_NAME.println(F("USB up"));


#ifdef USE_DISPLAY
  /*
  display.init();
  int lineHeight = 10;
  // TODO for .. split strings by 26 chars per line
  display.drawString(0, 0, "Lorem ipsum dolor sit amet,");
  display.drawString(0, lineHeight, "consectetur adipiscing elit.");
  display.drawString(0, lineHeight*2, "Etiam interd mauris libero");
  display.drawString(0, lineHeight*3, "vel tempus turpis aliquet.");
  display.drawString(0, lineHeight*4, "Nunc pretium gravida est,");
  display.drawString(0, lineHeight*5, "cras consequat elementum. ");
  display.drawString(0, lineHeight*6, "Vulputate ultrices erat justo.");
  //display.drawString(0, lineHeight*7, "Morbi facilisis consequat.");
  display.display();
  */

	// The ESP is capable of rendering 60fps in 80Mhz mode
	// but that won't give you much time for anything else
	// run it in 160Mhz mode or just set it to 30 fps
  ui.setTargetFPS(30);

	// Customize the active and inactive symbol
  ui.setActiveSymbol(activeSymbol);
  ui.setInactiveSymbol(inactiveSymbol);

  // You can change this to
  // TOP, LEFT, BOTTOM, RIGHT
  ui.setIndicatorPosition(TOP);

  // Defines where the first frame is located in the bar.
  ui.setIndicatorDirection(LEFT_RIGHT);

  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);

  // Add frames
  ui.setFrames(frames, frameCount);

  // Add overlays
  ui.setOverlays(overlays, overlaysCount);
  //#ifdef USE_DISPLAY

  // Initialising the UI will init the display too.
  ui.init();

  display.flipScreenVertically();
  //display.flipScreenHorizontally();

  unsigned long secsSinceStart = millis();
  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
  const unsigned long seventyYears = 2208988800UL;
  // subtract seventy years:
  unsigned long epoch = secsSinceStart - seventyYears * SECS_PER_HOUR;
  setTime(epoch);

//#endif

#endif

#endif
    for (int i = FIRST_SENSOR; i <= LAST_SENSOR; i++) {
        int SENSORToEnable = SENSOR_PIN_OFFSET + i;
         
        #ifdef SENSOR_PIN_TU_COMPENSATION
        if (i == SENSOR_PIN_TU) {
            SENSORToEnable = SENSOR_PIN_TU_COMPENSATION;
        }
        #endif
        #ifdef SENSOR_PIN_SU_COMPENSATION
        if (i == SENSOR_PIN_SU) {
            SENSORToEnable = SENSOR_PIN_SU_COMPENSATION;
        }
        #endif
        #ifdef SENSOR_PIN_FU_COMPENSATION
        if (i == SENSOR_PIN_FU) {
            SENSORToEnable = SENSOR_PIN_FU_COMPENSATION;
        }
        #endif
        #ifdef SENSOR_PIN_MU_COMPENSATION
        if (i == SENSOR_PIN_MU) {
            SENSORToEnable = SENSOR_PIN_MU_COMPENSATION;
        }
        #endif
        #ifdef SENSOR_PIN_EU_COMPENSATION
        if (i == SENSOR_PIN_EU) {
            SENSORToEnable = SENSOR_PIN_EU_COMPENSATION;
        }
        #endif
        #ifdef SENSOR_PIN_HP_COMPENSATION
        if (i == SENSOR_PIN_HP) {
            SENSORToEnable = SENSOR_PIN_HP_COMPENSATION;
        }
        #endif
        pinMode(SENSORToEnable, OUTPUT);
    }
#ifdef ESP32_RIGHT
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
#ifdef USE_BT_MASTER
    //TODO fix, rename
    PC_SERIAL_NAME.begin(BT_BAUD);
    // while (!hc05.available()){}
    // hc05Master.println(F("BT up"));
#endif

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    for (int i = FIRST_SENSOR; i <= LAST_SENSOR; i++) {
        selectedSENSOR = (SENSOR)i;
        enableSingleMPU(selectedSENSOR);
        if(i == HP) {
            //gyros[selectedSENSOR].mpuM = new MPU6050(MPU6050_ADDRESS_AD0_HIGH); // MPU6050_ADDRESS_AD0_LOW / MPU6050_ADDRESS_AD0_HIGH
            //gyros[selectedSENSOR].mpuM = new MPU9250(MPU6050_ADDRESS_AD0_LOW); // MPU6050_ADDRESS_AD0_LOW / MPU6050_ADDRESS_AD0_HIGH
            //gyros[selectedSENSOR].mpuM->isMPU6050 = true;
            gyros[selectedSENSOR].mpu = new MPU6050_MPU9250(MPU6050_MPU9250_ADDRESS_AD0_LOW);//   0x68 / 0x69
            gyros[selectedSENSOR].mpu->isMPU9250 = true;
        } else {
            //gyros[selectedSENSOR].mpu = new MPU6050(MPU6050_ADDRESS_AD0_HIGH);//   0x68 / 0x69
            //gyros[selectedSENSOR].mpu = new MPU6050(MPU6050_ADDRESS_AD0_LOW);//   0x68 / 0x69
            //gyros[selectedSENSOR].mpu->isMPU6050 = true;
            gyros[selectedSENSOR].mpu = new MPU6050_MPU9250(MPU6050_MPU9250_ADDRESS_AD0_LOW);//   0x68 / 0x69
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

//#define MEASURE_OFFSETS
#ifdef MEASURE_OFFSETS
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int16_t ax, ay, az,gx, gy, gz;
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

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
#endif

int initMPUAndDMP(int attempt) {
    if (attempt <= 0) {
        return 0;
    }
// initialize device
#ifdef USE_BT
    hc05.println(F("BT: Initializing I2C devices..."));
#endif
#ifdef USE_USB
    MASTER_SERIAL_NAME.println(F("USB: Initializing I2C devices..."));
#endif
    if(selectedSENSOR == HP) {
        //MPU9250 mpu = *gyros[selectedSENSOR].mpuM;
        MPU6050_MPU9250 mpu = *gyros[selectedSENSOR].mpu;
        //mpu.initialize();
        mpu.initialize();

    #ifdef USE_USB
        MASTER_SERIAL_NAME.println(F("Testing device connections..."));
        MASTER_SERIAL_NAME.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

        MASTER_SERIAL_NAME.println("testConnection");
        MASTER_SERIAL_NAME.println(mpu.getDeviceID());
    
        // wait for ready
        MASTER_SERIAL_NAME.println(F("Send any character to begin DMP programming and demo: "));
        // load and configure the DMP
        MASTER_SERIAL_NAME.println(F("Initializing DMP..."));
    #endif
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
  
    #ifdef USE_USB
            MASTER_SERIAL_NAME.print(F("Enabling DMP... "));
            MASTER_SERIAL_NAME.println(selectedSENSOR);
            mpu.setDMPEnabled(true);

            // set our DMP Ready flag so the main loop( ) function knows it's okay to use it
            MASTER_SERIAL_NAME.println(F("DMP ready! Getting packet size..."));
            //gyros[selectedSENSOR].dmpReady = true;
            MASTER_SERIAL_NAME.print(F("packet size: "));
            MASTER_SERIAL_NAME.print(packetSizeM);
            MASTER_SERIAL_NAME.println(F(""));
    #endif
        } else {
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
    } else {
        //MPU6050 mpu = *gyros[selectedSENSOR].mpu;
        MPU6050_MPU9250 mpu = *gyros[selectedSENSOR].mpu;
        mpu.initialize();

    #ifdef USE_USB
        MASTER_SERIAL_NAME.println(F("Testing device connections..."));
        MASTER_SERIAL_NAME.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
        MASTER_SERIAL_NAME.println("testConnection");
        MASTER_SERIAL_NAME.println(mpu.getDeviceID());
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
        #ifdef MEASURE_OFFSETS
        measureOffsets(&mpu);
        #endif
        // make sure it worked (returns 0 if so)
        //if (devStatus == 0) {
        if (true) {
    // turn on the DMP, now that it's ready
    // initialize device
    #ifdef USE_BT
            hc05.print(F("Enabling DMP... "));
            hc05.println(selectedSENSOR);
            mpu.setDMPEnabled(true);

            // set our DMP Ready flag so the main loop( ) function knows it's okay to use it
            hc05.println(F("DMP ready! Getting packet size..."));
            //gyros[selectedSENSOR].dmpReady = true;
            // get expected DMP packet size for later comparison
            hc05.print(F("packet size: "));
            hc05.print(packetSizeS);
            hc05.println(F(""));
    #endif
    #ifdef USE_USB
            MASTER_SERIAL_NAME.print(F("Enabling DMP... "));
            MASTER_SERIAL_NAME.println(selectedSENSOR);
            mpu.setDMPEnabled(true);

            // set our DMP Ready flag so the main loop( ) function knows it's okay to use it
            MASTER_SERIAL_NAME.println(F("DMP ready! Getting packet size..."));
            //gyros[selectedSENSOR].dmpReady = true;
            MASTER_SERIAL_NAME.print(F("packet size: "));
            MASTER_SERIAL_NAME.print(packetSizeS);
            MASTER_SERIAL_NAME.println(F(""));
    #endif
        } else {
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
    }
    return 0;
}

int setOrRotateSelectedGyro(int i) {
    if (i == -1) {
        i = selectedSENSOR + 1;
    }
    if (i > LAST_SENSOR) {
        i = FIRST_SENSOR;
    }
    selectedSENSOR = (SENSOR)i;
    enableSingleMPU(selectedSENSOR);
    return i;
}

#ifdef OLD_RESET
void resetMPUs(int around) {
    // reset n after (TODO without current?)
    if (around > 0) {
        for (int i = 0; i < around; i++) {
            //MPU6050 mpu = *gyros[selectedSENSOR].mpu;
            MPU6050_MPU9250 mpu = *gyros[selectedSENSOR].mpu;

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
            //MPU6050 mpu = *gyros[selectedSENSOR].mpu;
            MPU6050_MPU9250 mpu = *gyros[selectedSENSOR].mpu;
            mpu.resetFIFO();
            setOrRotateSelectedGyro(-1);
        }
    }
}
#endif

void automaticFifoReset() {
    long now = millis();
    int currentlySellectedSensor = selectedSENSOR;

    for(int i = 0; i < SENSORS_COUNT; i++){
        if(gyros[i].lastResetTime + MIN_TIME_TO_RESET < now 
        #ifdef RIGHT_HAND
        && gyros[i].hasDataReady
        #endif
        ) {
            int selectedNow = setOrRotateSelectedGyro(i);
            if(selectedNow != 5){
                //MPU6050 mpu = *gyros[selectedNow].mpu;
                MPU6050_MPU9250 mpu = *gyros[selectedSENSOR].mpu;
               
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
                MPU6050_MPU9250 mpu = *gyros[selectedSENSOR].mpu;

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

void fifoToPacket(byte * fifoBuffer, byte * packet, int selectedSENSOR) {
    packet[2] = selectedSENSOR;
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
void sendDataRequest(int selectedSENSOR) {
    PC_SERIAL_NAME.write('$');
    PC_SERIAL_NAME.write(selectedSENSOR);
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
//void getMPU9250Data(MPU9250 * mpuI) {
void getMPU9250Data(MPU6050_MPU9250 * mpu) {

   // uint8_t buffer_m[6];

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t   mx, my, mz;

    
    //float q[4];
    uint16_t qI[4];

    // MPU6050_MPU9250 mpu = *gyros[selectedSENSOR].mpu;
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
    uint8_t *fifoBuffer = gyros[selectedSENSOR].fifoBuffer; // FIFO storage buffer*/

    fifoBuffer[1] = qI[0] & 0xFF;
    fifoBuffer[0] = qI[0]>> 8;
    
    fifoBuffer[5] = qI[1] & 0xFF;
    fifoBuffer[4] = qI[1]>> 8;
    
    fifoBuffer[9] = qI[2] & 0xFF;
    fifoBuffer[8] = qI[2] >> 8;
    

    fifoBuffer[13] = qI[3] & 0xFF;
    fifoBuffer[12] = qI[3] >> 8;
}

bool loadDataFromFIFO(int forceLoad = false) {
    if(selectedSENSOR != LAST_SENSOR){
        //MPU6050 mpu = *gyros[selectedSENSOR].mpu;
        MPU6050_MPU9250 mpu = *gyros[selectedSENSOR].mpu;

        if(!gyros[selectedSENSOR].hasDataReady || forceLoad){
            fifoCount = mpu.getFIFOCount();
           // Serial.print("fifoCount:2 ");
            ///Serial.println(fifoCount);
            uint8_t *fifoBuffer = gyros[selectedSENSOR].fifoBuffer; // FIFO storage buffer
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
                gyros[selectedSENSOR].hasDataReady=true;
                return true;
            }
        }
    } else {
        //MPU9250 mpu = *gyros[selectedSENSOR].mpuM;
        //MPU6050_MPU9250 mpu = *gyros[selectedSENSOR].mpu;

        forceLoad = true;
        if(!gyros[selectedSENSOR].hasDataReady || forceLoad) {
            //getMPU9250Data(gyros[selectedSENSOR].mpuM);
            getMPU9250Data(gyros[selectedSENSOR].mpu);
            //gyros[selectedSENSOR].mpu->dmpGet6AxisQuaternion()
            gyros[selectedSENSOR].hasDataReady=true;
            gyros[selectedSENSOR].alreadySentData=false;
            return true;
        }
    }
    return false;
}

void writePacket() {
    uint8_t *fifoBuffer = gyros[selectedSENSOR].fifoBuffer; // FIFO storage buffer

#ifdef ESP32_RIGHT
    MASTER_SERIAL_NAME.write(teapotPacket, PACKET_LENGTH);

    if(!gyros[selectedSENSOR].alreadySentData && gyros[selectedSENSOR].hasDataReady) {
        fifoToPacket(fifoBuffer, teapotPacket, selectedSENSOR);
        MASTER_SERIAL_NAME.write(teapotPacket, PACKET_LENGTH);
        MASTER_SERIAL_NAME.write((byte)0x00);
        gyros[selectedSENSOR].hasDataReady = false;
        gyros[selectedSENSOR].alreadySentData = true;
        teapotPacket[PACKET_COUNTER_POSITION]++; // packetCount, loops at 0xFF on purpose
    }
#else
#ifdef RIGHT_HAND
    if(!gyros[selectedSENSOR].alreadySentData && gyros[selectedSENSOR].hasDataReady){
        fifoToPacket(fifoBuffer, teapotPacket, selectedSENSOR);
        MASTER_SERIAL_NAME.write(teapotPacket, PACKET_LENGTH);
        MASTER_SERIAL_NAME.write(0x00);
        teapotPacket[PACKET_COUNTER_POSITION]++; // packetCount, loops at 0xFF on purpose
        gyros[selectedSENSOR].hasDataReady = false;
        gyros[selectedSENSOR].alreadySentData = true;
    }
    readAlign = 0;
    readAligned = 0;
#endif
#ifdef LEFT_HAND
    if(!gyros[selectedSENSOR].alreadySentData && gyros[selectedSENSOR].hasDataReady) {
        fifoToPacket(fifoBuffer, teapotPacket, selectedSENSOR);
        MASTER_SERIAL_NAME.write(teapotPacket, PACKET_LENGTH);
        MASTER_SERIAL_NAME.write((byte)0x00);
        gyros[selectedSENSOR].hasDataReady = false;
        gyros[selectedSENSOR].alreadySentData = true;
        teapotPacket[PACKET_COUNTER_POSITION]++; // packetCount, loops at 0xFF on purpose
    }
#endif        
#endif

}

void loadDataAndSendPacket() {
    loadDataFromFIFO(false);
    if(gyros[selectedSENSOR].hasDataReady) {

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
void slaveHandDataRequestHandler() {
    int limit = REPEAT_LEFT_HAND_READ_LIMIT;
    readAlign = 0;
    readAligned = 0;
    while(limit > 0) {
        int ch = MASTER_SERIAL_NAME.read();
        if (ch != -1) {
         
            if(readAlign<1) {
                if(ch == '$') {
                    readAlign=1;
                    //MASTER_SERIAL_NAME.println("$");
                }
            } else {
                if(ch >= 0 && ch < SENSORS_COUNT) {
                    //MASTER_SERIAL_NAME.println("al");
                    readAligned = 1;
                    readAlign=0;//++;
                    setOrRotateSelectedGyro(ch);

                    if(readAligned == 1){
                        gyros[selectedSENSOR].alreadySentData = false;
                        writePacket();
                        loadDataAndSendPacket();
                        int currentlySellectedSensor = selectedSENSOR;
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
void loadSlaveHandData() {
    int limit = REPEAT_RIGHT_HAND_READ_LIMIT;
    
    while(--limit>0){ 
        sendDataRequest(selectedSENSOR);
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
 int remainingTimeBudget =  0;

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

// reset interrupt flag and get INT_STATUS byte
#ifdef USE_BT_MASTER
    //loadSlaveHandData();
#endif

// TODO switch hands
#ifdef RIGHT_HAND_SLAVE 
#endif

#ifdef ESP32_RIGHT
    gyros[selectedSENSOR].alreadySentData = false;
    //writePacket();
    loadDataAndSendPacket();
    int currentlySellectedSensor = selectedSENSOR;
    setOrRotateSelectedGyro(-1);
    //setOrRotateSelectedGyro(2);
    loadDataFromFIFO(true);
#endif

#ifdef LEFT_HAND
        gyros[selectedSENSOR].alreadySentData = false;
        //writePacket();
        loadDataAndSendPacket();
        int currentlySellectedSensor = selectedSENSOR;
        setOrRotateSelectedGyro(-1);
        loadDataFromFIFO(true);
        //resetMPUs(-3);
        //automaticFifoReset();
#endif
    automaticFifoReset();
#ifdef USE_BT_MASTER
    //loadSlaveHandData();
#endif

}
