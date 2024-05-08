/**
 * Copytright Vojtěch Průša | prusa.vojtech@gmail.com 
 * 
 * TODO: 
 * - modularize and librarize 
 * - add macros for switching modules on/off
 * - would not it be nice to have a script that extracts code from examples and puts them together ...
 * 
 * Install dev note:
 * - pip install esptool
*/

#include <TFT_eSPI.h>
//#include <User_Setups/Setup25_TTGO_T_Display.h>

#include <SPI.h>
#include "WiFi.h"
// #include <Wire.h>
#include <Button2.h>
#include "esp_adc_cal.h"
#include "bmp.h"

#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
#define TFT_SLPIN 0x10
#endif

#define TFT_MOSI 19
#define TFT_SCLK 18
#define TFT_CS 5
#define TFT_DC 16
#define TFT_RST 23

#define TFT_BL 4 // Display backlight control pin
#define ADC_EN 14
#define ADC_PIN 34
#define BUTTON_1 35
#define BUTTON_2 0

#define MENU_SCAN_WIFI 1
#define MENU_VOLTAGE 2
#define MENU_CUBE 3
#define MENU_CLOCK_SKETCH 4
#define MENU_CLOCK 5
#define MENU_LIFE 6
#define MENU_FRACTALS 7
#define MENU_GY25 8

#define MENU_ITEM_MAX 8

#define BLACK 0x0000
#define WHITE 0xFFFF

// #include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
// #include <SPI.h>

//TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

#define TFT_GREY 0xBDF7
TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library


// #define USE_DEMO_GY25
#ifdef USE_DEMO_GY25
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
// #include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

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
// #define OUTPUT_READABLE_YAWPITCHROLL

#define OUTPUT_SHOW_ON_SCREEN

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

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// #define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
// #define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
// bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 gy;      // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

#define MPU25_SDA 21
#define MPU25_SCL 22

uint16_t SDAs[] = {21, 17}; // TODO Where did Bimbo the Elephant lose the list of working SDA pins?

void setup_gy25()
{
  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  // Serial.begin(115200);
  // while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Serial.println(F("I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE"));
  // Wire.begin();
  Wire.endTransmission();
  pinMode(MPU25_SDA, INPUT); // needed because Wire.end() enables pullups, power Saving
  pinMode(MPU25_SCL, INPUT);
  Wire.begin(MPU25_SDA, MPU25_SCL, 400000);
  // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Serial.println(F("I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE"));
  // Fastwire::setup(400, true);
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  // Serial.begin(115200);
  // while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  // pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  // while (Serial.available() && Serial.read()); // empty buffer
  // while (!Serial.available());                 // wait for data
  // while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    // Serial.println(F(")..."));
    // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    // mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  // pinMode(LED_PIN, OUTPUT);
}

void showGY25Data()
{
  static uint64_t timeStamp = 0;
  if (millis() - timeStamp > 200)
  {
    timeStamp = millis();
    String GY25Data = "ypr:\t" + String(ypr[0] * 180 / M_PI) + "\t" + String(ypr[1] * 180 / M_PI) + "\t" + String(ypr[2] * 180 / M_PI);
    Serial.println(GY25Data);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(GY25Data, tft.width() / 2, 5);
  }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop_gy25()
{
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_SHOW_ON_SCREEN
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#endif
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    /*
      mpu.dmpGetAccel(&aa, fifoBuffer);
      Serial.print("\tRaw Accl XYZ\t");
      Serial.print(aa.x);
      Serial.print("\t");
      Serial.print(aa.y);
      Serial.print("\t");
      Serial.print(aa.z);
      mpu.dmpGetGyro(&gy, fifoBuffer);
      Serial.print("\tRaw Gyro XYZ\t");
      Serial.print(gy.x);
      Serial.print("\t");
      Serial.print(gy.y);
      Serial.print("\t");
      Serial.print(gy.z);
    */
    Serial.println();

#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

    // blink LED to indicate activity
    // blinkState = !blinkState;
    // digitalWrite(LED_PIN, blinkState);
  }
}

#endif

// #define USE_DEMO_GY25
//#define USE_DEMO_RADIO
#ifdef USE_DEMO_RADIO
// https: //dratek.cz/arduino/1492-fm-rds-radio-si4703-modul-tuneru-pro-avr-arm-pic.html
// https://navody.dratek.cz/navody-k-produktum/arduino-fm-radio-si4703.html
// Arduino FM Rádio Si4703

// připojení potřebných knihoven
#include <Si4703_Breakout.h>
#include <Wire.h>
// nastavení propojovacích pinů
#define resetPin 2
#define SDIO A4
#define SCLK A5
// inicializace modulu z knihovny
Si4703_Breakout radio(resetPin, SDIO, SCLK);
// proměnné pro běh programu
int frekvence;
int hlasitost;
char rdsBuffer[10];

void setup_radio();

void loop_radio();
void zobrazInfo();
#endif

/*
 An example analogue clock using a TFT LCD screen to show the time
 use of some of the drawing commands with the ST7735 library.

 For a more accurate clock, it would be better to use the RTClib library.
 But this is just a demo. 

 Uses compile time to set the time so a reset will start with the compile time again
 
 Gilchrist 6/2/2014 1.0
 Updated by Bodmer
 */

// Maximum number of generations until the screen is refreshed
#define MAX_GEN_COUNT 1000

// The ESP8266 has plenty of memory so we can create a large array
// 2 x 2 pixel cells, array size = 5120 bytes per array, runs fast
#define GRIDX 128
// 64
#define GRIDY 240
// 120
#define CELLXY 1
//2

// 1 x 1 pixel cells, array size = 20480 bytes per array
//#define GRIDX 160
//#define GRIDY 128
//#define CELLXY 1

#define GEN_DELAY 20 // Set a delay between each generation to slow things down

//Current grid and newgrid arrays are needed
uint8_t grid[GRIDX][GRIDY];

//The new grid for the next generation
uint8_t newgrid[GRIDX][GRIDY];

//Number of generations
uint16_t genCount = 0;

#define MAX_GEN_COUNT 1000

// The ESP8266 has plenty of memory so we can create a large array
// 2 x 2 pixel cells, array size = 5120 bytes per array, runs fast
#define FGRIDX 64
#define FGRIDY 120
#define FCELLXY 2

// 1 x 1 pixel cells, array size = 20480 bytes per array
//#define GRIDX 160
//#define GRIDY 128
//#define CELLXY 1

#define FGEN_DELAY 20 // Set a delay between each generation to slow things down

//Current grid and newgrid arrays are needed
uint8_t fgrid[FGRIDX][FGRIDY];

//The new grid for the next generation
uint8_t fnewgrid[FGRIDX][FGRIDY];

//Number of generations
uint16_t fgenCount = 0;

float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0; // Saved H, M, S x & y multipliers
float sdeg = 0, mdeg = 0, hdeg = 0;
uint16_t osx = 64, osy = 64, omx = 64, omy = 64, ohx = 64, ohy = 64; // Saved H, M, S x & y coords
uint16_t x0 = 0, x1 = 0, yy0 = 0, yy1 = 0;
uint32_t targetTime = 0; // for next 1 second timeout

static uint8_t conv2d(const char *p)
{
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

uint8_t hh = conv2d(__TIME__), mm = conv2d(__TIME__ + 3), ss = conv2d(__TIME__ + 6); // Get H, M, S from compile time

boolean initial = 1;

Button2 btnR(BUTTON_1);
Button2 btnL(BUTTON_2);

//TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

int16_t h;
int16_t w;

int inc = -2;

float xx, xy, xz;
float yx, yy, yz;
float zx, zy, zz;

float fact;

int Xan, Yan, Zan;

int Xoff;
int Yoff;
int Zoff;

struct Point3d
{
  int x;
  int y;
  int z;
};

struct Point2d
{
  int x;
  int y;
};

int LinestoRender;    // lines to render.
int OldLinestoRender; // lines to render just in case it changes. this makes sure the old lines all get erased.

struct Line3d
{
  Point3d p0;
  Point3d p1;
};

struct Line2d
{
  Point2d p0;
  Point2d p1;
};

Line3d Lines[20];
Line2d Render[20];
Line2d ORender[20];

char buff[512];
int vref = 1100;
int btnRClick = false;
int btnLClick = false;

boolean inMenu = false;
int menuItem = 8;
//int menuItem = 7;
int menuItemDefVal = 7;
bool menuItemChanged = false;
bool cubePlaying = false;
bool clockPlaying = false;
bool lifePlaying = false;
bool fractalsPlaying = false;

#ifdef USE_DEMO_GY25
bool gy25Playing = false;
#endif
#ifdef USE_DEMO_RADIO
bool radioPlaying = false;
#endif

void wifi_scan();
void cube();
void play_clock();
void setup_life();
void setup_fractals();
void SetVars();
void ProcessLine(struct Line2d *ret, struct Line3d vec);
void RenderImage();

//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
void espDelay(int ms)
{
  /*esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
    esp_light_sleep_start();*/
  delay(ms);
}

#ifdef USE_DEMO_RADIO
void setup_radio()
{
  menuItem = menuItemDefVal;
  menuItem++;

  // zahájení komunikace po sériové lince
  // rychlostí 9600 baud
  Serial.begin(9600);
  // vytištění nápovědy
  Serial.println("\n\nSi4703 Radio");
  Serial.println("===========================");
  Serial.println("a b     Oblibene stanice nastavene v programu");
  Serial.println("+ -     hlasitost (max 15)");
  Serial.println("n d     Zmena frekvence nahoru/dolu");
  Serial.println("r       Nacteni RDS dat (15 sekund timeout)");
  Serial.println();
  // zahájení komunikace s modulem
  radio.powerOn();
  // nastavení hlasitosti na nulu
  radio.setVolume(0);
}

void loop_radio()
{
  // kontrola sériové linky na příchozí data
  if (Serial.available())
  {
    // načteme přijatý znak do proměnné
    char ch = Serial.read();
    // dále pomocí if podmínek hledáme známé znaky pro ovládání činnosti,
    // při každé činnosti je zároveň zavolán podprogram zobrazInfo,
    // který vytiskne aktuální frekvenci a hlasitost po sériové lince

    // při znaku n spustíme hledání směrem nahoru
    if (ch == 'n')
    {
      frekvence = radio.seekUp();
      zobrazInfo();
    }
    // při znaku D spustíme hledání směrem dolů
    else if (ch == 'd')
    {
      frekvence = radio.seekDown();
      zobrazInfo();
    }
    // při znaku + přidáme hlasitost
    else if (ch == '+')
    {
      hlasitost++;
      // maximální hodnota hlasitosti je 15,
      // při větší nastavíme pouze 15
      if (hlasitost == 16)
        hlasitost = 15;
      radio.setVolume(hlasitost);
      zobrazInfo();
    }
    // při znaku - snížíme hlasitost
    else if (ch == '-')
    {
      hlasitost--;
      // minimální hodnota hlasitosti je 0,
      // při záporné hodnotě nastavíme 0
      if (hlasitost < 0)
        hlasitost = 0;
      radio.setVolume(hlasitost);
      zobrazInfo();
    }
    // při znaku a nastavíme oblíbenou stanici,
    // pro kterou je nastavena frekvence níže
    else if (ch == 'a')
    {
      frekvence = 910; // Radio Beat
      radio.setChannel(frekvence);
      zobrazInfo();
    }
    // při znaku b nastavíme oblíbenou stanici,
    // pro kterou je nastavena frekvence níže
    else if (ch == 'b')
    {
      frekvence = 1055; // Radio Evropa 2
      radio.setChannel(frekvence);
      zobrazInfo();
    }
    // při znaku r spustíme načítání RDS dat,
    // které bude trvat 15 sekund - lze změnit níže
    else if (ch == 'r')
    {
      // pomocí funkce readRDS se pokusíme
      // načíst RDS data do proměnné
      // a následně je vytiskneme
      Serial.println("Nacteni RDS dat...");
      radio.readRDS(rdsBuffer, 15000);
      Serial.print("RDS data:");
      Serial.println(rdsBuffer);
    }
  }
}

void zobrazInfo()
{
  // vytištění informací z proměnných
  Serial.print("Frekvence:");
  Serial.print(frekvence);
  Serial.print(" | Hlasitost:");
  Serial.println(hlasitost);
}
#endif



void showVoltage()
{
  static uint64_t timeStamp = 0;
  if (millis() - timeStamp > 1000)
  {
    timeStamp = millis();
    uint16_t v = analogRead(ADC_PIN);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    String voltage = "Voltage :" + String(battery_voltage) + "V";
    Serial.println(voltage);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(voltage, tft.width() / 2, tft.height() / 2);
  }
}

void showMenu()
{
  inMenu = true;
  static uint64_t timeStamp = 0;
  if (millis() - timeStamp > 1000 || menuItemChanged)
  {
    menuItemChanged = false;
    timeStamp = millis();
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);
    tft.setCursor(0, 0);
/*
    tft.drawString("Menu", 10, 0);
    //TODO fix selector prefix from space to padding
    tft.drawString(" Scan", 15, 15);
    tft.drawString(" Voltage", 15, 26);
    tft.drawString(" Cube", 15, 37);
    tft.drawString(" Clock (sketch)", 15, 48);
    tft.drawString(" Clock", 15, 59);
    tft.drawString(" Life", 15, 70);
    tft.drawString(" Fractals", 15, 81);
#ifdef USE_DEMO_GY25
    tft.drawString(" GY25", 15, 81);
#endif
    tft.setCursor(0, 0);
    tft.drawString(">", 0, (menuItem * 11) + 5);
    */

    tft.drawString("Menu", 20, 15);
    //TODO fix selector prefix from space to padding
    tft.drawString(" Scan", 25, 25);
    tft.drawString(" Voltage", 25, 36);
    tft.drawString(" Cube", 25, 47);
    tft.drawString(" Clock (sk)", 25, 58);
    tft.drawString(" Clock", 25, 69);
    tft.drawString(" Life", 25, 80);
    tft.drawString(" Fractals", 25, 91);
#ifdef USE_DEMO_GY25
    tft.drawString(" GY25", 25, 101);
#endif
    tft.setCursor(0, 0);
    tft.drawString(">", 0, (menuItem * 11) + 5); // TODO fix cursor
  }
}

void button_init()
{
  btnR.setLongClickHandler([](Button2 &b) {
        //btnRClick = false;

        /*
        if(inMenu){
            int r = digitalRead(TFT_BL);
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("Press again to wake up",  tft.width() / 2, tft.height() / 2 );
            //espDelay(6000);
            espDelay(1000);
            digitalWrite(TFT_BL, !r);
    
            tft.writecommand(TFT_DISPOFF);
            tft.writecommand(TFT_SLPIN);
            esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
            esp_deep_sleep_start();
        }
        */
        });
  btnR.setPressedHandler([](Button2 &b)
                         {
                           btnRClick = true;
                           btnLClick = false;
                           cubePlaying = false;
                           clockPlaying = false;
                           lifePlaying = false;
                           fractalsPlaying = false;
#ifdef USE_DEMO_GY25
                           gy25Playing = false;
#endif

                           if (inMenu)
                           {
                             inMenu = false;
                             switch (menuItem - 1)
                             {
                             case MENU_SCAN_WIFI:
                               Serial.println("wifi scan");
                               wifi_scan();
                               break;
                             case MENU_VOLTAGE:
                               Serial.println("Detect Voltage..");
                               showVoltage();
                               break;
                             case MENU_CUBE:
                               Serial.println("Cube..");
                               btnRClick = false;
                               btnLClick = false;
                               cubePlaying = true;
                               // cube(); // reset cube geometry
                               setup_cube();
                               break;
                             case MENU_CLOCK_SKETCH:
                               Serial.println("Clock(sketch)..");
                               btnRClick = false;
                               btnLClick = false;
                               clockPlaying = true;
                               play_clock(); // reset cube geometry
                               break;
                             case MENU_CLOCK:
                               Serial.println("Clock..");
                               btnRClick = false;
                               btnLClick = false;
                               clockPlaying = true;
                               play_clock(); // reset cube geometry
                               break;
                             case MENU_LIFE:
                               Serial.println("Life..");
                               btnRClick = false;
                               btnLClick = false;
                               lifePlaying = true;
                               setup_life();
                               break;
                             case MENU_FRACTALS:
                               Serial.println("Fractals..");
                               btnRClick = false;
                               btnLClick = false;
                               fractalsPlaying = true;
                               setup_fractals();
                               break;
                             case MENU_GY25:
                               Serial.println("GY25..");
                               btnRClick = false;
                               btnLClick = false;
#ifdef USE_DEMO_GY25
                               gy25Playing = true;
#endif
                               setup_cube();
#ifdef USE_DEMO_GY25
                               setup_gy25();
#endif
                               break;
                             default:
                               inMenu = true;
                               break;
                             }
                           }
                         });

  btnL.setPressedHandler([](Button2 &b)
                         {
                           btnRClick = false;
                           btnLClick = true;

                           if (inMenu)
                           {
                             menuItemChanged = true;
                             menuItem++;
                             if (menuItem > MENU_ITEM_MAX)
                             {
                               menuItem = 1;
                             }
                           }
                           else
                           {
                             inMenu = true;
                           }
                         });
}

void button_loop()
{
  btnR.loop();
  btnL.loop();
}

void loop()
{
  espDelay(1);
}

void wifi_scan()
{
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);

  tft.drawString("Scan Network", tft.width() / 2, tft.height() / 2);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  espDelay(100);

  int16_t n = WiFi.scanNetworks();
  tft.fillScreen(TFT_BLACK);
  if (n == 0)
  {
    tft.drawString("no networks found", tft.width() / 2, tft.height() / 2);
  }
  else
  {
    // TODO add scroll down loop on right button press event
    tft.setTextDatum(TL_DATUM);
    tft.setCursor(0, 0);
    Serial.printf("Found %d net\n", n);
    for (int i = 0; i < n; ++i)
    {
      sprintf(buff,
              "[%d]:%s(%d)",
              i + 1,
              WiFi.SSID(i).c_str(),
              WiFi.RSSI(i));
      tft.println(buff);
    }
  }
  WiFi.mode(WIFI_OFF);
}

void loop_cube()
{

  // Rotate around x and y axes in 1 degree increments
  Xan++;
  Yan++;

  Yan = Yan % 360;
  Xan = Xan % 360; // prevents overflow.

  SetVars(); //sets up the global vars to do the 3D conversion.

  // Zoom in and out on Z axis within limits
  // the cube intersects with the screen for values < 160
  Zoff += inc;
  if (Zoff > 500)
    inc = -1; // Switch to zoom in
  else if (Zoff < 160)
    inc = 1; // Switch to zoom out

  for (int i = 0; i < LinestoRender; i++)
  {
    ORender[i] = Render[i];            // stores the old line segment so we can delete it later.
    ProcessLine(&Render[i], Lines[i]); // converts the 3d line segments to 2d.
  }
  RenderImage(); // go draw it!

  //delay(14); // Delay to reduce loop rate (reduces flicker caused by aliasing with TFT screen refresh rate)
  espDelay(14);
}

int offsetX = 0;
int offsetY = 100;

/***********************************************************************************************************************************/
void RenderImage(void)
{
  // renders all the lines after erasing the old ones.
  // in here is the only code actually interfacing with the OLED. so if you use a different lib, this is where to change it.

  for (int i = 0; i < OldLinestoRender; i++)
  {
    tft.drawLine(ORender[i].p0.x, ORender[i].p0.y + offsetY, ORender[i].p1.x, ORender[i].p1.y + offsetY, BLACK); // erase the old lines.
  }

  for (int i = 0; i < LinestoRender; i++)
  {
    uint16_t color = TFT_BLUE;
    if (i < 4)
      color = TFT_RED;
    if (i > 7)
      color = TFT_GREEN;
    tft.drawLine(Render[i].p0.x, Render[i].p0.y + offsetY, Render[i].p1.x, Render[i].p1.y + offsetY, color);
  }
  OldLinestoRender = LinestoRender;
}

/***********************************************************************************************************************************/
// Sets the global vars for the 3d transform. Any points sent through "process" will be transformed using these figures.
// only needs to be called if Xan or Yan are changed.
void SetVars(void)
{
  float Xan2, Yan2, Zan2;
  float _sx, _sy, _sz, _cx, _cy, _cz;

  Xan2 = Xan / fact; // convert degrees to radians.
  Yan2 = Yan / fact;
  Zan2 = Zan / fact;

  // Zan is assumed to be zero

  _sx = sin(Xan2);
  _sy = sin(Yan2);
  _sz = sin(Zan2);

  _cx = cos(Xan2);
  _cy = cos(Yan2);
  _cz = cos(Zan2);

  // https://en.wikipedia.org/wiki/3D_projection
  // extract partial multiplication from camera transformation matrix
  // and recalc these matrix multiplications (wolframalpha)
  // {{1, 0, 0},{0, cos(d_x), sin(d_x)},{0,-sin(d_x),cos(d_x)}} * {{cos(d_y),0,-sin(d_y)},{0,1,0},{sin(d_y),0,cos(d_y)}} * {{cos(d_z), sin(d_z),0},{-sin(d_z),cons(d_z),0},{0,0,1}}\
  // simplify to
  // {{1, 0, 0},{0, cosdx, sindx},{0,-sindx,cosdx}} * {{cosdy,0,-sindy},{0,1,0},{sindy,0,cosdy}} * {{cosdz, sindz,0},{-sindz,cosdz,0},{0,0,1}}
  // and write down as ...

  xx = (_cy * _cz);
  xy = _cy * _sz;
  xz = -_sy;

  yx = (_cz * _sx * _sy) - (_cx * _sz);
  yy = (_cx * _cz) + (_sx * _sy * _sz);
  yz = (_cy * _sx);

  zx = (_cx * _cz * _sy) + (_sx * _sz);
  zy = (_cx * _sy * _sz) - (_cz * _sx);
  zz = (_cx * _cy);
}

/***********************************************************************************************************************************/
// processes x1,y1,z1 and returns rx1,ry1 transformed by the variables set in SetVars()
// fairly heavy on floating point here.
// uses a bunch of global vars. Could be rewritten with a struct but not worth the effort.
void ProcessLine(struct Line2d *ret, struct Line3d vec)
{
  float zvt1;
  int xv1, yv1, zv1;

  float zvt2;
  int xv2, yv2, zv2;

  int rx1, ry1;
  int rx2, ry2;

  int x1;
  int y1;
  int z1;

  int x2;
  int y2;
  int z2;

  int Ok;

  x1 = vec.p0.x;
  y1 = vec.p0.y;
  z1 = vec.p0.z;

  x2 = vec.p1.x;
  y2 = vec.p1.y;
  z2 = vec.p1.z;

  Ok = 0; // defaults to not OK

  xv1 = (x1 * xx) + (y1 * xy) + (z1 * xz);
  yv1 = (x1 * yx) + (y1 * yy) + (z1 * yz);
  zv1 = (x1 * zx) + (y1 * zy) + (z1 * zz);

  zvt1 = zv1 - Zoff;

  if (zvt1 < -5)
  {
    rx1 = 256 * (xv1 / zvt1) + Xoff;
    ry1 = 256 * (yv1 / zvt1) + Yoff;
    Ok = 1; // ok we are alright for point 1.
  }

  xv2 = (x2 * xx) + (y2 * xy) + (z2 * xz);
  yv2 = (x2 * yx) + (y2 * yy) + (z2 * yz);
  zv2 = (x2 * zx) + (y2 * zy) + (z2 * zz);

  zvt2 = zv2 - Zoff;

  if (zvt2 < -5)
  {
    rx2 = 256 * (xv2 / zvt2) + Xoff;
    ry2 = 256 * (yv2 / zvt2) + Yoff;
  }
  else
  {
    Ok = 0;
  }

  if (Ok == 1)
  {

    ret->p0.x = rx1;
    ret->p0.y = ry1;

    ret->p1.x = rx2;
    ret->p1.y = ry2;
  }
  // The ifs here are checks for out of bounds. needs a bit more code here to "safe" lines that will be way out of whack, so they dont get drawn and cause screen garbage.
}

void setup_clock();

void play_clock()
{
  setup_clock();
}

/***********************************************************************************************************************************/
// line segments to draw a cube. basically p0 to p1. p1 to p2. p2 to p3 so on.
void cube(void)
{
  tft.fillScreen(TFT_BLACK);

  // Front Face.
  Lines[0].p0.x = -50;
  Lines[0].p0.y = -50;
  Lines[0].p0.z = 50;
  Lines[0].p1.x = 50;
  Lines[0].p1.y = -50;
  Lines[0].p1.z = 50;

  Lines[1].p0.x = 50;
  Lines[1].p0.y = -50;
  Lines[1].p0.z = 50;
  Lines[1].p1.x = 50;
  Lines[1].p1.y = 50;
  Lines[1].p1.z = 50;

  Lines[2].p0.x = 50;
  Lines[2].p0.y = 50;
  Lines[2].p0.z = 50;
  Lines[2].p1.x = -50;
  Lines[2].p1.y = 50;
  Lines[2].p1.z = 50;

  Lines[3].p0.x = -50;
  Lines[3].p0.y = 50;
  Lines[3].p0.z = 50;
  Lines[3].p1.x = -50;
  Lines[3].p1.y = -50;
  Lines[3].p1.z = 50;

  //back face.

  Lines[4].p0.x = -50;
  Lines[4].p0.y = -50;
  Lines[4].p0.z = -50;
  Lines[4].p1.x = 50;
  Lines[4].p1.y = -50;
  Lines[4].p1.z = -50;

  Lines[5].p0.x = 50;
  Lines[5].p0.y = -50;
  Lines[5].p0.z = -50;
  Lines[5].p1.x = 50;
  Lines[5].p1.y = 50;
  Lines[5].p1.z = -50;

  Lines[6].p0.x = 50;
  Lines[6].p0.y = 50;
  Lines[6].p0.z = -50;
  Lines[6].p1.x = -50;
  Lines[6].p1.y = 50;
  Lines[6].p1.z = -50;

  Lines[7].p0.x = -50;
  Lines[7].p0.y = 50;
  Lines[7].p0.z = -50;
  Lines[7].p1.x = -50;
  Lines[7].p1.y = -50;
  Lines[7].p1.z = -50;

  // now the 4 edge lines.

  Lines[8].p0.x = -50;
  Lines[8].p0.y = -50;
  Lines[8].p0.z = 50;
  Lines[8].p1.x = -50;
  Lines[8].p1.y = -50;
  Lines[8].p1.z = -50;

  Lines[9].p0.x = 50;
  Lines[9].p0.y = -50;
  Lines[9].p0.z = 50;
  Lines[9].p1.x = 50;
  Lines[9].p1.y = -50;
  Lines[9].p1.z = -50;

  Lines[10].p0.x = -50;
  Lines[10].p0.y = 50;
  Lines[10].p0.z = 50;
  Lines[10].p1.x = -50;
  Lines[10].p1.y = 50;
  Lines[10].p1.z = -50;

  Lines[11].p0.x = 50;
  Lines[11].p0.y = 50;
  Lines[11].p0.z = 50;
  Lines[11].p1.x = 50;
  Lines[11].p1.y = 50;
  Lines[11].p1.z = -50;

  LinestoRender = 12;
  OldLinestoRender = LinestoRender;
}

void setup_clock()
{
  //  tft.init();
  //   tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK); // Adding a black background colour erases previous text automatically

  // Draw clock face
  tft.fillCircle(64, 64, 61, TFT_BLUE);
  tft.fillCircle(64, 64, 57, TFT_BLACK);

  // Draw 12 lines
  for (int i = 0; i < 360; i += 30)
  {
    sx = cos((i - 90) * 0.0174532925);
    sy = sin((i - 90) * 0.0174532925);
    x0 = sx * 57 + 64;
    yy0 = sy * 57 + 64;
    x1 = sx * 50 + 64;
    yy1 = sy * 50 + 64;

    tft.drawLine(x0, yy0, x1, yy1, TFT_BLUE);
  }

  // Draw 60 dots
  for (int i = 0; i < 360; i += 6)
  {
    sx = cos((i - 90) * 0.0174532925);
    sy = sin((i - 90) * 0.0174532925);
    x0 = sx * 53 + 64;
    yy0 = sy * 53 + 64;

    tft.drawPixel(x0, yy0, TFT_BLUE);
    if (i == 0 || i == 180)
      tft.fillCircle(x0, yy0, 1, TFT_CYAN);
    if (i == 0 || i == 180)
      tft.fillCircle(x0 + 1, yy0, 1, TFT_CYAN);
    if (i == 90 || i == 270)
      tft.fillCircle(x0, yy0, 1, TFT_CYAN);
    if (i == 90 || i == 270)
      tft.fillCircle(x0 + 1, yy0, 1, TFT_CYAN);
  }

  tft.fillCircle(65, 65, 3, TFT_RED);

  // Draw text at position 64,125 using fonts 4
  // Only font numbers 2,4,6,7 are valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : . a p m
  // Font 7 is a 7 segment font and only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : .
  tft.drawCentreString("Time flies", 64, 130, 4);

  targetTime = millis() + 1000;
}

TaskHandle_t Task1;
TaskHandle_t Task2;

// LED pins
const int led1 = 2;
const int led2 = 4;

void Task2code(void *pvParameters);
void Task1code(void *pvParameters);

void setup_cube()
{
  cube();

  fact = 180 / 3.14159259; // conversion from degrees to radians.

  Xoff = 67; // Position the center of the 3d conversion space into the center of the TFT screen.
  Yoff = 120;
  Zoff = 1100; // Z offset in 3D space (smaller = closer and bigger rendering)
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Start");
  Serial.println(xPortGetCoreID());

  tft.init();

  h = tft.height();
  w = tft.width();

  setup_cube();

  tft.setRotation(1);

  tft.fillScreen(TFT_BLACK);

  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0, 0);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);

  showMenu();

  if (TFT_BL > 0)
  {                                         // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
    pinMode(TFT_BL, OUTPUT);                // Set backlight pin to output mode
    //digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
    digitalWrite(TFT_BL, TFT_DARKGREEN); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
  }

  tft.setSwapBytes(true);
  //tft.pushImage(0, 0,  240, 135, ttgo);
  espDelay(1000);

  tft.setRotation(0);
  int i = 5;
  while (i--)
  {
    tft.fillScreen(TFT_RED);
    //espDelay(1000);
    tft.fillScreen(TFT_BLUE);
    //espDelay(1000);
    tft.fillScreen(TFT_GREEN);
    //espDelay(1000);
  }

  button_init();

  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
  //Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
  {
    Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
    vref = adc_chars.vref;
  }
  else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
  {
    Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
  }
  else
  {
    Serial.println("Default Vref: 1100mV");
  }

  setup_clock();

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0

  xTaskCreatePinnedToCore(
      Task1code, // Task function. /
      "Task1",   //name of task. /
      10000,     // Stack size of task /
      NULL,      // parameter of the task /
      1,         // priority of the task /
      &Task1,    // Task handle to keep track of created task /
      0);        // pin task to core 0 /

  espDelay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
      Task2code, /* Task function. */
      "Task2",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task2,    /* Task handle to keep track of created task */
      1);        /* pin task to core 1 */
                 //espDelay(500);

#ifdef USE_DEMO_RADIO
  setup_radio();
#endif
}

void computeCA();

void loop_clock()
{
  if (targetTime < millis())
  {
    targetTime = millis() + 1000;
    ss++; // Advance second
    if (ss == 60)
    {
      ss = 0;
      mm++; // Advance minute
      if (mm > 59)
      {
        mm = 0;
        hh++; // Advance hour
        if (hh > 23)
        {
          hh = 0;
        }
      }
    }

    // Pre-compute hand degrees, x & y coords for a fast screen update
    sdeg = ss * 6;                     // 0-59 -> 0-354
    mdeg = mm * 6 + sdeg * 0.01666667; // 0-59 -> 0-360 - includes seconds
    hdeg = hh * 30 + mdeg * 0.0833333; // 0-11 -> 0-360 - includes minutes and seconds
    hx = cos((hdeg - 90) * 0.0174532925);
    hy = sin((hdeg - 90) * 0.0174532925);
    mx = cos((mdeg - 90) * 0.0174532925);
    my = sin((mdeg - 90) * 0.0174532925);
    sx = cos((sdeg - 90) * 0.0174532925);
    sy = sin((sdeg - 90) * 0.0174532925);

    if (ss == 0 || initial)
    {
      initial = 0;
      // Erase hour and minute hand positions every minute
      tft.drawLine(ohx, ohy, 65, 65, TFT_BLACK);
      ohx = hx * 33 + 65;
      ohy = hy * 33 + 65;
      tft.drawLine(omx, omy, 65, 65, TFT_BLACK);
      omx = mx * 44 + 65;
      omy = my * 44 + 65;
    }

    // Redraw new hand positions, hour and minute hands not erased here to avoid flicker
    tft.drawLine(osx, osy, 65, 65, TFT_BLACK);
    tft.drawLine(ohx, ohy, 65, 65, TFT_WHITE);
    tft.drawLine(omx, omy, 65, 65, TFT_WHITE);
    osx = sx * 47 + 65;
    osy = sy * 47 + 65;
    tft.drawLine(osx, osy, 65, 65, TFT_RED);

    tft.fillCircle(65, 65, 3, TFT_RED);
  }
}

void setup_life()
{
  //Set up the display
  // tft.init();
  //tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0, 0);
}

#include <vector>

struct Point {
    float x, y;

    // Constructs a new point given coordinates
    Point(float x, float y) : x(x), y(y) {}

    // Adds another point's coordinates to this point
    void add(const Point& other) {
        x += other.x;
        y += other.y;
    }

    // Scales the point coordinates by a factor
    void div(float value) {
        x /= value;
        y /= value;
    }

    // Rotates the point by an angle in degrees
    void rotate(float angle) {
        float rad = angle * PI / 180.0;
        float cosAngle = cos(rad);
        float sinAngle = sin(rad);
        float tx = x * cosAngle - y * sinAngle;
        float ty = x * sinAngle + y * cosAngle;
        x = tx;
        y = ty;
    }
    
    // Subtracts another point's coordinates from this point
    static Point sub(const Point& a, const Point& b) {
        return Point(a.x - b.x, a.y - b.y);
    }

    // Creates a copy of this point
    Point copy() const {
        return Point(x, y);
    }
};

class KochLine {
public:
    Point a, b;

    KochLine(const Point& start, const Point& end) : a(start), b(end) {}
    //KochLine(const Point& start, const Point& end) : a(end), b(start) {}

    void display() const {
        //drawLine(a.x, a.y, b.x, b.y, 0xFFFFFF); // Assuming white color
        tft.drawLine(a.x, a.y, b.x, b.y, WHITE); // Assuming white color
        //tft.drawLine(a.x, b.x, a.y, b.y, WHITE); // Assuming white color
        //tft.drawLine(a.x, b.y, a.y, b.x, WHITE); // Assuming white color
    }

    Point start() const {
        return a.copy();
    }

    Point end() const {
        return b.copy();
    }

    Point kochleft() const {
        Point v = Point::sub(b, a);
        v.div(3);
        v.add(a);
        return v;
    }

    Point kochmiddle() const {
        Point v = Point::sub(b, a);
        v.div(3);
        
        Point p = a.copy();
        p.add(v);
        
        v.rotate(-60); // Rotating by -60 degrees
        p.add(v);
        
        return p;
    }

    Point kochright() const {
        Point v = Point::sub(a, b);
        v.div(3);
        v.add(b);
        return v;
    }
};

class KochFractal {
    Point start, end;
    std::vector<KochLine> lines;
    int count = 0;

public:
    //KochFractal(int width, int height) : start(0, height - 20), end(width, height - 20) {
    KochFractal(int width, int height) : start(20, 0), end(20, height) {
        restart();
    }

    /**
    Calculate nex level lines
    */
    void nextLevel() {
        Serial.println("loop_fractals - nl1");
        std::vector<KochLine> nextLines;
        for (auto& line : lines) {
            Point a = line.start();
            Point b = line.kochleft();
            Point c = line.kochmiddle();
            Point d = line.kochright();
            Point e = line.end();
            nextLines.push_back(KochLine(a, b));
            nextLines.push_back(KochLine(b, c));
            nextLines.push_back(KochLine(c, d));
            nextLines.push_back(KochLine(d, e));
        }
        lines = nextLines;
        count++;
    }

    void restart() {
        count = 0;
        lines.clear();
        lines.push_back(KochLine(start, end));
    }

    int getCount() const {
        return count;
    }

    void render() const {
        Serial.println("loop_fractals - render");
        for (const auto& line : lines) {
          //Serial.print("Line: %d %d %d %d ", line.a.x, line.a.y, line.b.x, line.b.y);
          line.display();
        }
    }
};

// Global KochFractal object

void setup_fractals()
{
  //Set up the display
  // tft.init();
  //tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0, 0);
}



void loop_fractals() {
  KochFractal koch(135, 240); // Assuming screen dimensions
  Serial.println("loop_fractals");
  //Display a simple splash screen
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(40, 5);
  tft.println(F("Arduino"));
  tft.setCursor(35, 25);
  tft.println(F("Fractals"));
  Serial.println("loop_fractals - 1");

  espDelay(1000);
  // 6 is a limit for math precision
  for (int i = 0; i < 6; i++) {
    tft.fillScreen(TFT_BLACK);
    koch.render();
    koch.nextLevel();
    espDelay(1000); // Slow down the animation
  }
  koch.restart();

  fractalsPlaying = false;
  inMenu = true;
}

void initGrid();
void drawGrid();

void initFGrid();
void drawFGrid();
int mandel(int x, int y) {
  int a=0; int b=0;
  for (int i = 0; i<250; ++i) {
    // Complex z = z^2 + c
    int  t = a*a - b*b;
    b = 2*a*b;
    a = t;
    a = a + x;
    b = b + y;
    int m = a*a + b*b;
    if (m > 10) {
      return i;
    }
  }
  return 250;
}


void loop_life()
{
  //Display a simple splash screen
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(40, 5);
  tft.println(F("Arduino"));
  tft.setCursor(35, 25);
  tft.println(F("Cellular"));
  tft.setCursor(35, 45);
  tft.println(F("Automata"));

  espDelay(1000);

  tft.fillScreen(TFT_BLACK);

  initFGrid();

  genCount = 2; //MAX_GEN_COUNT;

  drawFGrid();

  //Compute generations
  for (int gen = 0; gen < genCount; gen++)
  {
    if (btnLClick)
    {
      lifePlaying = false;
      inMenu = true;
      return;
    }
    computeCA();
    drawFGrid();
    espDelay(GEN_DELAY);
    for (int16_t x = 1; x < GRIDX - 1; x++)
    {
      for (int16_t y = 1; y < GRIDY - 1; y++)
      {
        
        fgrid[x][y] = mandel(x, y); //newgrid[x][y];
      }
    }
  }
  lifePlaying = false;
  inMenu = true;
  //delay(1000);
}

//Draws the grid on the display
void drawGrid(void)
{
  uint16_t color = TFT_WHITE;
  for (int16_t x = 1; x < GRIDX - 1; x++)
  {
    for (int16_t y = 1; y < GRIDY - 1; y++)
    {
      if ((grid[x][y]) != (newgrid[x][y]))
      {
        if (newgrid[x][y] == 1)
          color = 0xFFFF; //random(0xFFFF);
        else
          color = 0;
        tft.fillRect(CELLXY * x, CELLXY * y, CELLXY, CELLXY, color);
      }
    }
  }
}

//Initialise Grid
void initGrid(void)
{
  for (int16_t x = 0; x < GRIDX; x++)
  {
    for (int16_t y = 0; y < GRIDY; y++)
    {
      newgrid[x][y] = 0;

      if (x == 0 || x == GRIDX - 1 || y == 0 || y == GRIDY - 1)
      {
        grid[x][y] = 0;
      }
      else
      {
        if (random(3) == 1)
          grid[x][y] = 1;
        else
          grid[x][y] = 0;
      }
    }
  }
}

//Draws the grid on the display
void drawFGrid(void)
{
  uint16_t color = TFT_WHITE;
  for (int16_t x = 1; x < FGRIDX - 1; x++)
  {
    for (int16_t y = 1; y < FGRIDY - 1; y++)
    {
          color = fnewgrid[x][y];
        tft.fillRect(FCELLXY * x, FCELLXY * y, FCELLXY, FCELLXY, color);
      /*
      if ((grid[x][y]) != (fnewgrid[x][y]))
      {
        if (fnewgrid[x][y] == 1)
          color = 0xFFFF; //random(0xFFFF);
        else
          color = 0;
        tft.fillRect(CELLXY * x, CELLXY * y, CELLXY, CELLXY, color);
      }
      */
    }
  }
}

//Initialise Grid
void initFGrid(void)
{
  for (int16_t x = 0; x < FGRIDX; x++)
  {
    for (int16_t y = 0; y < FGRIDY; y++)
    {
      newgrid[x][y] = 0;
    /*
      if (x == 0 || x == FGRIDX - 1 || y == 0 || y == FGRIDY - 1)
      {
        grid[x][y] = 0;
      }
      else
      {
        if (random(3) == 1)
          grid[x][y] = 1;
        else
          grid[x][y] = 0;
      }
      */
    }
  }
}

int getNumberOfNeighbors(int, int);

//Compute the CA. Basically everything related to CA starts here
void computeCA()
{
  for (int16_t x = 1; x < GRIDX; x++)
  {
    for (int16_t y = 1; y < GRIDY; y++)
    {
      int neighbors = getNumberOfNeighbors(x, y);
      if (grid[x][y] == 1 && (neighbors == 2 || neighbors == 3))
      {
        newgrid[x][y] = 1;
      }
      else if (grid[x][y] == 1)
        newgrid[x][y] = 0;
      if (grid[x][y] == 0 && (neighbors == 3))
      {
        newgrid[x][y] = 1;
      }
      else if (grid[x][y] == 0)
        newgrid[x][y] = 0;
    }
  }
}

// Check the Moore neighborhood
int getNumberOfNeighbors(int x, int y)
{
  return grid[x - 1][y] + grid[x - 1][y - 1] + grid[x][y - 1] + grid[x + 1][y - 1] + grid[x + 1][y] + grid[x + 1][y + 1] + grid[x][y + 1] + grid[x - 1][y + 1];
}

#ifdef USE_DEMO_GY25
void loop_gy25_cube()
{
  // TODO fix ypr conversion when Xan over 90° makes Yan=Yan+180 ...

  // Rotate around x and y axes in 1 degree increments
  // Xan++;
  // Yan++;
  // String GY25Data = "ypr:\t" + String(ypr[0] * 180 / M_PI) + "\t" + String(ypr[1] * 180 / M_PI) + "\t" +  String(ypr[2] * 180 / M_PI);

  // Yan = Yan % 360;
  // Xan = Xan % 360; // prevents overflow.

  SetVars(); //sets up the global vars to do the 3D conversion.

  // Zoom in and out on Z axis within limits
  // the cube intersects with the screen for values < 160
  // Zoff += inc;
  // if (Zoff > 500) inc = -1;     // Switch to zoom in
  // else if (Zoff < 160) inc = 1; // Switch to zoom out

  Xan = -ypr[2] * 180 / M_PI;
  Yan = -ypr[1] * 180 / M_PI;
  Zan = -ypr[0] * 180 / M_PI;
  // prevents overflow.
  Xan = Xan % 360;
  Yan = Yan % 360;
  Zan = Zan % 360;

  if (Xan >= 90)
  {
    Yan = 180 - Yan;
    Zan = 180 - Zan;
  }
  else if (Xan < -90)
  {
    Yan += 180;
  }

  Zoff = 500;

  for (int i = 0; i < LinestoRender; i++)
  {
    ORender[i] = Render[i];            // stores the old line segment so we can delete it later.
    ProcessLine(&Render[i], Lines[i]); // converts the 3d line segments to 2d.
  }
  RenderImage(); // go draw it!

  espDelay(14); // Delay to reduce loop rate (reduces flicker caused by aliasing with TFT screen refresh rate)
}
#endif

void Task1code(void *pvParameters)
{
  static uint64_t timeStamp = 0;
  for (;;)
  {
    if (millis() - timeStamp > 1000)
    {
      Serial.print("Task1 running on core ");
      Serial.println(xPortGetCoreID());
      timeStamp = millis();
    }
    espDelay(1000);
  }
  vTaskDelete(NULL);
}

void Task2code(void *pvParameters)
{
  static uint64_t timeStamp = 0;
  for (;;)
  {
    if (millis() - timeStamp > 1000)
    {
      Serial.print("Task2 running on core ");
      Serial.println(xPortGetCoreID());
      // Serial.print(" delay ");
      // Serial.println((int)(millis() - timeStamp));
      timeStamp = millis();
    }

    if (inMenu)
    {
      showMenu();
    }
    else
    {
      if (btnLClick && (cubePlaying || lifePlaying))
      {
        cubePlaying = false;
        lifePlaying = false;
        inMenu = true;
      }
      else if (cubePlaying)
      {
        loop_cube();
      }
      else if (clockPlaying)
      {
        loop_clock();
      }
      else if (lifePlaying)
      {
        loop_life();
      }
      else if (fractalsPlaying)
      {
         loop_fractals();
      }
#ifdef USE_DEMO_GY25
      else if (gy25Playing)
      {
        loop_gy25();
        showGY25Data();
        loop_gy25_cube();
      }
#endif
    }
    button_loop();

    // just in case .. to avoid https://github.com/espressif/arduino-esp32/issues/922
    espDelay(1);
  }
  vTaskDelete(NULL);
}
