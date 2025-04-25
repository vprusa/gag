/**
 * Copyright (c) 2018 Vojtěch Průša
 * 
 * -- I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
 * -- Based on example of MPU6050 from https://github.com/jrowberg/i2cdevlib
 * -- from 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
 * TODO's:
 * - dedicate cores:
 * -- 1 core for retrieving data from sensors
 * -- 1 core for:
 * --- sending data over BLE
 * --- synchronizing communication between app <-> master <-> agent
*/

#define ESP32_RIGHT 1
//#define USE_DISPLAY 1

//#define MEASURE_OFFSETS 1
#include "definitions.h"
#include "gag.h"
#include "Wire.h"
// #include <WiFi.h>
// #include "esp_wifi.h"  // Required for WiFi power management functions

#ifdef ESP32_RIGHT
#include "esp_pm.h"
#endif

#ifdef USE_DISPLAY
    extern SSD1306Wire display;
    extern OLEDDisplayUi ui;
    extern int remainingTimeBudget;
#endif

#ifdef MEASURE_OFFSETS
    //#include "gag_offsetting.h"
    extern bool calibrationDone;
#endif
    
#ifdef USE_BT_GATT_SERIAL
    ServerCallbacks * sclbk;
    CharCallbacks * cclbk;
#endif

#ifdef MASTER_HAND
extern bool useSlaveHand = false;
#endif
void setup() {
  // #ifdef SLAVE_HAND
  // Serial.begin(57600);
  // Serial.println("SetupSlaveHand");
  // Serial.flush();
  // Serial.end();
  // #endif
   #ifdef ESP32_RIGHT
   esp_sleep_enable_timer_wakeup(0);  // Prevent sleep mode
   esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT); // Ensure only BLE is active
  //  esp_pm_config_esp32_t pm_config = {
  //     .max_freq_mhz = 240,  // Max CPU frequency
  //     .min_freq_mhz = 80,   // Min CPU frequency
  //     .light_sleep_enable = false
  //   };
  //  esp_pm_configure(&pm_config);
  // esp_wifi_set_ps(WIFI_PS_NONE);
  // setCpuFrequencyMhz(240);  // Set max CPU frequency
  // WiFi.disconnect(true);
  // WiFi.mode(WIFI_OFF);
  // btStop();  // Stops Bluetooth Classic, leaving BLE active
    #endif
    
    #ifdef GAG_DEBUG 
        Serial.begin(115200);
    #elif SEND_DATA_ALSO_OVER_SERIAL 
        Serial.begin(115200);
    #endif

    #ifdef USE_BT_GATT_SERIAL
        sclbk = new ServerCallbacks();
        cclbk = new CharCallbacks();
        //SerialBT = new BluetoothSerial();
        MASTER_SERIAL_NAME.begin(MASTER_BT_SERIAL_NAME, sclbk, cclbk);
    #endif
    #ifdef MASTER_BT_SERIAL
        #ifndef USE_BT_GATT_SERIAL
           // MASTER_SERIAL_NAME.begin(MASTER_BT_SERIAL_NAME);
        #endif
    #else
    MASTER_SERIAL_NAME.begin(MASTER_SERIAL_BAUD);
    while (!MASTER_SERIAL_NAME)
        ; // wait for Leonardo enumeration, others continue immediately
    #endif
    
    MASTER_SERIAL_NAME.println(F("USB up"));
    
#ifdef USE_DISPLAY
    displaySetup();
#endif

    for (int i = FIRST_SENSOR; i <= LAST_SENSOR; i++) {
      int sensorToEnable = selectSingleMPU(i);
      pinMode(sensorToEnable, OUTPUT);
      digitalWrite(sensorToEnable, HIGH);
    }
    pinMode(SENSOR_PIN_HP_COMPENSATION, OUTPUT);
    digitalWrite(SENSOR_PIN_HP_COMPENSATION, HIGH);

#ifdef MASTER_HAND
    Wire.begin();
    //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    //Wire.begin(21 , 22, 200000);
    //Wire.setTimeOut(2);
    //Fastwire::setup(400, true);
    //Wire.begin();
    Wire.setClock(100000);
#else
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
        // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        //Fastwire::setup(400, true);
    #endif
#endif

// initialize serial communication
// (9600 38400 57600 74880 115200 230400 250000 57600 38400 chosen because it is required for output, but it's
// really up to you depending on your project)
#ifdef MASTER_HAND
    //TODO fix, rename
    SLAVE_SERIAL_NAME.begin(SLAVE_SERIAL_BAUD);
#endif

    setupSensors();
    #ifdef MEASURE_OFFSETS
    calibrationDone = true;
    #endif
    timeNow = millis(); //Start counting time in milliseconds
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {

    int currentlySellectedSensor = selectedSensor;
    // enableSingleMPU(HG);
    // setOrRotateSelectedGyro(HG);
    // loadHGData(HG);
    // enableSingleMPU(currentlySellectedSensor);
    
    handSwitchPrev = timeNow;
    timePrev = timeNow; // the previous time is stored before the actual time read
    timeNow = millis(); // actual time read
    elapsedTime = (timeNow - timePrev);
 
#ifdef USE_DISPLAY
  //if(elapsedTime > 10){
    remainingTimeBudget = ui.update();
    if (elapsedTime - remainingTimeBudget > 0) {
        // Serial.println("Remaining time budget:");
        //Serial.println(remainingTimeBudget);
    }

#endif

#ifdef MASTER_HAND
    if(useSlaveHand) {
        masterHandDataRequestHandler();
        loadSlaveHandData();
    }else{
        uint8_t limit = REPEAT_MASTER_HAND_READ_LIMIT;
        uint8_t endOfPacketAlign = 0;
        int8_t readAlign = 0;
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
                &ch)){ break; }
            limit--;
            GAG_DEBUG_PRINT("limit ");
            GAG_DEBUG_PRINTLN(limit);
        }
    }

    gyros[selectedSensor].alreadySentData = false;
    loadDataAndSendPacket();

    setOrRotateSelectedGyro(-1); 
    loadDataFromFIFO(true);
    
#endif

#ifdef SLAVE_HAND
    slaveHandDataRequestHandler();
#endif
    automaticFifoReset();
#ifdef MASTER_HAND
    if(useSlaveHand) {
        loadSlaveHandData();
    }
#endif

}
