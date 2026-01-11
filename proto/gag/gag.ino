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

#ifdef ESP32_RIGHT
#include "esp_pm.h"
#endif

// #ifdef USE_DISPLAY
//     extern SSD1306Wire display;
//     extern OLEDDisplayUi ui;
//     extern int remainingTimeBudget;
// #endif

// #define USE_VISUALIZATION 1
#define USE_VISUALIZATION 1 

#ifdef USE_VISUALIZATION
// #include "gag_display.cpp"
#include "gag_display.h"
#endif
// void viz_init();

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
    // #ifdef ESP32_RIGHT
    #ifdef USE_BT_GATT_SERIAL
    esp_sleep_enable_timer_wakeup(0);  // Prevent sleep mode
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT); // Ensure only BLE is active
    // #endif
    #endif
    
    #ifdef GAG_DEBUG 
        Serial.begin(115200);
    #elif 
     
        Serial.begin(115200);
    #endif

    #ifdef USE_VISUALIZATION
        // MASTER_SERIAL_NAME.println(F("Init vis2"));
        // MASTER_SERIAL_NAME.println(F("Init vis3"));
        // viz_init();
        // if (viz_init == nullptr) Serial.println("viz_init pointer is NULL!?");
        viz_init();
        // MASTER_SERIAL_NAME.println(F("Init vis - done"));
        // Optional tweaks:
        // viz_set_deg_spacing(18.0f);     // tighter palm fan
        // viz_use_perspective(true);      // simple perspective (try off first on 1-bit OLED)
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
    
    // MASTER_SERIAL_NAME.println(F("USB up"));
    
// #ifdef USE_DISPLAY
//     displaySetup();
// #endif

    for (int i = FIRST_SENSOR; i <= LAST_SENSOR; i++) {
      int sensorToEnable = selectSingleMPU(i);
      pinMode(sensorToEnable, OUTPUT);
      digitalWrite(sensorToEnable, HIGH);
    }
    pinMode(SENSOR_PIN_HP_COMPENSATION, OUTPUT);
    digitalWrite(SENSOR_PIN_HP_COMPENSATION, HIGH);

#ifdef MASTER_HAND
    Wire.begin();
   
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
    MASTER_SERIAL_NAME.println(F("Setup - done"));
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {

    int currentlySellectedSensor = selectedSensor;
  
    handSwitchPrev = timeNow;
    timePrev = timeNow; // the previous time is stored before the actual time read
    timeNow = millis(); // actual time read
    elapsedTime = (timeNow - timePrev);
 
// #ifdef USE_DISPLAY
//     remainingTimeBudget = ui.update();
// #endif

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

// #ifdef USE_VISUALIZATION
//   VizQuaternion q[GAG_NUM_SENSORS];

//   // Fill from your existing data.
//   // Example if you have: gyros[i].q: {w, x, y, z}
//   for (int i = 0; i < GAG_NUM_SENSORS; ++i) {
//     q[i].w = gyros[i].q.w;
//     q[i].x = gyros[i].q.x;
//     q[i].y = gyros[i].q.y;
//     q[i].z = gyros[i].q.z;
//   }

//   // Safety for missing sensors: set identity for any not ready
//   // e.g., if (!gyros[i].dmpReady) q[i] = {1,0,0,0};

//   viz_draw_frame(q);
// #endif


#ifdef USE_VISUALIZATION
  // Uncomment to enable detailed debug logs
//   #define VIZ_DEBUG 1

  // Helper: extract quaternion from DMP FIFO bytes
  auto quatFromFifo = [](const byte* fb, int idx) -> VizQuaternion {
    // Combine big-endian bytes into signed 16-bit integers
    int16_t qw = (int16_t)((int16_t)fb[0]  << 8 | fb[1]);
    int16_t qx = (int16_t)((int16_t)fb[4]  << 8 | fb[5]);
    int16_t qy = (int16_t)((int16_t)fb[8]  << 8 | fb[9]);
    int16_t qz = (int16_t)((int16_t)fb[12] << 8 | fb[13]);

    // Convert to float — DMP quaternions are typically Q14 (16384 = 1.0)
    const float S = 1.0f / 16384.0f;
    VizQuaternion qf{
      (float)qw * S,
      (float)qx * S,
      (float)qy * S,
      (float)qz * S
    };

    // Normalize to unit quaternion
    float n = sqrtf(qf.w*qf.w + qf.x*qf.x + qf.y*qf.y + qf.z*qf.z);
    if (n > 1e-6f) {
      qf.w /= n; qf.x /= n; qf.y /= n; qf.z /= n;
    } else {
      qf = {1.f, 0.f, 0.f, 0.f};
    }

  #ifdef VIZ_DEBUG
    Serial.print(F("[VIZ] Sensor ")); Serial.print(idx);
    Serial.print(F(" raw qw=")); Serial.print(qw);
    Serial.print(F(" qx=")); Serial.print(qx);
    Serial.print(F(" qy=")); Serial.print(qy);
    Serial.print(F(" qz=")); Serial.print(qz);
    Serial.print(F(" | norm q=("));
    Serial.print(qf.w, 4); Serial.print(", ");
    Serial.print(qf.x, 4); Serial.print(", ");
    Serial.print(qf.y, 4); Serial.print(", ");
    Serial.print(qf.z, 4); Serial.println(")");
  #endif

    return qf;
  };

  VizQuaternion q[GAG_NUM_SENSORS];

  for (int i = 0; i < GAG_NUM_SENSORS; ++i) {
    if (gyros[i].fifoBuffer) {
      q[i] = quatFromFifo(gyros[i].fifoBuffer, i);
    } else {
      q[i] = {1.f, 0.f, 0.f, 0.f};
    }
  }

  viz_draw_frame(q);
#endif

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
