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

// ================================================================
// === Gesture recognition integration (gag-recog-lib)            ===
// ================================================================
// Set to 0 if you want to compile the firmware without the recognition library.
// #ifndef USE_GAG_RECOG
#define USE_GAG_RECOG 1
// #endif

#if USE_GAG_RECOG

#define GAG_DEBUG_RECOG
#ifdef GAG_DEBUG_RECOG
    #define GAG_DEBUG_RECOG_PRINT(x) Serial.print(x)
    #define GAG_DEBUG_RECOG_PRINTF(x, y) Serial.print(x, y)
    #define GAG_DEBUG_RECOG_PRINTLN(x) Serial.println(x)
    #define GAG_DEBUG_RECOG_PRINTLNF(x, y) Serial.println(x, y)
    #define GAG_DEBUG_RECOG_WRITE(x) Serial.write(x)
    #define GAG_DEBUG_RECOG_WRITE_LEN(x,y) Serial.write(x,y)
#else
    #define GAG_DEBUG_RECOG_PRINT(x)
    #define GAG_DEBUG_RECOG_PRINTF(x, y) 
    #define GAG_DEBUG_RECOG_PRINTLN(x) 
    #define GAG_DEBUG_RECOG_PRINTLNF(x, y)
    #define GAG_DEBUG_RECOG_WRITE(x)
    #define GAG_DEBUG_RECOG_WRITE_LEN(x, y)
#endif

  #include "GagRecog.h"
  #include "GagRecogSerialLoader.h"

  static gag::Recognizer g_recog;
  static gag::SerialLoader g_recogLoader(g_recog);

  // By default we silence library prints to avoid corrupting the binary packet stream.
  // (Recognitions are handled via callback + visualization flash.)
  class NullPrint : public Print {
   public:
    size_t write(uint8_t) override { return 1; }
  };
  static NullPrint g_recogNullOut;

  static inline gag::Sensor mapGagSensorToRecog(uint8_t gagIdx) {
    // Your sensor order: 0..4 = fingers (TU..EU), 5 = wrist (HG)
    switch (gagIdx) {
      case 5: return gag::Sensor::WRIST;
      case 0: return gag::Sensor::THUMB;
      case 1: return gag::Sensor::INDEX;
      case 2: return gag::Sensor::MIDDLE;
      case 3: return gag::Sensor::RING;
      case 4: return gag::Sensor::LITTLE;
      default: return gag::Sensor::WRIST;
    }
  }

  static inline uint8_t mapRecogMaskToGagMask(uint8_t libMask) {
    // Library order bits: 0=WRIST,1=THUMB,2=INDEX,3=MIDDLE,4=RING,5=LITTLE
    // Project order bits: 0=TU,1=SU,2=FU,3=MU,4=EU,5=HG
    uint8_t m = 0;
    if (libMask & (1u << 0)) m |= (1u << 5);
    if (libMask & (1u << 1)) m |= (1u << 0);
    if (libMask & (1u << 2)) m |= (1u << 1);
    if (libMask & (1u << 3)) m |= (1u << 2);
    if (libMask & (1u << 4)) m |= (1u << 3);
    if (libMask & (1u << 5)) m |= (1u << 4);
    return m;
  }

  static void onGestureRecognized(const gag::RecognizedGesture& gr) {
    // Flash the sensors used by the recognized gesture for 100ms.
    #ifdef USE_VISUALIZATION
      const uint8_t libMask = g_recog.getGestureSensorMaskByName(gr.name);
      const uint8_t gagMask = mapRecogMaskToGagMask(libMask);
      viz_flash_sensors(gagMask, getGestureColour(gr.name), 100);
    #endif
  }

  static inline void feedRecognizerFromSelectedSensor() {
    if (!gyros[selectedSensor].q) {
      return;
    }
    // GAG_DEBUG_RECOG_PRINTLN("feeding");
    const uint8_t idx = (uint8_t)selectedSensor;
    gag::Quaternion q(
      (float)gyros[selectedSensor].q->w,
      (float)gyros[selectedSensor].q->x,
      (float)gyros[selectedSensor].q->y,
      (float)gyros[selectedSensor].q->z
    );
    q.normalizeInPlace();
    g_recog.processSample(mapGagSensorToRecog(idx), q, millis());
  }
#endif // USE_GAG_RECOG

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

    // ---- Recognition library init ----
    #if USE_GAG_RECOG
        g_recog.begin(g_recogNullOut);
        g_recog.setOnRecognized(onGestureRecognized);
        // Read gesture definitions from the same stream as commands, but only
        // when the next byte is 'G'/'g' (see loop()).
        g_recogLoader.begin(MASTER_SERIAL_NAME, g_recogNullOut);
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
    // --- Gesture serial loader (only consumes input if it starts with 'G'/'g') ---
    bool handledGestureLine = false;
    #if USE_GAG_RECOG
        if (MASTER_SERIAL_NAME.available() > 0) {
            int p = MASTER_SERIAL_NAME.peek();
            if (p == 'G' || p == 'g') {
                g_recogLoader.poll();
                handledGestureLine = true;
            }
        }
    #endif

    if(useSlaveHand) {
        masterHandDataRequestHandler();
        loadSlaveHandData();
    }else{
        if (handledGestureLine) {
            // Skip the binary command parser for this loop tick.
        } else {
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
    }

    gyros[selectedSensor].alreadySentData = false;

    // Read FIFO if needed and send a packet for the currently selected sensor.
    // (This keeps the original round-robin sensor switching logic intact.)
    loadDataFromFIFO(false);

    #if USE_GAG_RECOG
        if (gyros[selectedSensor].hasDataReady) {
            feedRecognizerFromSelectedSensor();
        }
    #endif

    if (gyros[selectedSensor].hasDataReady) {
        writePacket();
    }

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
