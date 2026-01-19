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
//#include "definitions.h"
//#include "gag.h"
//#include "Wire.h"V
//#include <cty// ================================================================
#include "definitions.h"
#include "gag.h"
#include "Wire.h"
#include <ctype.h>
#include <string.h>
#include <strings.h>
#include "GagRecog.h"
#include "GagRecogSerialLoader.h"
// === BLE HID (Mouse) command output                            ===
// ================================================================
//
// Uses the "ESP32 BLE Mouse" library by T-vK:
//   https://github.com/T-vK/ESP32-BLE-Mouse
//
// Notes:
//  - Only MASTER_HAND sends HID events (the hand connected to the computer).
//  - If you also enable USE_BT_GATT_SERIAL (custom BLE serial), you will likely
//    need to merge BLE services (both code paths typically call BLEDevice::init()).
//

// Set to 0 to compile without BLE HID mouse output.
#define USE_BLE_HID_MOUSE 1

// Normalizes a command string:
//  - uppercase
//  - collapses any non-alnum separators into a single underscore '_'
static void gagNormalizeCommand(const char* in, char* out, size_t outLen) {
  if (!out || outLen == 0) return;
  out[0] = '\0';
  if (!in) return;

  size_t j = 0;
  bool prevUnderscore = false;

  // Skip leading whitespace
  while (*in == ' ' || *in == '\t' || *in == '\r' || *in == '\n') ++in;

  for (size_t i = 0; in[i] != '\0' && j + 1 < outLen; ++i) {
    const unsigned char c = (unsigned char)in[i];

    if (isalnum(c)) {
      out[j++] = (char)toupper(c);
      prevUnderscore = false;
    } else {
      if (!prevUnderscore && j > 0) {
        out[j++] = '_';
        prevUnderscore = true;
      }
    }
  }

  while (j > 0 && out[j - 1] == '_') --j;
  out[j] = '\0';
}

// Optional hook for your OLED "command log" text area.
// If your visualization code provides a strong definition, it will override this.
__attribute__((weak)) void viz_log_command(const char* line) { (void)line; }
__attribute__((weak)) void viz_log_label(const char* label, bool commandsEnabled) { (void)label; (void)commandsEnabled; }


#if USE_BLE_HID_MOUSE && defined(MASTER_HAND)
  #include <BleMouse.h>

  // Device name shown on the laptop/PC when pairing.
  static BleMouse g_bleMouse("GAG Mouse", "vprusa/gag", 100);

  static uint8_t gagMouseButtonFromToken(const char* token) {
    if (!token) return 0;
    if (!strcasecmp(token, "LEFT")) return MOUSE_LEFT;
    if (!strcasecmp(token, "RIGHT")) return MOUSE_RIGHT;
    if (!strcasecmp(token, "MIDDLE")) return MOUSE_MIDDLE;
    if (!strcasecmp(token, "BACK")) return MOUSE_BACK;
    if (!strcasecmp(token, "FORWARD")) return MOUSE_FORWARD;
    return 0;
  }

  // Executes a normalized command string (already passed through gagNormalizeCommand()).
  static bool gagExecuteBleMouseCommandNorm(const char* cmdNorm) {
    if (!cmdNorm || cmdNorm[0] == '\0') return false;
    if (!g_bleMouse.isConnected()) return false;

    // Accept both "MOUSE_..." and "...".
    const char* s = cmdNorm;
    if (!strncmp(s, "MOUSE_", 6)) s += 6;

    // SCROLL/WHEEL commands:
    //   MOUSE_SCROLL_UP / MOUSE_SCROLL_DOWN / MOUSE_SCROLL_LEFT / MOUSE_SCROLL_RIGHT
    //   MOUSE_WHEEL_UP  / MOUSE_WHEEL_DOWN  (aliases)
    //
    // Per library README:
    //   - vertical scroll: bleMouse.move(0,0,-1) => scroll down
    //   - horizontal scroll: bleMouse.move(0,0,0,1) => scroll left, (0,0,0,-1) => scroll right
    if (!strncmp(s, "SCROLL_", 7) || !strncmp(s, "WHEEL_", 6)) {
      const char* dir = strchr(s, '_');
      if (!dir || dir[1] == '\0') return false;
      dir++; // after '_'

      if (!strcasecmp(dir, "UP"))    { g_bleMouse.move(0, 0, +1); return true; }
      if (!strcasecmp(dir, "DOWN"))  { g_bleMouse.move(0, 0, -1); return true; }
      if (!strcasecmp(dir, "LEFT"))  { g_bleMouse.move(0, 0, 0, +1); return true; }
      if (!strcasecmp(dir, "RIGHT")) { g_bleMouse.move(0, 0, 0, -1); return true; }

      return false;
    }

    // Button commands:
    //   <BTN>_<ACTION>
    // Example: LEFT_CLICK, BACK_CLICK, FORWARD_CLICK, RIGHT_DOWN, ...
    const char* us = strchr(s, '_');
    if (!us) return false;

    char btnTok[16] = {0};
    char actTok[16] = {0};

    const size_t btnLen = (size_t)(us - s);
    if (btnLen == 0 || btnLen >= sizeof(btnTok)) return false;
    memcpy(btnTok, s, btnLen);
    btnTok[btnLen] = '\0';

    strncpy(actTok, us + 1, sizeof(actTok) - 1);

    const uint8_t btn = gagMouseButtonFromToken(btnTok);
    if (btn == 0) return false;

    if (!strcasecmp(actTok, "CLICK")) { g_bleMouse.click(btn); return true; }
    if (!strcasecmp(actTok, "DOWN") || !strcasecmp(actTok, "PRESS")) { g_bleMouse.press(btn); return true; }
    if (!strcasecmp(actTok, "UP") || !strcasecmp(actTok, "RELEASE")) { g_bleMouse.release(btn); return true; }

    return false;
  }

  static bool gagExecuteBleMouseCommand(const char* commandRaw) {
    if (!commandRaw || commandRaw[0] == '\0') return false;

    char cmd[64];
    gagNormalizeCommand(commandRaw, cmd, sizeof(cmd));
    return gagExecuteBleMouseCommandNorm(cmd);
  }
#endif



// ================================================================
// === Gesture recognition integration (gag-recog-lib)            ===
// ================================================================
// Set to 0 if you want to compile the firmware without the recognition library.
#define USE_GAG_RECOG 1

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


static gag::Recognizer g_recog;
static gag::SerialLoader g_recogLoader(g_recog);

// By default we silence library prints to avoid corrupting the binary packet stream.
// (Recognitions are handled via callback + visualization flash.)
class NullPrint : public Print {
 public:
  size_t write(uint8_t) override { return 1; }
};
static NullPrint g_recogNullOut;

// Command execution enable/disable gate.
// Requirement: initial state disabled; only thumb-toggle gesture can change it.
static bool g_commandsEnabled = false;

// Special command string used by the "thumb toggle" gesture.
static const char* GAG_TOGGLE_COMMAND = "GAG_TOGGLE_COMMANDS";

static inline bool gagIsToggleCommand(const char* cmdRaw) {
  if (!cmdRaw || cmdRaw[0] == '\0') return false;
  char c[64];
  gagNormalizeCommand(cmdRaw, c, sizeof(c));
  // Accept a couple of aliases for convenience.
  return (!strcmp(c, "GAG_TOGGLE_COMMANDS") ||
          !strcmp(c, "TOGGLE_COMMANDS") ||
          !strcmp(c, "CMD_TOGGLE_COMMANDS") ||
          !strcmp(c, "GAG_TOGGLE"));
}

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
  const char* cmdRaw = (gr.command && gr.command[0] != '\0') ? gr.command : gr.name;
  const char* lblRaw = (gr.label && gr.label[0] != '\0') ? gr.label : cmdRaw;

  // Always flash sensors for UI feedback (even when command output is disabled).
  #ifdef USE_VISUALIZATION
    const uint8_t libMask = g_recog.getGestureSensorMaskByName(gr.name);
    const uint8_t gagMask = mapRecogMaskToGagMask(libMask);
    viz_flash_sensors(gagMask, getGestureColour(gr.name), 100);
  #endif

  // Toggle command: always allowed, even when disabled.
  if (gagIsToggleCommand(cmdRaw)) {
    g_commandsEnabled = !g_commandsEnabled;
    // Log the toggle gesture with the NEW state (":"=enabled, "!"=disabled).
    viz_log_label(lblRaw, g_commandsEnabled);
return;
  }
  // Log the gesture label (even if command output is currently disabled).
  viz_log_label(lblRaw, g_commandsEnabled);

// When disabled, do not emit any HID events.
  if (!g_commandsEnabled) return;

  // Execute BLE HID mouse command if supported / connected.
  #if USE_BLE_HID_MOUSE && defined(MASTER_HAND)
    gagExecuteBleMouseCommand(cmdRaw);
  #endif
}

static inline void feedRecognizerFromSelectedSensor() {
  if (!gyros[selectedSensor].q) return;

  const uint8_t idx = (uint8_t)selectedSensor;
  const uint32_t now = millis();
  gag::Quaternion q(
    (float)gyros[selectedSensor].q->w,
    (float)gyros[selectedSensor].q->x,
    (float)gyros[selectedSensor].q->y,
    (float)gyros[selectedSensor].q->z
  );
  q.normalizeInPlace();

  const gag::Sensor s = mapGagSensorToRecog(idx);
  g_recog.processSample(s, q, now);

  // Wrist-only: also feed accelerometer samples to the recognizer and visualization.
  // NOTE: units are expressed in "g" based on a 16384 LSB/g assumption (MPU6050 default).
  if (idx == 5) { // project wrist index (HG)
    int16_t accRaw[3] = {0,0,0};
    // Extract from the most recent DMP packet.
    if (gyros[selectedSensor].mpu) {
      gyros[selectedSensor].mpu->dmpGetAccel(accRaw, gyros[selectedSensor].fifoBuffer);
    }
    const float invG = 1.0f / 16384.0f;
    gag::AccelData a((float)accRaw[0] * invG, (float)accRaw[1] * invG, (float)accRaw[2] * invG);
    g_recog.processSample(s, gag::RecogData::fromAccel(a), now);

    #ifdef USE_VISUALIZATION
      viz_set_wrist_accel(a.x, a.y, a.z);

      // Optional: magnetometer-based yaw cube ('m') if getMotion9 is available.
      int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
      gyros[selectedSensor].mpu->getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
      // Very basic validity check.
      if (!(mx == 0 && my == 0 && mz == 0)) {
        const float axg = (float)ax * invG;
        const float ayg = (float)ay * invG;
        const float azg = (float)az * invG;

        // Roll/pitch from accel.
        const float roll  = atan2f(ayg, azg);
        const float pitch = atan2f(-axg, sqrtf(ayg*ayg + azg*azg));

        const float cr = cosf(roll),  sr = sinf(roll);
        const float cp = cosf(pitch), sp = sinf(pitch);

        // Tilt-compensate mag.
        const float mxf = (float)mx;
        const float myf = (float)my;
        const float mzf = (float)mz;

        const float mx2 = mxf * cp + mzf * sp;
        const float my2 = mxf * sr * sp + myf * cr - mzf * sr * cp;
        const float heading = atan2f(-my2, mx2);

        VizQuaternion qm;
        qm.w = cosf(0.5f * heading);
        qm.x = 0.0f;
        qm.y = 0.0f;
        qm.z = sinf(0.5f * heading);
        viz_set_wrist_mag_quat(qm);
      }
    #endif
  }
}

static void gag_setup_my_gestures(gag::Recognizer& recog) {
  constexpr float PI_F = 3.14159265358979323846f;
  constexpr float DEG2RAD = PI_F / 180.0f;

  auto quatAxisAngleDeg = [&](float ax, float ay, float az, float deg) -> gag::Quaternion {
    const float rad = deg * DEG2RAD;
    const float half = rad * 0.5f;
    const float s = sinf(half);
    return gag::Quaternion(cosf(half), ax * s, ay * s, az * s);
  };

  // --- Common timing ---
  const float thrClick = 12.0f * DEG2RAD;
  const uint32_t clickDelay = 250;
  const uint32_t clickMaxMs = 1000;

  // ================================
  // Index finger => LEFT CLICK
  // Rotation: same as your existing working "index_up_45"
  // ================================
  {
    gag::GestureDef g;
    strncpy(g.name, "index_left_click", sizeof(g.name) - 1);
    strncpy(g.command, "MOUSE_LEFT_CLICK", sizeof(g.command) - 1);
    strncpy(g.label, "LMBC", sizeof(g.label) - 1);
    g.threshold_rad = thrClick;
    g.recognition_delay_ms = clickDelay;
    g.max_time_ms = clickMaxMs;
    g.active = true;
    g.relative = false;

    auto& sd = g.perSensor[(uint8_t)gag::Sensor::INDEX];
    sd.len = 2;
    sd.q[0] = gag::Quaternion(1, 0, 0, 0);
    sd.q[1] = quatAxisAngleDeg(1, 0, 0, -45.0f); // matches your original (-X) orientation
    recog.addGesture(g);
  }

  // ================================
  // Ring finger => RIGHT CLICK
  // Same rotation as index click
  // ================================
  {
    gag::GestureDef g;
    strncpy(g.name, "ring_right_click", sizeof(g.name) - 1);
    strncpy(g.command, "MOUSE_RIGHT_CLICK", sizeof(g.command) - 1);
    strncpy(g.label, "RMBC", sizeof(g.label) - 1);
    g.threshold_rad = thrClick;
    g.recognition_delay_ms = clickDelay;
    g.max_time_ms = clickMaxMs;
    g.active = true;
    g.relative = false;

    auto& sd = g.perSensor[(uint8_t)gag::Sensor::RING];
    sd.len = 2;
    sd.q[0] = gag::Quaternion(1, 0, 0, 0);
    sd.q[1] = quatAxisAngleDeg(1, 0, 0, -45.0f);
    recog.addGesture(g);
  }

  // ================================
  // Little finger => MIDDLE CLICK
  // Same rotation as index click
  // ================================
  {
    gag::GestureDef g;
    strncpy(g.name, "little_middle_click", sizeof(g.name) - 1);
    strncpy(g.command, "MOUSE_MIDDLE_CLICK", sizeof(g.command) - 1);
    strncpy(g.label, "MMBC", sizeof(g.label) - 1);
    g.threshold_rad = thrClick;
    g.recognition_delay_ms = clickDelay;
    g.max_time_ms = clickMaxMs;
    g.active = true;
    g.relative = false;

    auto& sd = g.perSensor[(uint8_t)gag::Sensor::LITTLE];
    sd.len = 2;
    sd.q[0] = gag::Quaternion(1, 0, 0, 0);
    sd.q[1] = quatAxisAngleDeg(1, 0, 0, -45.0f);
    recog.addGesture(g);
  }

  // ================================
  // Middle finger => SCROLL
  // Down: same rotation as index click
  // Up: opposite rotation
  // ================================
  {
    gag::GestureDef g;
    strncpy(g.name, "middle_scroll_down", sizeof(g.name) - 1);
    strncpy(g.command, "MOUSE_SCROLL_DOWN", sizeof(g.command) - 1);
    strncpy(g.label, "SCDN", sizeof(g.label) - 1);
    g.threshold_rad = thrClick;
    g.recognition_delay_ms = 150; // allow repeated scroll when held
    g.max_time_ms = 800;
    g.active = true;
    g.relative = false;

    auto& sd = g.perSensor[(uint8_t)gag::Sensor::MIDDLE];
    sd.len = 2;
    sd.q[0] = gag::Quaternion(1, 0, 0, 0);
    sd.q[1] = quatAxisAngleDeg(1, 0, 0, -45.0f);
    recog.addGesture(g);
  }

  {
    gag::GestureDef g;
    strncpy(g.name, "middle_scroll_up", sizeof(g.name) - 1);
    strncpy(g.command, "MOUSE_SCROLL_UP", sizeof(g.command) - 1);
    strncpy(g.label, "SCUP", sizeof(g.label) - 1);
    g.threshold_rad = thrClick;
    g.recognition_delay_ms = 150;
    g.max_time_ms = 800;
    g.active = true;
    g.relative = false;

    auto& sd = g.perSensor[(uint8_t)gag::Sensor::MIDDLE];
    sd.len = 2;
    sd.q[0] = gag::Quaternion(1, 0, 0, 0);
    sd.q[1] = quatAxisAngleDeg(1, 0, 0, +45.0f); // opposite of down
    recog.addGesture(g);
  }

  // ================================
  // Thumb => PAGE FORWARD/BACK via mouse side buttons
  // Forward: thumb rotates right (+30° around Y)
  // Back:    thumb rotates left  (-30° around Y)
  //
  // NOTE: You mentioned the thumb sensor is mounted with ~-45° yaw (left).
  //       For easier debugging we provide *two* versions: raw and offset.
  //       For now, ONLY the raw version is active. The offset version is commented out.
  // ================================
  {
    gag::GestureDef g;
    strncpy(g.name, "thumb_page_forward_y30_raw", sizeof(g.name) - 1);
    strncpy(g.command, "MOUSE_FORWARD_CLICK", sizeof(g.command) - 1);
    strncpy(g.label, "PFWD", sizeof(g.label) - 1);
    g.threshold_rad = 15.0f * DEG2RAD;
    g.recognition_delay_ms = 400;
    g.max_time_ms = 1200;
    g.active = true;
    g.relative = false;

    auto& sd = g.perSensor[(uint8_t)gag::Sensor::THUMB];
    sd.len = 2;
    sd.q[0] = gag::Quaternion(1, 0, 0, 0);
    sd.q[1] = quatAxisAngleDeg(0, 1, 0, +30.0f);
    recog.addGesture(g);
  }

  {
    gag::GestureDef g;
    strncpy(g.name, "thumb_page_back_y30_raw", sizeof(g.name) - 1);
    strncpy(g.command, "MOUSE_BACK_CLICK", sizeof(g.command) - 1);
    strncpy(g.label, "PBCK", sizeof(g.label) - 1);
    g.threshold_rad = 15.0f * DEG2RAD;
    g.recognition_delay_ms = 400;
    g.max_time_ms = 1200;
    g.active = true;
    g.relative = false;

    auto& sd = g.perSensor[(uint8_t)gag::Sensor::THUMB];
    sd.len = 2;
    sd.q[0] = gag::Quaternion(1, 0, 0, 0);
    sd.q[1] = quatAxisAngleDeg(0, 1, 0, -30.0f);
    recog.addGesture(g);
  }

  /*
  // Offset version (commented out for now):
  // If thumb neutral is around Y=-45°, then:
  //   forward (+30° relative) => Y=-15°
  //   back    (-30° relative) => Y=-75°
  {
    gag::GestureDef g;
    strncpy(g.name, "thumb_page_forward_y30_offset", sizeof(g.name) - 1);
    strncpy(g.command, "MOUSE_FORWARD_CLICK", sizeof(g.command) - 1);
    strncpy(g.label, "PFWD", sizeof(g.label) - 1);
    g.threshold_rad = 15.0f * DEG2RAD;
    g.recognition_delay_ms = 400;
    g.max_time_ms = 1200;
    g.active = true;
    g.relative = false;

    auto& sd = g.perSensor[(uint8_t)gag::Sensor::THUMB];
    sd.len = 2;
    sd.q[0] = gag::Quaternion(1, 0, 0, 0);
    sd.q[1] = quatAxisAngleDeg(0, 1, 0, -15.0f);
    recog.addGesture(g);
  }

  {
    gag::GestureDef g;
    strncpy(g.name, "thumb_page_back_y30_offset", sizeof(g.name) - 1);
    strncpy(g.command, "MOUSE_BACK_CLICK", sizeof(g.command) - 1);
    strncpy(g.label, "PBCK", sizeof(g.label) - 1);
    g.threshold_rad = 15.0f * DEG2RAD;
    g.recognition_delay_ms = 400;
    g.max_time_ms = 1200;
    g.active = true;
    g.relative = false;

    auto& sd = g.perSensor[(uint8_t)gag::Sensor::THUMB];
    sd.len = 2;
    sd.q[0] = gag::Quaternion(1, 0, 0, 0);
    sd.q[1] = quatAxisAngleDeg(0, 1, 0, -75.0f);
    recog.addGesture(g);
  }
  */

  // ================================
  // Thumb => TOGGLE COMMAND OUTPUT
  // "Thumb click": rotate thumb up by ~30° around X.
  // This is the only gesture that works when output is disabled.
  // ================================
  {
    gag::GestureDef g;
    strncpy(g.name, "thumb_toggle_commands", sizeof(g.name) - 1);
    strncpy(g.command, GAG_TOGGLE_COMMAND, sizeof(g.command) - 1);
    g.threshold_rad = 15.0f * DEG2RAD;
    g.recognition_delay_ms = 800;
    g.max_time_ms = 1500;
    g.active = true;
    g.relative = false;

    auto& sd = g.perSensor[(uint8_t)gag::Sensor::THUMB];
    sd.len = 2;
    sd.q[0] = gag::Quaternion(1, 0, 0, 0);
    sd.q[1] = quatAxisAngleDeg(1, 0, 0, -30.0f); // uses same sign convention as the index gesture
    recog.addGesture(g);
  }
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
#else
        Serial.begin(115200);
    #endif

    #if USE_BLE_HID_MOUSE && defined(MASTER_HAND)
        // Start BLE HID mouse advertising (pair as a regular Bluetooth mouse).
        g_bleMouse.begin();
    #endif

    #ifdef USE_VISUALIZATION
        // MASTER_SERIAL_NAME.println(F("Init vis2"));
        // MASTER_SERIAL_NAME.println(F("Init vis3"));
        // viz_init();
        // if (viz_init == nullptr) Serial.println("viz_init pointer is NULL!?");
        viz_init();
        // Initial state: command output disabled (use thumb-toggle to enable).
        viz_log_label("BOOT", false);
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
        gag_setup_my_gestures(g_recog);
        g_recog.setOnRecognized(onGestureRecognized);
        // Read gesture definitions from the same stream as commands, but only
        // when the next byte is 'G'/'g' (see loop()).
        g_recogLoader.begin(MASTER_SERIAL_NAME, g_recogNullOut);

        // gag_setup_my_gestures(g_recog);
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
static uint32_t last = 0;

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {

    int currentlySellectedSensor = selectedSensor;
  
 static uint32_t last = 0;
    handSwitchPrev = timeNow;
    timePrev = timeNow; // the previous time is stored before the actual time read
    timeNow = millis(); // actual time read
    elapsedTime = (timeNow - timePrev);
 
if (millis() - last > 500) {
  last = millis();
  Serial.printf("heap=%u\n", (unsigned)ESP.getFreeHeap());
}

// #ifdef USE_DISPLAY
//     remainingTimeBudget = ui.update();
// #endif

#ifdef MASTER_HAND
    // --- Gesture serial loader (only consumes input if it starts with 'G'/'g') ---
    bool handledGestureLine = false;
  if (timeNow - last > 500) {
    last = millis();
    Serial.printf("heap=%u\n", (unsigned)ESP.getFreeHeap());
  }
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