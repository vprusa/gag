#pragma once
/**
 * GAG_Recog - quaternion gesture recognition for ESP32/Arduino
 *
 * This is a C++/Arduino reimplementation of the core matching approach used in gag-web
 * (Java classes: BaseComparator/SensorComparator/HandComparator/GestureRecognizer).
 *
 * Key points:
 *  - Gestures are defined as per-sensor sequences (keyframes) of quaternions.
 *  - Each incoming (sensor, quaternion) sample updates matchers for gestures that use that sensor.
 *  - When all required sensors of a gesture have completed their sequences within max_time_ms,
 *    the gesture is considered recognized.
 *  - After recognition, a per-gesture cooldown (recognition_delay_ms) is applied.
 *
 * Serial output:
 *  - On recognition, library prints a single line:
 *      GAG:RECOG name=<name> cmd=<cmd> start_ms=<start> end_ms=<end> dur_ms=<dur>
 *
 * Optional debug:
 *  - Define GAG_RECOG_DEBUG before including this header to get verbose prints.
 */

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>

#ifndef GAG_RECOG_MAX_GESTURES
#define GAG_RECOG_MAX_GESTURES 16
#endif

#ifndef GAG_RECOG_MAX_NAME_LEN
#define GAG_RECOG_MAX_NAME_LEN 32
#endif

#ifndef GAG_RECOG_MAX_CMD_LEN
#define GAG_RECOG_MAX_CMD_LEN 32
#endif

#ifndef GAG_RECOG_MAX_LABEL_LEN
// Max length for GestureDef.label including null terminator.
// The UI typically uses up to 6 chars (or up to 8 when space allows).
#define GAG_RECOG_MAX_LABEL_LEN 9
#endif

#ifndef GAG_RECOG_MAX_QUATS_PER_SENSOR
// In gag-web, typical per-sensor keyframes are ~3-10. Keep some headroom.
#define GAG_RECOG_MAX_QUATS_PER_SENSOR 16
#endif

#ifndef GAG_RECOG_MAX_MATCHERS_PER_SENSOR
// How many parallel "starts" we keep per sensor (prevents missing gestures when the first keyframe is a rest pose).
#define GAG_RECOG_MAX_MATCHERS_PER_SENSOR 8
#endif

namespace gag {

enum class Sensor : uint8_t {
  WRIST  = 0,
  THUMB  = 1,
  INDEX  = 2,
  MIDDLE = 3,
  RING   = 4,
  LITTLE = 5,
  COUNT  = 6
};

static inline const char* sensorToString(Sensor s) {
  switch (s) {
    case Sensor::WRIST:  return "WRIST";
    case Sensor::THUMB:  return "THUMB";
    case Sensor::INDEX:  return "INDEX";
    case Sensor::MIDDLE: return "MIDDLE";
    case Sensor::RING:   return "RING";
    case Sensor::LITTLE: return "LITTLE";
    default:             return "UNKNOWN";
  }
}

// Bitmask helper: 1<<sensorOrdinal
static inline uint8_t sensorBit(Sensor s) { return static_cast<uint8_t>(1u << static_cast<uint8_t>(s)); }

struct Quaternion {
  float w;
  float x;
  float y;
  float z;

  Quaternion() : w(1), x(0), y(0), z(0) {}
  Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

  inline void normalizeInPlace() {
    const float n2 = w*w + x*x + y*y + z*z;
    if (n2 <= 0.0f) { w = 1; x = y = z = 0; return; }
    const float inv = 1.0f / sqrtf(n2);
    w *= inv; x *= inv; y *= inv; z *= inv;
  }

  inline Quaternion normalized() const {
    Quaternion q(*this);
    q.normalizeInPlace();
    return q;
  }

  inline Quaternion conjugate() const {
    return Quaternion(w, -x, -y, -z);
  }

  /**
   * Hamilton product (unit quaternion multiplication).
   * If both inputs are normalized, the result represents composition of rotations.
   */
  static inline Quaternion mul(const Quaternion& a, const Quaternion& b) {
    return Quaternion(
      a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
      a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
      a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
      a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    );
  }

  /**
   * Inverse for unit quaternion is conjugate.
   * (We normalize inputs across the library, so this is sufficient.)
   */
  inline Quaternion inverseUnit() const { return conjugate(); }

  /**
   * Distance metric copied from gag-web BaseComparator.quatsAbsDist:
   *   dist = acos(2*(q1·q2) - 1)
   * and if NaN => 0.
   *
   * Note: This is not the most common quaternion angular distance formula, but we keep it
   * to preserve compatibility with the Java implementation and existing thresholds.
   */
  static inline float absDistJavaStyle(const Quaternion& aNorm, const Quaternion& bNorm) {
    const float dot = aNorm.w*bNorm.w + aNorm.x*bNorm.x + aNorm.y*bNorm.y + aNorm.z*bNorm.z;
    const float v = 2.0f * dot - 1.0f;
    float d = acosf(v);
    if (isnan(d)) return 0.0f;
    return d;
  }
};

struct SensorGestureData {
  uint8_t len = 0;
  Quaternion q[GAG_RECOG_MAX_QUATS_PER_SENSOR];
};

struct GestureDef {
  char name[GAG_RECOG_MAX_NAME_LEN] = {0};
  char command[GAG_RECOG_MAX_CMD_LEN] = {0};
  char label[GAG_RECOG_MAX_LABEL_LEN] = {0};

  // threshold in radians (0..pi-ish). Must match the distance metric used.
  float threshold_rad = 0.0f;

  // cooldown after recognition (ms)
  uint32_t recognition_delay_ms = 0;

  // maximum time window for multi-sensor completion and per-sensor matcher expiration (ms).
  // If 0, max time is treated as "infinite" (no expiration).
  uint32_t max_time_ms = 0;

  /**
   * If true, comparisons for non-wrist sensors are performed in a "wrist-relative" frame.
   *
   * Intuition:
   *  - Incoming samples are treated relative to the *current* incoming wrist orientation.
   *  - Reference keyframes are treated relative to the gesture's reference wrist keyframes.
   *
   * This makes finger gestures more robust against a constant/global wrist orientation offset.
   *
   * Notes:
   *  - The gesture must provide wrist keyframes (perSensor[WRIST].len > 0) when relative=true.
   *  - Wrist itself is treated as a "reference track" for relative gestures and is NOT required
   *    for completion by default (see requiredSensorMask()).
   */
  bool relative = false;

  bool active = true;

  // Per-sensor sequences. A gesture may use only a subset (len=0 for unused sensors).
  SensorGestureData perSensor[static_cast<uint8_t>(Sensor::COUNT)];

  /**
   * Sensors that are required to complete for a recognition.
   *
   * In "relative" mode, WRIST is treated as a reference track and excluded from the required
   * set by default (because requiring an absolute wrist match would defeat the purpose of
   * wrist-relative normalization).
   */
  inline uint8_t requiredSensorMask() const {
    uint8_t m = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(Sensor::COUNT); ++i) {
      if (perSensor[i].len == 0) continue;
      if (relative && i == static_cast<uint8_t>(Sensor::WRIST)) continue;
      m |= (1u << i);
    }
    return m;
  }

  // All sensors present in this gesture definition (including WRIST, even in relative mode).
  inline uint8_t sensorMask() const {
    uint8_t m = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(Sensor::COUNT); ++i) {
      if (perSensor[i].len > 0) m |= (1u << i);
    }
    return m;
  }

  inline void normalizeAllKeyframes() {
    for (uint8_t i = 0; i < static_cast<uint8_t>(Sensor::COUNT); ++i) {
      for (uint8_t k = 0; k < perSensor[i].len; ++k) {
        perSensor[i].q[k].normalizeInPlace();
      }
    }
  }
};

struct RecognizedGesture {
  const char* name;
  const char* command;
  const char* label;
  uint32_t start_ms;
  uint32_t end_ms;
  uint32_t duration_ms;
};

typedef void (*OnRecognizedCallback)(const RecognizedGesture& g);

class Recognizer {
public:
  Recognizer();

  // Print target for recognition/debug output. Defaults to Serial.
  void begin(Print& out = Serial);

  // Clears all gestures (both static and loaded) and runtime state.
  void clear();

  // Number of gestures currently registered.
  uint8_t count() const;

  // Add a gesture definition (copied into internal storage).
  // Returns true on success; false if storage full or invalid (no sensors).
  bool addGesture(const GestureDef& def);

  // Convenience: load sample gestures described in the prompt.
  void addSampleGestures();

  // Set callback invoked on recognition (in addition to Serial prints).
  void setOnRecognized(OnRecognizedCallback cb);

  // Main entry: feed a new sample (sensor + quaternion).
  void processSample(Sensor sensor, const Quaternion& q, uint32_t now_ms = 0);

  // Print a summary of loaded gestures.
  void printGestures() const;

  // ---- Small helpers for integration / UI ----
  // Returns the gesture index by name, or -1 if not found.
  int8_t findGestureIndexByName(const char* name) const;

  // Returns the sensor bitmask (including WRIST) for the gesture with this name.
  // Bit order matches gag::Sensor enum (0=WRIST,1=THUMB,...,5=LITTLE).
  // Returns 0 if not found.
  uint8_t getGestureSensorMaskByName(const char* name) const;

private:
  struct PartialMatcher {
    bool active = false;
    uint8_t index = 0;      // next ref index to match (1..len-1). 0 unused.
    uint32_t start_ms = 0;  // when started (matched ref[0])
  };

  struct SensorRuntime {
    PartialMatcher m[GAG_RECOG_MAX_MATCHERS_PER_SENSOR];

    bool completed = false;
    uint32_t completed_start_ms = 0;
    uint32_t completed_end_ms = 0;
  };

  struct GestureRuntime {
    uint32_t cooldown_until_ms = 0;
    SensorRuntime srt[static_cast<uint8_t>(Sensor::COUNT)];
  };

  GestureDef _gestures[GAG_RECOG_MAX_GESTURES];
  GestureRuntime _rt[GAG_RECOG_MAX_GESTURES];
  uint8_t _count = 0;

  Print* _out = &Serial;
  OnRecognizedCallback _cb = nullptr;

  // Last observed wrist sample (normalized). Used for wrist-relative matching.
  Quaternion _lastWristNorm;
  bool _haveWrist = false;
  uint32_t _lastWristTimeMs = 0;

  inline bool isCooldownActive(uint8_t gi, uint32_t now) const;
  inline void clearRuntime(uint8_t gi);
  inline void clearAllRuntimes();
  inline bool doesMatch(uint8_t gi, Sensor sensor, uint8_t refIndex, const Quaternion& refNorm, const Quaternion& sampleNorm) const;

  void handleSensorUpdate(uint8_t gi, Sensor sensor, const Quaternion& sampleNorm, uint32_t now);
  void maybeRecognize(uint8_t gi, uint32_t now);
  void printRecognized(const RecognizedGesture& rg);
};

} // namespace gag
