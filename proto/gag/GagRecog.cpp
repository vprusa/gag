// ======================= GagRecog.cpp =======================
// Quaternion + accelerometer gesture recognition for ESP32/Arduino.

#include "GagRecog.h"
#include <string.h>

// -----------------------------------------------------------------------------
// Optional library debug macros
// -----------------------------------------------------------------------------
// #define GAG_DEBUG_RECOG_LIB
#ifdef GAG_DEBUG_RECOG_LIB
  #define GAG_DEBUG_RECOG_LIB_PRINT(x) Serial.print(x)
  #define GAG_DEBUG_RECOG_LIB_PRINTLN(x) Serial.println(x)
#else
  #define GAG_DEBUG_RECOG_LIB_PRINT(x)
  #define GAG_DEBUG_RECOG_LIB_PRINTLN(x)
#endif

namespace {

static inline bool timeReached(uint32_t now, uint32_t target) {
  // Works across millis() wrap-around (uint32_t).
  return (int32_t)(now - target) >= 0;
}

// Map a progress index from one track length to another (used for relative wrist track sampling).
static inline uint8_t mapIndexByProgress(uint8_t idx, uint8_t fromLen, uint8_t toLen) {
  if (toLen == 0 || toLen == 1) return 0;
  if (fromLen <= 1) return 0;
  if (idx >= fromLen) idx = (uint8_t)(fromLen - 1);
  const uint32_t num = (uint32_t)idx * (uint32_t)(toLen - 1);
  const uint32_t den = (uint32_t)(fromLen - 1);
  return (uint8_t)((num + den / 2u) / den);
}

static inline float accelDist(const gag::AccelData& a, const gag::AccelData& b) {
  const float dx = a.x - b.x;
  const float dy = a.y - b.y;
  const float dz = a.z - b.z;
  return sqrtf(dx*dx + dy*dy + dz*dz);
}

} // namespace

namespace gag {

Recognizer::Recognizer() {
  GAG_DEBUG_RECOG_LIB_PRINTLN("Recognizer::Recognizer");
}

void Recognizer::begin(Print& out) {
  _out = &out;
}

void Recognizer::clear() {
  _count = 0;
  clearAllRuntimes();
}

uint8_t Recognizer::count() const {
  return _count;
}

int8_t Recognizer::findGestureIndexByName(const char* name) const {
  if (!name || name[0] == '\0') return -1;
  for (uint8_t i = 0; i < _count; ++i) {
    if (strncmp(name, _gestures[i].name, GAG_RECOG_MAX_NAME_LEN) == 0) return (int8_t)i;
  }
  return -1;
}

uint8_t Recognizer::getGestureSensorMaskByName(const char* name) const {
  const int8_t idx = findGestureIndexByName(name);
  if (idx < 0) return 0;
  return _gestures[(uint8_t)idx].sensorMask();
}

bool Recognizer::addGesture(const GestureDef& def) {
  if (_count >= GAG_RECOG_MAX_GESTURES) return false;

  const uint8_t requiredMask = def.requiredSensorMask();
  if (requiredMask == 0) return false;

  // Wrist-relative gestures need a reference wrist quaternion track.
  if (def.relative && def.perSensor[static_cast<uint8_t>(Sensor::WRIST)].len == 0) {
    return false;
  }

  _gestures[_count] = def;
  _gestures[_count].name[GAG_RECOG_MAX_NAME_LEN - 1] = '\0';
  _gestures[_count].command[GAG_RECOG_MAX_CMD_LEN - 1] = '\0';
  _gestures[_count].label[GAG_RECOG_MAX_LABEL_LEN - 1] = '\0';

  // If no label is provided, derive a compact one from command or name (best-effort).
  if (_gestures[_count].label[0] == '\0') {
    const char* src = (_gestures[_count].command[0] != '\0') ? _gestures[_count].command : _gestures[_count].name;
    strncpy(_gestures[_count].label, src, GAG_RECOG_MAX_LABEL_LEN - 1);
    _gestures[_count].label[GAG_RECOG_MAX_LABEL_LEN - 1] = '\0';
  }

  _gestures[_count].normalizeAllKeyframes();
  clearRuntime(_count);

  _count++;
  return true;
}

void Recognizer::addSampleGestures() {
  // Sample gesture 1: wrist 0 -> 90deg left -> 0
  {
    GestureDef g;
    strncpy(g.name, "wrist_left_90", sizeof(g.name)-1);
    strncpy(g.command, "CMD_WRIST_LEFT", sizeof(g.command)-1);
    strncpy(g.label, "W90L", sizeof(g.label)-1);
    g.threshold_rad = 20.0f * (float)M_PI / 180.0f;
    g.recognition_delay_ms = 5000;
    g.max_time_ms = 5000;
    g.active = true;

    auto& sd = g.perSensor[static_cast<uint8_t>(Sensor::WRIST)];
    sd.len = 3;
    sd.q[0] = Quaternion(1, 0, 0, 0);
    sd.q[1] = Quaternion(0.70710678f, 0, 0, 0.70710678f);
    sd.q[2] = Quaternion(1, 0, 0, 0);

    addGesture(g);
  }

  // Sample gesture 2: index bend 45deg (identity -> x45 -> identity)
  {
    GestureDef g;
    strncpy(g.name, "index_bend_45", sizeof(g.name)-1);
    strncpy(g.command, "CMD_INDEX_BEND", sizeof(g.command)-1);
    strncpy(g.label, "I45", sizeof(g.label)-1);
    g.threshold_rad = 15.0f * (float)M_PI / 180.0f;
    g.recognition_delay_ms = 500;
    g.max_time_ms = 2000;
    g.active = true;

    auto& sd = g.perSensor[static_cast<uint8_t>(Sensor::INDEX)];
    sd.len = 3;
    sd.q[0] = Quaternion(1,0,0,0);
    sd.q[1] = Quaternion(0.92387953f, 0.38268343f, 0, 0);
    sd.q[2] = Quaternion(1,0,0,0);

    addGesture(g);
  }

  // Sample gesture 3: index + middle bend 45deg (identity -> x45 -> identity)
  {
    GestureDef g;
    strncpy(g.name, "index_middle_bend_45", sizeof(g.name)-1);
    strncpy(g.command, "CMD_INDEX_MIDDLE", sizeof(g.command)-1);
    strncpy(g.label, "IM45", sizeof(g.label)-1);
    g.threshold_rad = 15.0f * (float)M_PI / 180.0f;
    g.recognition_delay_ms = 500;
    g.max_time_ms = 2000;
    g.active = true;

    auto& i = g.perSensor[static_cast<uint8_t>(Sensor::INDEX)];
    i.len = 3;
    i.q[0] = Quaternion(1,0,0,0);
    i.q[1] = Quaternion(0.92387953f, 0.38268343f, 0, 0);
    i.q[2] = Quaternion(1,0,0,0);

    auto& m = g.perSensor[static_cast<uint8_t>(Sensor::MIDDLE)];
    m.len = 3;
    m.q[0] = Quaternion(1,0,0,0);
    m.q[1] = Quaternion(0.92387953f, 0.38268343f, 0, 0);
    m.q[2] = Quaternion(1,0,0,0);

    addGesture(g);
  }

  // Sample gesture 4: wrist accel "z bump" (example). Useful for quick smoke-tests.
  {
    GestureDef g;
    strncpy(g.name, "wrist_accel_z_bump", sizeof(g.name)-1);
    strncpy(g.command, "CMD_WRIST_ACCEL", sizeof(g.command)-1);
    strncpy(g.label, "AZ", sizeof(g.label)-1);
    g.threshold_rad = 0.0f; // not used
    g.threshold_accel = 0.25f;
    g.recognition_delay_ms = 500;
    g.max_time_ms = 1200;
    g.active = false; // disabled by default (example only)

    auto& a = g.perSensorAccel[static_cast<uint8_t>(Sensor::WRIST)];
    a.len = 2;
    a.a[0] = AccelData(0,0,1.0f);
    a.a[1] = AccelData(0,0,1.7f);
    addGesture(g);
  }
}

void Recognizer::setOnRecognized(OnRecognizedCallback cb) {
  _cb = cb;
}

inline bool Recognizer::isCooldownActive(uint8_t gi, uint32_t now) const {
  const uint32_t until = _rt[gi].cooldown_until_ms;
  if (until == 0) return false;
  return !timeReached(now, until);
}

inline void Recognizer::clearRuntime(uint8_t gi) {
  for (uint8_t si = 0; si < static_cast<uint8_t>(Sensor::COUNT); ++si) {
    SensorRuntime& qrt = _rt[gi].qrt[si];
    qrt.completed = false;
    qrt.completed_start_ms = 0;
    qrt.completed_end_ms = 0;
    for (uint8_t mi = 0; mi < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++mi) {
      qrt.m[mi].active = false;
      qrt.m[mi].index = 0;
      qrt.m[mi].start_ms = 0;
    }

    SensorRuntime& art = _rt[gi].art[si];
    art.completed = false;
    art.completed_start_ms = 0;
    art.completed_end_ms = 0;
    for (uint8_t mi = 0; mi < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++mi) {
      art.m[mi].active = false;
      art.m[mi].index = 0;
      art.m[mi].start_ms = 0;
    }
  }
}

inline void Recognizer::clearAllRuntimes() {
  for (uint8_t gi = 0; gi < GAG_RECOG_MAX_GESTURES; ++gi) {
    _rt[gi].cooldown_until_ms = 0;
    clearRuntime(gi);
  }
}

inline bool Recognizer::doesMatchQuat(uint8_t gi, Sensor sensor, uint8_t refIndex,
                                     const Quaternion& refNorm, const Quaternion& sampleNorm) const {
  const GestureDef& g = _gestures[gi];

  // Default: absolute comparison.
  if (!g.relative || sensor == Sensor::WRIST) {
    const float dist = Quaternion::absDistJavaStyle(refNorm, sampleNorm);
    return (dist < g.threshold_rad);
  }

  // Wrist-relative comparisons require a current incoming wrist sample.
  if (!_haveWrist) return false;

  const SensorGestureData& wristRef = g.perSensor[static_cast<uint8_t>(Sensor::WRIST)];
  if (wristRef.len == 0) {
    const float dist = Quaternion::absDistJavaStyle(refNorm, sampleNorm);
    return (dist < g.threshold_rad);
  }

  const uint8_t si = static_cast<uint8_t>(sensor);
  const uint8_t sensorLen = g.perSensor[si].len;
  const uint8_t wristIdx = mapIndexByProgress(refIndex, sensorLen, wristRef.len);
  const Quaternion& refWrist = wristRef.q[wristIdx];

  // Compare in wrist coordinate frame: q_rel = inv(wrist) * q
  const Quaternion sampleRel = Quaternion::mul(_lastWristNorm.inverseUnit(), sampleNorm).normalized();
  const Quaternion refRel = Quaternion::mul(refWrist.inverseUnit(), refNorm).normalized();

  const float dist = Quaternion::absDistJavaStyle(refRel, sampleRel);
  return (dist < g.threshold_rad);
}

inline bool Recognizer::doesMatchAccel(uint8_t gi, Sensor /*sensor*/, uint8_t /*refIndex*/,
                                      const AccelData& ref, const AccelData& sample) const {
  const GestureDef& g = _gestures[gi];
  const float d = accelDist(ref, sample);
  return (d < g.threshold_accel);
}

void Recognizer::processSample(Sensor sensor, const Quaternion& q, uint32_t now_ms) {
  const RecogData d = RecogData::fromQuat(q);
  processSample(sensor, d, now_ms);
}

void Recognizer::processSample(Sensor sensor, const RecogData& data, uint32_t now_ms) {
  const uint32_t now = (now_ms == 0) ? millis() : now_ms;
  const uint8_t si = static_cast<uint8_t>(sensor);
  if (si >= static_cast<uint8_t>(Sensor::COUNT)) return;

  if (data.isQuat()) {
    const Quaternion sampleNorm = data.q->normalized();

    // Keep the latest wrist quaternion sample available for wrist-relative matching.
    if (sensor == Sensor::WRIST) {
      _lastWristNorm = sampleNorm;
      _haveWrist = true;
      _lastWristTimeMs = now;
    }

    for (uint8_t gi = 0; gi < _count; ++gi) {
      const GestureDef& g = _gestures[gi];
      if (!g.active) continue;
      if (g.perSensor[si].len == 0) continue;

      // In wrist-relative mode, wrist quaternion keyframes are a reference track, not a required track.
      // Skip processing WRIST quaternion samples for such gestures (it would create a redundant matcher).
      if (g.relative && sensor == Sensor::WRIST) continue;

      if (isCooldownActive(gi, now)) continue;
      handleQuatUpdate(gi, sensor, sampleNorm, now);
      maybeRecognize(gi, now);
    }
    return;
  }

  if (data.isAccel()) {
    const AccelData sample = *data.a;
    for (uint8_t gi = 0; gi < _count; ++gi) {
      const GestureDef& g = _gestures[gi];
      if (!g.active) continue;
      if (g.perSensorAccel[si].len == 0) continue;
      if (isCooldownActive(gi, now)) continue;
      handleAccelUpdate(gi, sensor, sample, now);
      maybeRecognize(gi, now);
    }
  }
}

void Recognizer::handleQuatUpdate(uint8_t gi, Sensor sensor, const Quaternion& sampleNorm, uint32_t now) {
  GestureDef& g = _gestures[gi];
  const uint8_t si = static_cast<uint8_t>(sensor);
  SensorRuntime& srt = _rt[gi].qrt[si];
  const SensorGestureData& sd = g.perSensor[si];
  const uint8_t len = sd.len;
  if (len == 0) return;

  const uint32_t maxTime = g.max_time_ms;

  // Purge old completion.
  if (maxTime > 0 && srt.completed) {
    if ((uint32_t)(now - srt.completed_end_ms) > maxTime) {
      srt.completed = false;
    }
  }

  // Update existing partial matchers.
  for (uint8_t mi = 0; mi < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++mi) {
    PartialMatcher& pm = srt.m[mi];
    if (!pm.active) continue;

    // Expire.
    if (maxTime > 0 && (uint32_t)(now - pm.start_ms) > maxTime) {
      pm.active = false;
      continue;
    }

    if (pm.index >= len) {
      pm.active = false;
      continue;
    }

    const Quaternion& refNorm = sd.q[pm.index];
    if (doesMatchQuat(gi, sensor, pm.index, refNorm, sampleNorm)) {
      pm.index++;
      if (pm.index >= len) {
        srt.completed = true;
        srt.completed_start_ms = pm.start_ms;
        srt.completed_end_ms = now;
        pm.active = false;
      }
    }
  }

  // Start a new matcher if we match the first keyframe.
  const Quaternion& firstRef = sd.q[0];
  if (doesMatchQuat(gi, sensor, 0, firstRef, sampleNorm)) {
    if (len == 1) {
      srt.completed = true;
      srt.completed_start_ms = now;
      srt.completed_end_ms = now;
      return;
    }

    int chosen = -1;
    for (uint8_t mi = 0; mi < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++mi) {
      if (!srt.m[mi].active) { chosen = (int)mi; break; }
    }
    if (chosen < 0) {
      uint32_t oldest = srt.m[0].start_ms;
      chosen = 0;
      for (uint8_t mi = 1; mi < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++mi) {
        if (srt.m[mi].start_ms < oldest) { oldest = srt.m[mi].start_ms; chosen = (int)mi; }
      }
    }

    PartialMatcher& pm = srt.m[chosen];
    pm.active = true;
    pm.index = 1;
    pm.start_ms = now;
  }
}

void Recognizer::handleAccelUpdate(uint8_t gi, Sensor sensor, const AccelData& sample, uint32_t now) {
  GestureDef& g = _gestures[gi];
  const uint8_t si = static_cast<uint8_t>(sensor);
  SensorRuntime& srt = _rt[gi].art[si];
  const SensorAccelGestureData& sd = g.perSensorAccel[si];
  const uint8_t len = sd.len;
  if (len == 0) return;

  const uint32_t maxTime = g.max_time_ms;

  // Purge old completion.
  if (maxTime > 0 && srt.completed) {
    if ((uint32_t)(now - srt.completed_end_ms) > maxTime) {
      srt.completed = false;
    }
  }

  for (uint8_t mi = 0; mi < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++mi) {
    PartialMatcher& pm = srt.m[mi];
    if (!pm.active) continue;

    if (maxTime > 0 && (uint32_t)(now - pm.start_ms) > maxTime) {
      pm.active = false;
      continue;
    }

    if (pm.index >= len) {
      pm.active = false;
      continue;
    }

    const AccelData& ref = sd.a[pm.index];
    if (doesMatchAccel(gi, sensor, pm.index, ref, sample)) {
      pm.index++;
      if (pm.index >= len) {
        srt.completed = true;
        srt.completed_start_ms = pm.start_ms;
        srt.completed_end_ms = now;
        pm.active = false;
      }
    }
  }

  const AccelData& firstRef = sd.a[0];
  if (doesMatchAccel(gi, sensor, 0, firstRef, sample)) {
    if (len == 1) {
      srt.completed = true;
      srt.completed_start_ms = now;
      srt.completed_end_ms = now;
      return;
    }

    int chosen = -1;
    for (uint8_t mi = 0; mi < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++mi) {
      if (!srt.m[mi].active) { chosen = (int)mi; break; }
    }
    if (chosen < 0) {
      uint32_t oldest = srt.m[0].start_ms;
      chosen = 0;
      for (uint8_t mi = 1; mi < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++mi) {
        if (srt.m[mi].start_ms < oldest) { oldest = srt.m[mi].start_ms; chosen = (int)mi; }
      }
    }

    PartialMatcher& pm = srt.m[chosen];
    pm.active = true;
    pm.index = 1;
    pm.start_ms = now;
  }
}

void Recognizer::maybeRecognize(uint8_t gi, uint32_t now) {
  const GestureDef& g = _gestures[gi];
  const uint16_t required = g.requiredTrackMask();
  if (required == 0) return;
  if (isCooldownActive(gi, now)) return;

  bool all = true;
  uint32_t startMin = 0;
  uint32_t endMax = 0;
  bool first = true;

  const uint8_t N = static_cast<uint8_t>(Sensor::COUNT);
  for (uint8_t si = 0; si < N; ++si) {
    // Quaternion track
    if (required & (uint16_t)(1u << si)) {
      const SensorRuntime& srt = _rt[gi].qrt[si];
      if (!srt.completed) { all = false; break; }
      if (first) {
        startMin = srt.completed_start_ms;
        endMax = srt.completed_end_ms;
        first = false;
      } else {
        if (srt.completed_start_ms < startMin) startMin = srt.completed_start_ms;
        if (srt.completed_end_ms > endMax) endMax = srt.completed_end_ms;
      }
    }
    // Accel track
    if (required & (uint16_t)(1u << (si + N))) {
      const SensorRuntime& srt = _rt[gi].art[si];
      if (!srt.completed) { all = false; break; }
      if (first) {
        startMin = srt.completed_start_ms;
        endMax = srt.completed_end_ms;
        first = false;
      } else {
        if (srt.completed_start_ms < startMin) startMin = srt.completed_start_ms;
        if (srt.completed_end_ms > endMax) endMax = srt.completed_end_ms;
      }
    }
  }

  if (!all || first) return;

  const uint32_t maxTime = g.max_time_ms;
  if (maxTime > 0 && (uint32_t)(endMax - startMin) > maxTime) {
    // Completions are too far apart; drop the ones that are too old relative to the newest.
    for (uint8_t si = 0; si < N; ++si) {
      if (required & (uint16_t)(1u << si)) {
        SensorRuntime& srt = _rt[gi].qrt[si];
        if (srt.completed && (uint32_t)(endMax - srt.completed_end_ms) > maxTime) srt.completed = false;
      }
      if (required & (uint16_t)(1u << (si + N))) {
        SensorRuntime& srt = _rt[gi].art[si];
        if (srt.completed && (uint32_t)(endMax - srt.completed_end_ms) > maxTime) srt.completed = false;
      }
    }
    return;
  }

  RecognizedGesture rg;
  rg.name = g.name;
  rg.command = g.command;
  rg.label = g.label;
  rg.start_ms = startMin;
  rg.end_ms = endMax;
  rg.duration_ms = (uint32_t)(endMax - startMin);

  printRecognized(rg);
  if (_cb) _cb(rg);

  _rt[gi].cooldown_until_ms = endMax + g.recognition_delay_ms;
  clearRuntime(gi);
}

void Recognizer::printRecognized(const RecognizedGesture& rg) {
  if (!_out) return;
  _out->print(F("GAG:RECOG name="));
  _out->print(rg.name);
  _out->print(F(" cmd="));
  _out->print(rg.command);
  _out->print(F(" label="));
  _out->print(rg.label ? rg.label : "");
  _out->print(F(" start_ms="));
  _out->print(rg.start_ms);
  _out->print(F(" end_ms="));
  _out->print(rg.end_ms);
  _out->print(F(" dur_ms="));
  _out->println(rg.duration_ms);
}

void Recognizer::printGestures() const {
  if (!_out) return;
  _out->println(F("GAG:GESTURES BEGIN"));
  _out->print(F("count="));
  _out->println(_count);

  for (uint8_t gi = 0; gi < _count; ++gi) {
    const GestureDef& g = _gestures[gi];
    _out->print(F("[")); _out->print(gi); _out->print(F("] "));
    _out->print(g.name);
    _out->print(F(" cmd=")); _out->print(g.command);
    _out->print(F(" label=")); _out->print(g.label);
    _out->print(F(" active=")); _out->print(g.active ? 1 : 0);
    _out->print(F(" rel=")); _out->print(g.relative ? 1 : 0);
    _out->print(F(" thr_q=")); _out->print(g.threshold_rad, 6);
    _out->print(F(" thr_a=")); _out->print(g.threshold_accel, 6);
    _out->print(F(" delay_ms=")); _out->print(g.recognition_delay_ms);
    _out->print(F(" max_ms=")); _out->println(g.max_time_ms);

    for (uint8_t si = 0; si < static_cast<uint8_t>(Sensor::COUNT); ++si) {
      if (g.perSensor[si].len > 0) {
        _out->print(F("  - Q ")); _out->print(sensorToString((Sensor)si));
        _out->print(F(" len=")); _out->println(g.perSensor[si].len);
      }
      if (g.perSensorAccel[si].len > 0) {
        _out->print(F("  - A ")); _out->print(sensorToString((Sensor)si));
        _out->print(F(" len=")); _out->println(g.perSensorAccel[si].len);
      }
    }
  }
  _out->println(F("GAG:GESTURES END"));
}

} // namespace gag
