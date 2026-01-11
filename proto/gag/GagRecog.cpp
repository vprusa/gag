// ======================= GagRecog.cpp (updated) =======================
// Add this whole file content (replace your existing GagRecog.cpp with this one)

#include "GagRecog.h"
#include <string.h>

// -----------------------------------------------------------------------------
// Library debug macros (requested)
// -----------------------------------------------------------------------------

// #define GAG_DEBUG_RECOG_LIB
#ifdef GAG_DEBUG_RECOG_LIB
    #define GAG_DEBUG_RECOG_LIB_PRINT(x) Serial.print(x)
    #define GAG_DEBUG_RECOG_LIB_PRINTF(x, y) Serial.print(x, y)
    #define GAG_DEBUG_RECOG_LIB_PRINTLN(x) Serial.println(x)
    #define GAG_DEBUG_RECOG_LIB_PRINTLNF(x, y) Serial.println(x, y)
    #define GAG_DEBUG_RECOG_LIB_WRITE(x) Serial.write(x)
    #define GAG_DEBUG_RECOG_LIB_WRITE_LEN(x,y) Serial.write(x,y)
#else
    #define GAG_DEBUG_RECOG_LIB_PRINT(x)
    #define GAG_DEBUG_RECOG_LIB_PRINTF(x, y)
    #define GAG_DEBUG_RECOG_LIB_PRINTLN(x)
    #define GAG_DEBUG_RECOG_LIB_PRINTLNF(x, y)
    #define GAG_DEBUG_RECOG_LIB_WRITE(x)
    #define GAG_DEBUG_RECOG_LIB_WRITE_LEN(x, y)
#endif

namespace {

// Small RAII helper that prints "<method>-start" on entry and "<method>-end" on exit.
struct GagRecogLibMethodScope {
  const char* method;

  explicit GagRecogLibMethodScope(const char* m) : method(m) {
    GAG_DEBUG_RECOG_LIB_PRINT(method);
    GAG_DEBUG_RECOG_LIB_PRINTLN("-start");
  }

  ~GagRecogLibMethodScope() {
    GAG_DEBUG_RECOG_LIB_PRINT(method);
    GAG_DEBUG_RECOG_LIB_PRINTLN("-end");
  }
};

}  // namespace

namespace gag {

static inline bool timeReached(uint32_t now, uint32_t target) {
  // Works across millis() wrap-around (uint32_t).
  return (int32_t)(now - target) >= 0;
}

Recognizer::Recognizer() {
  GagRecogLibMethodScope _dbg("Recognizer::Recognizer");
}

void Recognizer::begin(Print& out) {
  GagRecogLibMethodScope _dbg("Recognizer::begin");
  _out = &out;
}

void Recognizer::clear() {
  GagRecogLibMethodScope _dbg("Recognizer::clear");
  _count = 0;
  clearAllRuntimes();
}

uint8_t Recognizer::count() const {
  GagRecogLibMethodScope _dbg("Recognizer::count");
  return _count;
}

int8_t Recognizer::findGestureIndexByName(const char* name) const {
  GagRecogLibMethodScope _dbg("Recognizer::findGestureIndexByName");
  if (!name || name[0] == '\0') return -1;
  for (uint8_t i = 0; i < _count; ++i) {
    // Names are stored as NUL-terminated fixed arrays.
    if (strncmp(name, _gestures[i].name, GAG_RECOG_MAX_NAME_LEN) == 0) {
      return (int8_t)i;
    }
  }
  return -1;
}

uint8_t Recognizer::getGestureSensorMaskByName(const char* name) const {
  GagRecogLibMethodScope _dbg("Recognizer::getGestureSensorMaskByName");
  const int8_t idx = findGestureIndexByName(name);
  if (idx < 0) return 0;
  return _gestures[(uint8_t)idx].sensorMask();
}

bool Recognizer::addGesture(const GestureDef& def) {
  GagRecogLibMethodScope _dbg("Recognizer::addGesture");
  if (_count >= GAG_RECOG_MAX_GESTURES) return false;

  const uint8_t requiredMask = def.requiredSensorMask();
  if (requiredMask == 0) return false;

  // Wrist-relative gestures need a reference wrist track.
  if (def.relative && def.perSensor[static_cast<uint8_t>(Sensor::WRIST)].len == 0) {
    return false;
  }

  // Copy and normalize.
  _gestures[_count] = def;
  _gestures[_count].name[GAG_RECOG_MAX_NAME_LEN - 1] = '\0';
  _gestures[_count].command[GAG_RECOG_MAX_CMD_LEN - 1] = '\0';
  _gestures[_count].normalizeAllKeyframes();

  // Clear runtime for this slot.
  clearRuntime(_count);

#ifdef GAG_RECOG_DEBUG
  if (_out) {
    _out->print(F("[GAG][DBG] addGesture: "));
    _out->print(_gestures[_count].name);
    _out->print(F(" reqMask=0x"));
    _out->print(requiredMask, HEX);
    _out->print(F(" rel="));
    _out->println(_gestures[_count].relative ? 1 : 0);
  }
#endif

  _count++;
  return true;
}

void Recognizer::setOnRecognized(OnRecognizedCallback cb) {
  GagRecogLibMethodScope _dbg("Recognizer::setOnRecognized");
  _cb = cb;
}

inline bool Recognizer::isCooldownActive(uint8_t gi, uint32_t now) const {
  GagRecogLibMethodScope _dbg("Recognizer::isCooldownActive");
  const uint32_t until = _rt[gi].cooldown_until_ms;
  if (until == 0) return false;
  return !timeReached(now, until);
}

inline void Recognizer::clearRuntime(uint8_t gi) {
  GagRecogLibMethodScope _dbg("Recognizer::clearRuntime");
  for (uint8_t si = 0; si < static_cast<uint8_t>(Sensor::COUNT); ++si) {
    SensorRuntime& srt = _rt[gi].srt[si];
    srt.completed = false;
    srt.completed_start_ms = 0;
    srt.completed_end_ms = 0;
    for (uint8_t mi = 0; mi < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++mi) {
      srt.m[mi].active = false;
      srt.m[mi].index = 0;
      srt.m[mi].start_ms = 0;
    }
  }
}

inline void Recognizer::clearAllRuntimes() {
  GagRecogLibMethodScope _dbg("Recognizer::clearAllRuntimes");
  for (uint8_t gi = 0; gi < GAG_RECOG_MAX_GESTURES; ++gi) {
    _rt[gi].cooldown_until_ms = 0;
    clearRuntime(gi);
  }
}

static inline uint8_t mapIndexByProgress(uint8_t idx, uint8_t fromLen, uint8_t toLen) {
  if (toLen == 0) return 0;
  if (toLen == 1) return 0;
  if (fromLen <= 1) return 0;
  if (idx >= fromLen) idx = (uint8_t)(fromLen - 1);

  // Map idx in [0, fromLen-1] to [0, toLen-1] with rounding.
  const uint32_t num = (uint32_t)idx * (uint32_t)(toLen - 1);
  const uint32_t den = (uint32_t)(fromLen - 1);
  return (uint8_t)((num + den / 2u) / den);
}

inline bool Recognizer::doesMatch(uint8_t gi, Sensor sensor, uint8_t refIndex, const Quaternion& refNorm, const Quaternion& sampleNorm) const {
  // GagRecogLibMethodScope _dbg("Recognizer::doesMatch");
  const GestureDef& g = _gestures[gi];

  // Default: absolute comparison.
  if (!g.relative || sensor == Sensor::WRIST) {
    const float dist = Quaternion::absDistJavaStyle(refNorm, sampleNorm);
    return (dist < g.threshold_rad);
  }

  // Wrist-relative comparisons require a current incoming wrist sample.
  if (!_haveWrist) {
    return false;
  }

  const SensorGestureData& wristRef = g.perSensor[static_cast<uint8_t>(Sensor::WRIST)];
  if (wristRef.len == 0) {
    // Should not happen (guarded in addGesture), but fall back to absolute.
    const float dist = Quaternion::absDistJavaStyle(refNorm, sampleNorm);
    return (dist < g.threshold_rad);
  }

  const uint8_t si = static_cast<uint8_t>(sensor);
  const uint8_t sensorLen = g.perSensor[si].len;
  const uint8_t wristIdx = mapIndexByProgress(refIndex, sensorLen, wristRef.len);
  const Quaternion& refWrist = wristRef.q[wristIdx];

  // Compare in wrist coordinate frame:
  //   q_rel = inv(wrist) * q
  const Quaternion sampleRel = Quaternion::mul(_lastWristNorm.inverseUnit(), sampleNorm).normalized();
  const Quaternion refRel = Quaternion::mul(refWrist.inverseUnit(), refNorm).normalized();

  const float dist = Quaternion::absDistJavaStyle(refRel, sampleRel);
  return (dist < g.threshold_rad);
}

void Recognizer::processSample(Sensor sensor, const Quaternion& q, uint32_t now_ms) {
  GagRecogLibMethodScope _dbg("Recognizer::processSample");
  const uint32_t now = (now_ms == 0) ? millis() : now_ms;
  const Quaternion sampleNorm = q.normalized();

  // Keep the latest wrist sample available for wrist-relative matching.
  if (sensor == Sensor::WRIST) {
    _lastWristNorm = sampleNorm;
    _haveWrist = true;
    _lastWristTimeMs = now;
  }

  for (uint8_t gi = 0; gi < _count; ++gi) {
    const GestureDef& g = _gestures[gi];
    if (!g.active) continue;

    const uint8_t si = static_cast<uint8_t>(sensor);
    if (si >= static_cast<uint8_t>(Sensor::COUNT)) continue;
    if (g.perSensor[si].len == 0) continue;

    // In wrist-relative mode we treat wrist keyframes as a reference track, not as a
    // required sensor sequence to match. Skip processing WRIST samples for such gestures.
    if (g.relative && sensor == Sensor::WRIST) {
      continue;
    }

    if (isCooldownActive(gi, now)) {
      continue;
    }

    handleSensorUpdate(gi, sensor, sampleNorm, now);
    maybeRecognize(gi, now);
  }
}

void Recognizer::handleSensorUpdate(uint8_t gi, Sensor sensor, const Quaternion& sampleNorm, uint32_t now) {
  GagRecogLibMethodScope _dbg("Recognizer::handleSensorUpdate");
  GestureDef& g = _gestures[gi];
  const uint8_t si = static_cast<uint8_t>(sensor);
  SensorRuntime& srt = _rt[gi].srt[si];
  const SensorGestureData& sd = g.perSensor[si];
  const uint8_t len = sd.len;
  if (len == 0) return;

  const uint32_t maxTime = g.max_time_ms;

  // Purge old completion.
  if (maxTime > 0 && srt.completed) {
    if ((uint32_t)(now - srt.completed_end_ms) > maxTime) {
#ifdef GAG_RECOG_DEBUG
      if (_out) {
        _out->print(F("[GAG][DBG] purge completion gi="));
        _out->print(gi);
        _out->print(F(" sensor="));
        _out->println(sensorToString(sensor));
      }
#endif
      srt.completed = false;
    }
  }

  // Update existing partial matchers.
  for (uint8_t mi = 0; mi < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++mi) {
    PartialMatcher& pm = srt.m[mi];
    if (!pm.active) continue;

    // Expire.
    if (maxTime > 0 && (uint32_t)(now - pm.start_ms) > maxTime) {
#ifdef GAG_RECOG_DEBUG
      if (_out) {
        _out->print(F("[GAG][DBG] expire matcher gi="));
        _out->print(gi);
        _out->print(F(" sensor="));
        _out->print(sensorToString(sensor));
        _out->print(F(" idx="));
        _out->println(pm.index);
      }
#endif
      pm.active = false;
      continue;
    }

    if (pm.index >= len) {
      pm.active = false;
      continue;
    }

    const Quaternion& refNorm = sd.q[pm.index];
    if (doesMatch(gi, sensor, pm.index, refNorm, sampleNorm)) {
      pm.index++;

#ifdef GAG_RECOG_DEBUG
      if (_out) {
        _out->print(F("[GAG][DBG] match gi="));
        _out->print(gi);
        _out->print(F(" sensor="));
        _out->print(sensorToString(sensor));
        _out->print(F(" -> idx="));
        _out->println(pm.index);
      }
#endif

      if (pm.index >= len) {
        // Completed this sensor sequence.
        srt.completed = true;
        srt.completed_start_ms = pm.start_ms;
        srt.completed_end_ms = now;
        pm.active = false;

#ifdef GAG_RECOG_DEBUG
        if (_out) {
          _out->print(F("[GAG][DBG] sensor completed gi="));
          _out->print(gi);
          _out->print(F(" sensor="));
          _out->print(sensorToString(sensor));
          _out->print(F(" start="));
          _out->print(srt.completed_start_ms);
          _out->print(F(" end="));
          _out->println(srt.completed_end_ms);
        }
#endif
      }
    }
  }

  // Start a new matcher if we match the first keyframe.
  // This mimics gag-web BaseComparator behavior where every match of the first
  // keyframe can create a new potential recognition path.
  const Quaternion& firstRef = sd.q[0];
  if (doesMatch(gi, sensor, 0, firstRef, sampleNorm)) {
    if (len == 1) {
      // Immediate completion.
      srt.completed = true;
      srt.completed_start_ms = now;
      srt.completed_end_ms = now;

#ifdef GAG_RECOG_DEBUG
      if (_out) {
        _out->print(F("[GAG][DBG] sensor immediate-complete gi="));
        _out->print(gi);
        _out->print(F(" sensor="));
        _out->println(sensorToString(sensor));
      }
#endif
      return;
    }

    // Find an empty slot; if none, replace the oldest matcher.
    int chosen = -1;
    for (uint8_t mi = 0; mi < GAG_RECOG_MAX_MATCHERS_PER_SENSOR; ++mi) {
      if (!srt.m[mi].active) { chosen = (int)mi; break; }
    }
    if (chosen < 0) {
      // Replace oldest.
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

#ifdef GAG_RECOG_DEBUG
    if (_out) {
      _out->print(F("[GAG][DBG] start matcher gi="));
      _out->print(gi);
      _out->print(F(" sensor="));
      _out->print(sensorToString(sensor));
      _out->print(F(" slot="));
      _out->println(chosen);
    }
#endif
  }
}

void Recognizer::maybeRecognize(uint8_t gi, uint32_t now) {
  GagRecogLibMethodScope _dbg("Recognizer::maybeRecognize");
  const GestureDef& g = _gestures[gi];

  const uint8_t requiredMask = g.requiredSensorMask();
  if (requiredMask == 0) return;
  if (isCooldownActive(gi, now)) return;

  // Check required sensors.
  bool all = true;
  uint32_t startMin = 0;
  uint32_t endMax = 0;
  bool first = true;

  for (uint8_t si = 0; si < static_cast<uint8_t>(Sensor::COUNT); ++si) {
    if ((requiredMask & (1u << si)) == 0) continue;

    const SensorRuntime& srt = _rt[gi].srt[si];
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

  if (!all || first) return;

  const uint32_t maxTime = g.max_time_ms;
  if (maxTime > 0 && (uint32_t)(endMax - startMin) > maxTime) {
    // Completions are too far apart; drop the ones that are too old relative to the newest.
    for (uint8_t si = 0; si < static_cast<uint8_t>(Sensor::COUNT); ++si) {
      if ((requiredMask & (1u << si)) == 0) continue;
      SensorRuntime& srt = _rt[gi].srt[si];
      if (srt.completed && (uint32_t)(endMax - srt.completed_end_ms) > maxTime) {
        srt.completed = false;
      }
    }
    return;
  }

  // Recognized!
  RecognizedGesture rg;
  rg.name = g.name;
  rg.command = g.command;
  rg.start_ms = startMin;
  rg.end_ms = endMax;
  rg.duration_ms = (uint32_t)(endMax - startMin);

  printRecognized(rg);
  if (_cb) _cb(rg);

  // Apply cooldown and reset runtime state.
  _rt[gi].cooldown_until_ms = endMax + g.recognition_delay_ms;
  clearRuntime(gi);
}

void Recognizer::printRecognized(const RecognizedGesture& rg) {
  GagRecogLibMethodScope _dbg("Recognizer::printRecognized");
  if (!_out) return;
  _out->print(F("GAG:RECOG name="));
  _out->print(rg.name);
  _out->print(F(" cmd="));
  _out->print(rg.command);
  _out->print(F(" command="));
  _out->print(rg.command);
  _out->print(F(" start_ms="));
  _out->print(rg.start_ms);
  _out->print(F(" end_ms="));
  _out->print(rg.end_ms);
  _out->print(F(" dur_ms="));
  _out->println(rg.duration_ms);
}

void Recognizer::printGestures() const {
  GagRecogLibMethodScope _dbg("Recognizer::printGestures");
  if (!_out) return;
  _out->println(F("GAG:GESTURES BEGIN"));
  _out->print(F("count="));
  _out->println(_count);

  for (uint8_t gi = 0; gi < _count; ++gi) {
    const GestureDef& g = _gestures[gi];
    _out->print(F("[")); _out->print(gi); _out->print(F("] "));
    _out->print(g.name);
    _out->print(F(" cmd=")); _out->print(g.command);
    _out->print(F(" command=")); _out->print(g.command);
    _out->print(F(" active=")); _out->print(g.active ? 1 : 0);
    _out->print(F(" rel=")); _out->print(g.relative ? 1 : 0);
    _out->print(F(" thr=")); _out->print(g.threshold_rad, 6);
    _out->print(F(" delay_ms=")); _out->print(g.recognition_delay_ms);
    _out->print(F(" max_ms=")); _out->println(g.max_time_ms);

    for (uint8_t si = 0; si < static_cast<uint8_t>(Sensor::COUNT); ++si) {
      if (g.perSensor[si].len == 0) continue;
      _out->print(F("  - ")); _out->print(sensorToString((Sensor)si));
      _out->print(F(" len=")); _out->println(g.perSensor[si].len);
    }
  }
  _out->println(F("GAG:GESTURES END"));
}

void Recognizer::addSampleGestures() {
  GagRecogLibMethodScope _dbg("Recognizer::addSampleGestures");
  // Sample gesture 1: wrist 0 -> 90deg left -> 0
  {
    GestureDef g;
    strncpy(g.name, "wrist_left_90", sizeof(g.name)-1);
    strncpy(g.command, "MOUSE_LEFT_CLICK", sizeof(g.command)-1);
    g.threshold_rad = 20.0f * (float)M_PI / 180.0f;
    g.recognition_delay_ms = 5000;
    g.max_time_ms = 5000;
    g.active = true;

    auto& sd = g.perSensor[static_cast<uint8_t>(Sensor::WRIST)];
    sd.len = 3;
    sd.q[0] = Quaternion(1, 0, 0, 0);
    // 90deg around Z axis (example)
    sd.q[1] = Quaternion(0.70710678f, 0, 0, 0.70710678f);
    sd.q[2] = Quaternion(1, 0, 0, 0);

    addGesture(g);
  }

  // Sample gesture 2: index 0 -> 45deg -> 0
  {
    GestureDef g;
    strncpy(g.name, "index_bend_45", sizeof(g.name)-1);
    strncpy(g.command, "MOUSE_RIGHT_CLICK", sizeof(g.command)-1);
    g.threshold_rad = 10.0f * (float)M_PI / 180.0f;
    g.recognition_delay_ms = 1000;
    g.max_time_ms = 2000;
    g.active = true;

    auto& sd = g.perSensor[static_cast<uint8_t>(Sensor::INDEX)];
    sd.len = 3;
    sd.q[0] = Quaternion(1, 0, 0, 0);
    // 45deg around X axis (example)
    sd.q[1] = Quaternion(0.92387953f, 0.38268343f, 0, 0);
    sd.q[2] = Quaternion(1, 0, 0, 0);

    addGesture(g);
  }

  // Sample gesture 3: index 0->45->0 AND middle 0->45->0
  {
    GestureDef g;
    strncpy(g.name, "index_middle_bend_45", sizeof(g.name)-1);
    strncpy(g.command, "MOUSE_MIDDLE_CLICK", sizeof(g.command)-1);
    g.threshold_rad = 10.0f * (float)M_PI / 180.0f;
    g.recognition_delay_ms = 1000;
    g.max_time_ms = 2000;
    g.active = true;

    auto& sIndex = g.perSensor[static_cast<uint8_t>(Sensor::INDEX)];
    sIndex.len = 3;
    sIndex.q[0] = Quaternion(1, 0, 0, 0);
    sIndex.q[1] = Quaternion(0.92387953f, 0.38268343f, 0, 0);
    sIndex.q[2] = Quaternion(1, 0, 0, 0);

    auto& sMiddle = g.perSensor[static_cast<uint8_t>(Sensor::MIDDLE)];
    sMiddle.len = 3;
    sMiddle.q[0] = Quaternion(1, 0, 0, 0);
    sMiddle.q[1] = Quaternion(0.92387953f, 0.38268343f, 0, 0);
    sMiddle.q[2] = Quaternion(1, 0, 0, 0);

    addGesture(g);
  }
}

} // namespace gag
