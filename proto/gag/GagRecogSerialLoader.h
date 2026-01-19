#pragma once
#include "GagRecog.h"

namespace gag {

/**
 * Simple Serial protocol for loading gestures at runtime (ASCII, line-oriented).
 *
 * All commands start with "GAG".
 *
 * Commands:
 *   GAG HELP
 *   GAG LIST
 *   GAG CLEAR
 *   GAG SAMPLE            (loads the 3 sample gestures)
 *
 * Add gesture:
 *   GAG BEGIN <name> <cmd> [label] <threshold_rad> <delay_ms> <max_time_ms> <active0|1> [relative0|1] [threshold_accel]
 *   GAG SENSOR <WRIST|THUMB|INDEX|MIDDLE|RING|LITTLE|0..5> <count>
 *   GAG Q <w> <x> <y> <z>        (repeat <count> times)
 *   GAG A <x> <y> <z>            (repeat <count> times)
 *   ... (more sensors) ...
 *   GAG END
 *
 * Abort:
 *   GAG ABORT
 *
 * Notes:
 *  - Quaternions are normalized on add.
 *  - Per-sensor quat count must be <= GAG_RECOG_MAX_QUATS_PER_SENSOR.
 */
class SerialLoader {
public:
  explicit SerialLoader(Recognizer& r);

  void begin(Stream& in = Serial, Print& out = Serial);
  void poll();

private:
  Recognizer& _r;
  Stream* _in = &Serial;
  Print* _out = &Serial;

  static constexpr size_t LINE_BUF = 200;
  char _line[LINE_BUF];
  size_t _pos = 0;

  struct Builder {
    bool active = false;
    GestureDef g;
    bool haveSensor = false;

    Sensor currentSensor = Sensor::WRIST;
    enum class Track : uint8_t { NONE = 0, QUAT = 1, ACCEL = 2 };
    Track currentTrack = Track::NONE;

    uint8_t expected = 0;
    uint8_t received = 0;
    uint8_t baseLen = 0; // existing length when the current SENSOR block started
  } _b;

  bool processLine(char* line);
  bool parseSensor(const char* tok, Sensor& s) const;

  void printHelp() const;
};

} // namespace gag
