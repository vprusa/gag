#include "GagRecogSerialLoader.h"
#include <string.h>
#include <strings.h>
#include <stdlib.h>

namespace gag {

static inline void trimInPlace(char* s) {
  if (!s) return;
  // leading
  size_t start = 0;
  while (s[start] == ' ' || s[start] == '\t' || s[start] == '\r' || s[start] == '\n') start++;
  if (start > 0) memmove(s, s + start, strlen(s + start) + 1);

  // trailing
  size_t n = strlen(s);
  while (n > 0 && (s[n-1] == ' ' || s[n-1] == '\t' || s[n-1] == '\r' || s[n-1] == '\n')) {
    s[n-1] = '\0';
    n--;
  }
}

SerialLoader::SerialLoader(Recognizer& r) : _r(r) {}

void SerialLoader::begin(Stream& in, Print& out) {
  _in = &in;
  _out = &out;
  _pos = 0;
  memset(_line, 0, sizeof(_line));
  _b = Builder();
}

void SerialLoader::poll() {
  if (!_in) return;

  while (_in->available() > 0) {
    const int c = _in->read();
    if (c < 0) return;

    if (c == '\n' || c == '\r') {
      if (_pos == 0) continue;
      _line[_pos] = '\0';
      _pos = 0;
      trimInPlace(_line);
      if (_line[0] == '\0') continue;
      processLine(_line);
      continue;
    }

    if (_pos + 1 < LINE_BUF) {
      _line[_pos++] = (char)c;
    } else {
      // overflow => reset line
      _pos = 0;
      if (_out) _out->println(F("GAG:ERR line too long"));
    }
  }
}

bool SerialLoader::parseSensor(const char* tok, Sensor& s) const {
  if (!tok) return false;

  // numeric
  if (tok[0] >= '0' && tok[0] <= '9') {
    const int v = atoi(tok);
    if (v >= 0 && v < (int)Sensor::COUNT) { s = (Sensor)v; return true; }
    return false;
  }

  // uppercase compare
  if (!strcasecmp(tok, "WRIST"))  { s = Sensor::WRIST; return true; }
  if (!strcasecmp(tok, "THUMB"))  { s = Sensor::THUMB; return true; }
  if (!strcasecmp(tok, "INDEX"))  { s = Sensor::INDEX; return true; }
  if (!strcasecmp(tok, "MIDDLE")) { s = Sensor::MIDDLE; return true; }
  if (!strcasecmp(tok, "RING"))   { s = Sensor::RING; return true; }
  if (!strcasecmp(tok, "LITTLE")) { s = Sensor::LITTLE; return true; }

  return false;
}

void SerialLoader::printHelp() const {
  if (!_out) return;
  _out->println(F("GAG:HELP"));
  _out->println(F("  GAG HELP"));
  _out->println(F("  GAG LIST"));
  _out->println(F("  GAG CLEAR"));
  _out->println(F("  GAG SAMPLE"));
  _out->println(F("  GAG BEGIN <name> <command> <threshold_rad> <delay_ms> <max_time_ms> <active0|1> [relative0|1]"));
  _out->println(F("  GAG SENSOR <WRIST|THUMB|INDEX|MIDDLE|RING|LITTLE|0..5> <count>"));
  _out->println(F("  GAG Q <w> <x> <y> <z>   (repeat <count> times)"));
  _out->println(F("  GAG END"));
  _out->println(F("  GAG ABORT"));
}

bool SerialLoader::processLine(char* line) {
  // Comments
  if (!line || line[0] == '#') return true;

  // Tokenize in-place
  char* tok = strtok(line, " \t");
  if (!tok) return false;
  if (strcasecmp(tok, "GAG") != 0) return false;

  char* cmd = strtok(nullptr, " \t");
  if (!cmd) return false;

  if (!strcasecmp(cmd, "HELP")) {
    printHelp();
    return true;
  }

  if (!strcasecmp(cmd, "LIST")) {
    _r.printGestures();
    return true;
  }

  if (!strcasecmp(cmd, "CLEAR")) {
    _r.clear();
    if (_out) _out->println(F("GAG:OK cleared"));
    return true;
  }

  if (!strcasecmp(cmd, "SAMPLE")) {
    _r.addSampleGestures();
    if (_out) _out->println(F("GAG:OK sample added"));
    return true;
  }

  if (!strcasecmp(cmd, "ABORT")) {
    _b = Builder();
    if (_out) _out->println(F("GAG:OK aborted"));
    return true;
  }

  if (!strcasecmp(cmd, "BEGIN")) {
    const char* name = strtok(nullptr, " \t");
    const char* gcmd = strtok(nullptr, " \t");
    const char* thr = strtok(nullptr, " \t");
    const char* dly = strtok(nullptr, " \t");
    const char* mx  = strtok(nullptr, " \t");
    const char* act = strtok(nullptr, " \t");
    const char* rel = strtok(nullptr, " \t");

    if (!name || !gcmd || !thr || !dly || !mx || !act) {
      if (_out) _out->println(F("GAG:ERR BEGIN expects 6 args (+ optional relative flag)"));
      return true;
    }

    _b = Builder();
    _b.active = true;
    strncpy(_b.g.name, name, sizeof(_b.g.name)-1);
    strncpy(_b.g.command, gcmd, sizeof(_b.g.command)-1);
    _b.g.threshold_rad = (float)atof(thr);
    _b.g.recognition_delay_ms = (uint32_t)strtoul(dly, nullptr, 10);
    _b.g.max_time_ms = (uint32_t)strtoul(mx, nullptr, 10);
    _b.g.active = (atoi(act) != 0);
    _b.g.relative = (rel != nullptr && atoi(rel) != 0);

    if (_out) {
      _out->print(F("GAG:OK begin name="));
      _out->println(_b.g.name);
    }
    return true;
  }

  if (!_b.active) {
    if (_out) _out->println(F("GAG:ERR no active BEGIN (use GAG BEGIN ...)"));
    return true;
  }

  if (!strcasecmp(cmd, "SENSOR")) {
    const char* st = strtok(nullptr, " \t");
    const char* cnt = strtok(nullptr, " \t");
    if (!st || !cnt) {
      if (_out) _out->println(F("GAG:ERR SENSOR expects 2 args"));
      return true;
    }

    Sensor s;
    if (!parseSensor(st, s)) {
      if (_out) _out->println(F("GAG:ERR unknown sensor"));
      return true;
    }

    const int n = atoi(cnt);
    if (n <= 0 || n > (int)GAG_RECOG_MAX_QUATS_PER_SENSOR) {
      if (_out) _out->println(F("GAG:ERR invalid count"));
      return true;
    }

    _b.currentSensor = s;
    _b.expected = (uint8_t)n;
    _b.received = 0;
    _b.haveSensor = true;

    // Clear any old data for this sensor.
    _b.g.perSensor[(uint8_t)s].len = 0;

    if (_out) {
      _out->print(F("GAG:OK sensor "));
      _out->print(sensorToString(s));
      _out->print(F(" count="));
      _out->println(_b.expected);
    }
    return true;
  }

  if (!strcasecmp(cmd, "Q") || !strcasecmp(cmd, "QUAT")) {
    if (!_b.haveSensor) {
      if (_out) _out->println(F("GAG:ERR no SENSOR selected"));
      return true;
    }
    if (_b.received >= _b.expected) {
      if (_out) _out->println(F("GAG:ERR too many Q for this SENSOR"));
      return true;
    }

    const char* sw = strtok(nullptr, " \t");
    const char* sx = strtok(nullptr, " \t");
    const char* sy = strtok(nullptr, " \t");
    const char* sz = strtok(nullptr, " \t");
    if (!sw || !sx || !sy || !sz) {
      if (_out) _out->println(F("GAG:ERR Q expects 4 floats"));
      return true;
    }

    Quaternion q((float)atof(sw), (float)atof(sx), (float)atof(sy), (float)atof(sz));
    SensorGestureData& sd = _b.g.perSensor[(uint8_t)_b.currentSensor];
    sd.q[_b.received] = q;
    _b.received++;
    sd.len = _b.received;

    if (_out) {
      _out->print(F("GAG:OK q "));
      _out->print(_b.received);
      _out->print(F("/"));
      _out->println(_b.expected);
    }

    return true;
  }

  if (!strcasecmp(cmd, "END")) {
    if (_b.g.sensorMask() == 0) {
      if (_out) _out->println(F("GAG:ERR gesture has no sensors"));
      _b = Builder();
      return true;
    }

    const bool ok = _r.addGesture(_b.g);
    if (_out) {
      if (ok) {
        _out->print(F("GAG:OK added "));
        _out->println(_b.g.name);
      } else {
        _out->println(F("GAG:ERR addGesture failed"));
      }
    }
    _b = Builder();
    return true;
  }

  if (_out) _out->println(F("GAG:ERR unknown command (use GAG HELP)"));
  return true;
}

} // namespace gag
