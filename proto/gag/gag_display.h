#pragma once

// #include "definitions.h"
#define USE_VISUALIZATION 1 
#ifdef USE_VISUALIZATION

#include <Arduino.h>
#include <Wire.h>
#include "SSD1306Wire.h"  // ThingPulse: https://github.com/ThingPulse/esp8266-oled-ssd1306

// ----- Pins / address (keep consistent with your wiring) -----
#ifndef GAG_OLED_ADDR
#define GAG_OLED_ADDR 0x3C
#endif

#ifndef GAG_OLED_SDA
#define GAG_OLED_SDA 18
#endif

#ifndef GAG_OLED_SCL
#define GAG_OLED_SCL 19
#endif

// Sensor indices should match your existing enum/order: TU, SU, FU, MU, EU, HG
#ifndef GAG_NUM_SENSORS
#define GAG_NUM_SENSORS 6
#endif
#ifndef GAG_WRIST_INDEX
#define GAG_WRIST_INDEX 5  // HG
#endif

// Public API: light-weight and easy to integrate with your existing quaternions.
struct VizQuaternion {
  float w, x, y, z;
};

// Initialize display and visualization state.
// Call once from setup() if USE_VISUALIZATION is defined.
void viz_init();
// void viz_init(void);

// Draw one frame of the skeleton.
// Provide 6 quaternions (order must match your sensors; index 5 = wrist/HG).
// If you don’t have all sensors yet, pass identity (1,0,0,0) for missing ones.
void viz_draw_frame(const VizQuaternion q[GAG_NUM_SENSORS]);

// Optional: tweak look/feel at runtime
void viz_set_deg_spacing(float degrees);   // fan spacing between palm rays (default ~20°)
void viz_use_perspective(bool enable);     // enable simple perspective (default false)

// ---- Gesture highlight ("flash" sensors used by a recognized gesture) ----
//
// When a gesture is recognized, you can request a short highlight window where
// the finger segments (skeleton) and the corresponding sensor cubes are drawn
// in an alternate "colour".
//
// NOTE: SSD1306 is 1-bit, so "colour" is implemented as a distinctive line
// pattern. Each gesture name is deterministically mapped to a unique pattern
// (up to a fixed maximum number of distinct gestures).

// Returns a small "colour" id (pattern index) for a given gesture name.
// The same name always returns the same id for the lifetime of the sketch.
uint8_t getGestureColour(const char* gestureName);

// Request a highlight for the given sensor mask for a short period.
// sensorMask bits correspond to your project sensor order:
//   bit0=TU, bit1=SU, bit2=FU, bit3=MU, bit4=EU, bit5=HG (wrist)
// colourId is typically the value returned from getGestureColour().
// durationMs defaults to 100ms per your requirement.
void viz_flash_sensors(uint8_t sensorMask, uint8_t colourId, uint32_t durationMs = 100);

#endif // USE_VISUALIZATION
