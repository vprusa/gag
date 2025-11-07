/*
*/

#ifndef _GAG_DISPLAY_H_
#define _GAG_DISPLAY_H_

#include "definitions.h"
#ifdef USE_DISPLAY

#include "Wire.h"
// #include <TimeLib.h>
// #include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`
// #include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`
// #include <SSD1306.h>
#include <SSD1306Wire.h>

#include "OLEDDisplayUi.h"

const unsigned char activeSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00011000,
    B00100100,
    B01000010,
    B01000010,
    B00100100,
    B00011000
};

const unsigned char inactiveSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000
};

void displaySetup();

// --- Visualization API ---
#ifdef USE_VISUALIZATION
// Draws a simple wireframe hand skeleton on the SSD1306.
// Quaternions are in [w,x,y,z]. fingerQuats has 5 entries in order TU..EU.
// Call this directly OR let the UI call visualizationFrame for you.
void displayDrawVisualization(const float wristQuat[4],
                              const float fingerQuats[5][4]);

// Optional: an OLEDDisplayUi frame you can register with the UI.
void visualizationFrame(OLEDDisplay *display,
                        OLEDDisplayUiState* state,
                        int16_t x, int16_t y);
#endif


#endif
#endif
