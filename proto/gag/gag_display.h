/*
*/

#ifndef _GAG_DISPLAY_H_
#define _GAG_DISPLAY_H_

#include "definitions.h"
#ifdef USE_DISPLAY

#include "Wire.h"
#include <TimeLib.h>
#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`
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

#endif
#endif
