/*
*/

#include "definitions.h"
#include <math.h>
#include "gag_display.h"
#ifdef USE_DISPLAY
SSD1306Wire display(0x3c, 18, 19);
OLEDDisplayUi ui ( &display );
int remainingTimeBudget = 0;

int screenW = 128;
int screenH = 64;
int clockCenterX = screenW/2;
int clockCenterY = ((screenH-16)/2)+16;   // top yellow part is 16 px height
int clockRadius = 23;

/*
// utility function for digital clock display: prints leading 0
String twoDigits(int digits){
  if(digits < 10) {
    String i = '0'+String(digits);
    return i;
  }
  else {
    return String(digits);
  }
}

void clockOverlay(OLEDDisplay *display, OLEDDisplayUiState* state) {
}

void analogClockFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  //  ui.disableIndicator();
  // Draw the clock face
  //  display->drawCircle(clockCenterX + x, clockCenterY + y, clockRadius);
  display->drawCircle(clockCenterX + x, clockCenterY + y, 2);
  //
  //hour ticks
  for( int z=0; z < 360;z= z + 30 ){
  //Begin at 0° and stop at 360°
    float angle = z ;
    angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
    int x2 = ( clockCenterX + ( sin(angle) * clockRadius ) );
    int y2 = ( clockCenterY - ( cos(angle) * clockRadius ) );
    int x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 8 ) ) ) );
    int y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 8 ) ) ) );
    display->drawLine( x2 + x , y2 + y , x3 + x , y3 + y);
  }

  // display second hand
  float angle = second() * 6 ;
  angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
  int x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 5 ) ) ) );
  int y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 5 ) ) ) );
  display->drawLine( clockCenterX + x , clockCenterY + y , x3 + x , y3 + y);
  //
  // display minute hand
  angle = minute() * 6 ;
  angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
  x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 4 ) ) ) );
  y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 4 ) ) ) );
  display->drawLine( clockCenterX + x , clockCenterY + y , x3 + x , y3 + y);
  //
  // display hour hand
  angle = hour() * 30 + int( ( minute() / 12 ) * 6 )   ;
  angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
  x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 2 ) ) ) );
  y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 2 ) ) ) );
  display->drawLine( clockCenterX + x , clockCenterY + y , x3 + x , y3 + y);
}

void digitalClockFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  String timenow = String(hour())+":"+twoDigits(minute())+":"+twoDigits(second());
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_24);
  display->drawString(clockCenterX + x , clockCenterY + y, timenow );
}

// This array keeps function pointers to all frames
// frames are the single views that slide in
FrameCallback frames[] = { analogClockFrame, digitalClockFrame };

// how many frames are there?
int frameCount = 2;

// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback overlays[] = { clockOverlay };
int overlaysCount = 1;

void displaySetup(){

	// The ESP is capable of rendering 60fps in 80Mhz mode
	// but that won't give you much time for anything else
	// run it in 160Mhz mode or just set it to 30 fps
  ui.setTargetFPS(30);

	// Customize the active and inactive symbol
  ui.setActiveSymbol(activeSymbol);
  ui.setInactiveSymbol(inactiveSymbol);

  // You can change this to
  // TOP, LEFT, BOTTOM, RIGHT
  ui.setIndicatorPosition(TOP);

  // Defines where the first frame is located in the bar.
  ui.setIndicatorDirection(LEFT_RIGHT);

  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);

  // Add frames
  ui.setFrames(frames, frameCount);

  // Add overlays
  ui.setOverlays(overlays, overlaysCount);
  //#ifdef USE_DISPLAY

  // Initialising the UI will init the display too.
  ui.init();

  //display.flipScreenVertically();
  //display.flipScreenHorizontally();

  unsigned long secsSinceStart = millis();
  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
  const unsigned long seventyYears = 2208988800UL;
  // subtract seventy years:
  unsigned long epoch = secsSinceStart - seventyYears * SECS_PER_HOUR;
  setTime(epoch);

}

*/



#ifdef USE_VISUALIZATION
static void visualizationFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y);
#endif

// If visualization is enabled, include it as the first frame.
#ifdef USE_VISUALIZATION
FrameCallback frames[] = { visualizationFrame, analogClockFrame, digitalClockFrame };
int frameCount = 3;
#else
FrameCallback frames[] = { analogClockFrame, digitalClockFrame };
int frameCount = 2;
#endif

OverlayCallback overlays[] = { clockOverlay };
int overlaysCount = 1;

void displaySetup() {
  ui.setTargetFPS(30);
  ui.setActiveSymbol(activeSymbol);
  ui.setInactiveSymbol(inactiveSymbol);
  ui.setIndicatorPosition(TOP);
  ui.setIndicatorDirection(LEFT_RIGHT);
  ui.setFrameAnimation(SLIDE_LEFT);
  ui.setFrames(frames, frameCount);
  ui.setOverlays(overlays, overlaysCount);
  ui.init();

  unsigned long secsSinceStart = millis();
  const unsigned long seventyYears = 2208988800UL;
  unsigned long epoch = secsSinceStart - seventyYears * SECS_PER_HOUR;
  setTime(epoch);
}


#ifdef USE_VISUALIZATION

// To get quaternions straight from your IMU state:
#include "gag.h"   // brings Gyro, gyros[], and SENSOR_PIN_* indices

// ---------------- Quaternion helpers ----------------
static inline void quatNormalize(float q[4]) {
  float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (n > 0.0f) { q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n; }
}

// r = a * b
static inline void quatMul(const float a[4], const float b[4], float r[4]) {
  r[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  r[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  r[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  r[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

// v_out = q * v * conj(q)  (v treated as [0, vx, vy, vz])
static inline void quatRotateVec(const float q[4], const float v[3], float out[3]) {
  float qv[3] = { q[1], q[2], q[3] };
  float t[3]  = {
    2.0f * (qv[1]*v[2] - qv[2]*v[1]),
    2.0f * (qv[2]*v[0] - qv[0]*v[2]),
    2.0f * (qv[0]*v[1] - qv[1]*v[0])
  };
  out[0] = v[0] + q[0]*t[0] + (qv[1]*t[2] - qv[2]*t[1]);
  out[1] = v[1] + q[0]*t[1] + (qv[2]*t[0] - qv[0]*t[2]);
  out[2] = v[2] + q[0]*t[2] + (qv[0]*t[1] - qv[1]*t[0]);
}

// 2D orthographic projection
static inline void projectOrtho(const float p3[3], int16_t &x2, int16_t &y2,
                                int16_t cx, int16_t cy, float s) {
  x2 = (int16_t)(cx + s * p3[0]);
  y2 = (int16_t)(cy - s * p3[1]);
}

// ---------------- Drawing params ----------------
static const float DEG2RAD = 0.0174532925199432957f;

// Main draw: wrist at origin, 5 palm rays, 5 finger segments
void displayDrawVisualization(const float wristQuat[4], const float fingerQuats[5][4]) {
  // Layout (tweak to taste)
  const int16_t cx = 64, cy = 40;   // screen center
  const float scale = 2.0f;         // world→screen
  const float palmLen   = 18.0f;
  const float fingerLen = 14.0f;
  const float baseFanStartDeg = -40.0f; // thumb side
  const float baseFanStepDeg  = 20.0f;  // spacing between palm rays

  float qW[4] = { wristQuat[0], wristQuat[1], wristQuat[2], wristQuat[3] };
  quatNormalize(qW);

  display.clear();

  for (int i = 0; i < 5; ++i) {
    // Palm fan: lay out in wrist-local XY plane, +X to the right, +Y up
    float baseAngle = (baseFanStartDeg + i * baseFanStepDeg) * DEG2RAD;
    float palmEndLocal[3] = { palmLen * cosf(baseAngle),
                              palmLen * sinf(baseAngle),
                              0.0f };

    // Rotate by wrist orientation to world
    float palmEndWorld[3];
    quatRotateVec(qW, palmEndLocal, palmEndWorld);

    // Finger direction: start as +Y in finger-local, rotate by finger IMU, then by wrist
    float qFraw[4] = { fingerQuats[i][0], fingerQuats[i][1], fingerQuats[i][2], fingerQuats[i][3] };
    quatNormalize(qFraw);
    float qWF[4]; // wrist * finger
    quatMul(qW, qFraw, qWF);

    const float fingerLocalDir[3] = { 0.0f, fingerLen, 0.0f }; // one segment in local +Y
    float fingerDirWorld[3];
    quatRotateVec(qWF, fingerLocalDir, fingerDirWorld);

    // Final world points
    float palmStartWorld[3] = { 0.0f, 0.0f, 0.0f };
    float fingerEndWorld[3] = { palmEndWorld[0] + fingerDirWorld[0],
                                palmEndWorld[1] + fingerDirWorld[1],
                                palmEndWorld[2] + fingerDirWorld[2] };

    // Project to screen
    int16_t x0, y0, x1, y1, x2, y2;
    projectOrtho(palmStartWorld, x0, y0, cx, cy, scale);
    projectOrtho(palmEndWorld,   x1, y1, cx, cy, scale);
    projectOrtho(fingerEndWorld, x2, y2, cx, cy, scale);

    // Draw wrist→palm segment
    display.drawLine(x0, y0, x1, y1);

    // Small pivot circle at palm end
    display.drawCircle(x1, y1, 1);

    // Draw finger segment
    display.drawLine(x1, y1, x2, y2);
  }

  // Wrist pivot mark
  display.drawCircle(cx, cy, 2);

  display.display();
}

// UI frame: grab current quats from gyros[] and draw
void visualizationFrame(OLEDDisplay * /*display*/,
                        OLEDDisplayUiState* /*state*/,
                        int16_t /*x*/, int16_t /*y*/) {
  float wrist[4] = {1,0,0,0};
  float fingers[5][4] = {
    {1,0,0,0},{1,0,0,0},{1,0,0,0},{1,0,0,0},{1,0,0,0}
  };

  // Wrist = SENSOR_PIN_HG
  if (gyros[SENSOR_PIN_HG].q) {
    wrist[0] = gyros[SENSOR_PIN_HG].q->w;
    wrist[1] = gyros[SENSOR_PIN_HG].q->x;
    wrist[2] = gyros[SENSOR_PIN_HG].q->y;
    wrist[3] = gyros[SENSOR_PIN_HG].q->z;
  }

  // Fingers TU..EU are 0..4
  for (int i = 0; i < 5; ++i) {
    if (gyros[i].q) {
      fingers[i][0] = gyros[i].q->w;
      fingers[i][1] = gyros[i].q->x;
      fingers[i][2] = gyros[i].q->y;
      fingers[i][3] = gyros[i].q->z;
    }
  }

  displayDrawVisualization(wrist, fingers);
}

#endif // USE_VISUALIZATION

#endif