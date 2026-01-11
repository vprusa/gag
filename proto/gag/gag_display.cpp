// #ifdef USE_VISUALIZATION

#include "gag_display.h"
#include <math.h>
#include <string.h>

// ================== Debug controls ==================
// Uncomment to enable serial debug prints
// #define VIZ_DEBUG 1
#ifndef VIZ_DEBUG_INTERVAL_MS
#define VIZ_DEBUG_INTERVAL_MS 200
#endif
// ====================================================

// ---------- OLED driver ----------
static SSD1306Wire display(GAG_OLED_ADDR, GAG_OLED_SDA, GAG_OLED_SCL);

// ---------- Gesture highlight state ----------
static uint8_t  gFlashSensorMask = 0;   // bit0..bit5 match your project sensor order
static uint8_t  gFlashColourId   = 0;   // pattern index ("colour")
static uint32_t gFlashUntilMs    = 0;   // millis() timestamp when highlight expires

// ---------- Command history (for the rotated text area) ----------
static constexpr uint8_t kCmdHistoryLines = 3;
static constexpr size_t  kCmdLineMaxLen   = 40;  // stored; rendering may truncate further
static char    gCmdHistory[kCmdHistoryLines][kCmdLineMaxLen];
static uint8_t gCmdHistoryCount = 0;  // number of valid lines in gCmdHistory
static uint8_t gCmdSeq = 0;           // prefix counter (mod 10)

// ---------- Optional: wrist magnetometer quaternion cube ----------
// Stored in public VizQuaternion form to avoid order-of-declaration issues.
static VizQuaternion gWristMagQuat = {1,0,0,0};

static inline bool timeReached(uint32_t now, uint32_t target) {
  // Works across millis() wrap-around.
  return (int32_t)(now - target) >= 0;
}

// ---------- Tunables ----------
static float kPalmDegSpacing = 20.0f;   // degrees between palm rays
static bool  kUsePerspective  = false;  // off by default (clearer on 1-bit OLED)

// Model lengths in "model units" (auto-scaled to screen)
static const float kPalmLen   = 30.0f;
static const float kFingerLen = 26.0f;

// Projection params (computed on init)
static int   kScreenW = 128;
static int   kScreenH = 64;
static float kScale   = 1.0f;  // pixels per model unit
static float kZ0      = 60.0f; // perspective offset (bigger => weaker perspective)

// Debug timing
#ifdef VIZ_DEBUG
static uint32_t g_lastDebugMs = 0;
#endif

// ---------- Compile-time mounting correction (Euler degrees, Z·Y·X order) ----------
#ifndef GAG_FINGER_MOUNT_CORR_EULER_DEG_X
#define GAG_FINGER_MOUNT_CORR_EULER_DEG_X 0.0f
#endif
#ifndef GAG_FINGER_MOUNT_CORR_EULER_DEG_Y
#define GAG_FINGER_MOUNT_CORR_EULER_DEG_Y 0.0f
#endif
#ifndef GAG_FINGER_MOUNT_CORR_EULER_DEG_Z
#define GAG_FINGER_MOUNT_CORR_EULER_DEG_Z 0.0f // e.g., -90.0f if your finger IMUs are rotated CW around +Z
#endif

#ifndef GAG_WRIST_MOUNT_CORR_EULER_DEG_X
#define GAG_WRIST_MOUNT_CORR_EULER_DEG_X 0.0f
#endif
#ifndef GAG_WRIST_MOUNT_CORR_EULER_DEG_Y
#define GAG_WRIST_MOUNT_CORR_EULER_DEG_Y 0.0f
#endif
#ifndef GAG_WRIST_MOUNT_CORR_EULER_DEG_Z
#define GAG_WRIST_MOUNT_CORR_EULER_DEG_Z 0.0f
#endif

// ---------- Small math helpers ----------
struct V3 { float x, y, z; };
struct Q  { float w, x, y, z; };

static inline float deg2rad(float d) { return d * 0.017453292519943295f; } // pi/180

static inline Q q_mul(const Q& a, const Q& b) {
  // Hamilton product; rotation 'a' applied after 'b' (q_total = a*b)
  return Q{
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

static inline V3 q_rotate(const Q& q, const V3& v) {
  // Optimized quaternion * vector rotation: v' = v + 2*cross(q.xyz, cross(q.xyz, v) + q.w*v)
  float qx = q.x, qy = q.y, qz = q.z, qw = q.w;
  float t0 = 2.0f * (qy * v.z - qz * v.y);
  float t1 = 2.0f * (qz * v.x - qx * v.z);
  float t2 = 2.0f * (qx * v.y - qy * v.x);
  return V3{
    v.x + qw * t0 + (qy * t2 - qz * t1),
    v.y + qw * t1 + (qz * t0 - qx * t2),
    v.z + qw * t2 + (qx * t1 - qy * t0)
  };
}

static inline Q q_from_axis_angle(const V3& axis, float degrees) {
  float half = 0.5f * deg2rad(degrees);
  float s = sinf(half), c = cosf(half);
  float n = sqrtf(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
  if (n < 1e-6f) return Q{1,0,0,0};
  float nx = axis.x / n, ny = axis.y / n, nz = axis.z / n;
  return Q{ c, nx*s, ny*s, nz*s };
}

// Compose Z·Y·X Euler degrees into one quaternion (apply X, then Y, then Z)
static inline Q q_euler_zyx_deg(float z_deg, float y_deg, float x_deg) {
  Q qx = q_from_axis_angle(V3{1,0,0}, x_deg);
  Q qy = q_from_axis_angle(V3{0,1,0}, y_deg);
  Q qz = q_from_axis_angle(V3{0,0,1}, z_deg);
  return q_mul(qz, q_mul(qy, qx));
}

static inline V3 rotZ_apply(float deg, const V3& v) {
  float r = deg2rad(deg);
  float c = cosf(r), s = sinf(r);
  return V3{ c * v.x - s * v.y, s * v.x + c * v.y, v.z };
}

// Helper at top of file (if you don't already have an inverse)
static inline Q q_conj(const Q& q) { return Q{ q.w, -q.x, -q.y, -q.z }; } // unit quaternion inverse

static inline bool project(const V3& p, int& x, int& y) {
  float u, v;
  if (kUsePerspective) {
    float denom = (p.z + kZ0);
    if (denom < 1.0f) return false;  // behind camera; drop
    u = p.x / denom;
    v = p.y / denom;
  } else {
    u = p.x;
    v = p.y;
  }
  x = (int)(kScreenW * 0.5f + kScale * u + 0.5f);
  y = (int)(kScreenH * 0.5f - kScale * v + 0.5f);
  return (x > -10 && x < kScreenW + 10 && y > -10 && y < kScreenH + 10);
}

// NEW: projection with *screen* offset (used to shift the hand left by 30px)
static inline bool project_with_offset(const V3& p, int& x, int& y, int dx, int dy) {
  if (!project(p, x, y)) return false;
  x += dx; y += dy;
  return (x > -10 && x < kScreenW + 10 && y > -10 && y < kScreenH + 10);
}

// ---------- Line drawing helpers ----------
// We implement a tiny Bresenham so we can support "colours" on a 1‑bit OLED
// via patterns (dashed/dotted). Solid lines use pattern 0xFFFF.
static inline void drawLinePattern(int x0, int y0, int x1, int y1, uint16_t pattern, uint8_t phase = 0) {
  // Standard Bresenham (integer), drawing only the pixels that match the pattern.
  int dx = abs(x1 - x0);
  int sx = (x0 < x1) ? 1 : -1;
  int dy = -abs(y1 - y0);
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;

  uint16_t step = 0;
  while (true) {
    const uint8_t bit = (uint8_t)((pattern >> ((step + phase) & 15u)) & 1u);
    if (bit) display.setPixel(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    const int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
    ++step;
  }
}

// "Palette" of distinct patterns. Each entry is a different "colour".
// (We avoid 0x0000 and 0xFFFF as highlight colours because they either draw nothing or look identical to solid.)
static const uint16_t kGesturePatterns[] = {
  0xF0F0, // long dashes
  0xCCCC, // medium dashes
  0xAAAA, // dotted
  0xFF00, // dash-gap
  0x0F0F, // inverse dash-gap
  0x3333, // sparse
  0x5555, // alternating
  0x3C3C, // blocky
  0xE1E1, // asymmetric
  0x87E1, // pseudo-random
  0x1FE0, // head-heavy
  0x07F8, // center-heavy
  0xF99F, // dense ends
  0xD2D2, // pattern
  0xB4B4, // pattern
  0x6DB6  // pattern
};

static inline uint16_t gesturePatternFromColourId(uint8_t colourId) {
  const uint8_t n = (uint8_t)(sizeof(kGesturePatterns) / sizeof(kGesturePatterns[0]));
  return kGesturePatterns[(uint8_t)(colourId % n)];
}

static inline void drawLineStyled(int x0, int y0, int x1, int y1, bool highlight, uint8_t colourId) {
  if (!highlight) {
    // Fast path
    display.drawLine(x0, y0, x1, y1);
    return;
  }
  // Patterned line for highlight.
  drawLinePattern(x0, y0, x1, y1, gesturePatternFromColourId(colourId), 0);
}

// ================== Small rotated text for the command history ==================
// We render a tiny built-in 5x7 font rotated 90° clockwise, so a short command
// string can be drawn vertically in the narrow strip between skeleton and cubes.
//
// Glyph format: 7 rows, 5 bits per row (MSB = leftmost).
struct Glyph5x7 {
  char ch;
  uint8_t row[7];
};

// Only include the characters we expect in command strings and prefixes.
// Unknown characters are rendered as space.
static const Glyph5x7 kFont5x7[] = {
  {' ', {0,0,0,0,0,0,0}},
  {':', {0b00000,0b00100,0b00100,0b00000,0b00100,0b00100,0b00000}},
  {'-', {0b00000,0b00000,0b00000,0b11111,0b00000,0b00000,0b00000}},
  {'_', {0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b11111}},
  {'0', {0b01110,0b10001,0b10011,0b10101,0b11001,0b10001,0b01110}},
  {'1', {0b00100,0b01100,0b00100,0b00100,0b00100,0b00100,0b01110}},
  {'2', {0b01110,0b10001,0b00001,0b00010,0b00100,0b01000,0b11111}},
  {'3', {0b11111,0b00010,0b00100,0b00010,0b00001,0b10001,0b01110}},
  {'4', {0b00010,0b00110,0b01010,0b10010,0b11111,0b00010,0b00010}},
  {'5', {0b11111,0b10000,0b11110,0b00001,0b00001,0b10001,0b01110}},
  {'6', {0b00110,0b01000,0b10000,0b11110,0b10001,0b10001,0b01110}},
  {'7', {0b11111,0b00001,0b00010,0b00100,0b01000,0b01000,0b01000}},
  {'8', {0b01110,0b10001,0b10001,0b01110,0b10001,0b10001,0b01110}},
  {'9', {0b01110,0b10001,0b10001,0b01111,0b00001,0b00010,0b01100}},
  {'A', {0b01110,0b10001,0b10001,0b11111,0b10001,0b10001,0b10001}},
  {'B', {0b11110,0b10001,0b10001,0b11110,0b10001,0b10001,0b11110}},
  {'C', {0b01110,0b10001,0b10000,0b10000,0b10000,0b10001,0b01110}},
  {'D', {0b11110,0b10001,0b10001,0b10001,0b10001,0b10001,0b11110}},
  {'E', {0b11111,0b10000,0b10000,0b11110,0b10000,0b10000,0b11111}},
  {'F', {0b11111,0b10000,0b10000,0b11110,0b10000,0b10000,0b10000}},
  {'G', {0b01110,0b10001,0b10000,0b10111,0b10001,0b10001,0b01110}},
  {'H', {0b10001,0b10001,0b10001,0b11111,0b10001,0b10001,0b10001}},
  {'I', {0b11111,0b00100,0b00100,0b00100,0b00100,0b00100,0b11111}},
  {'J', {0b00111,0b00010,0b00010,0b00010,0b10010,0b10010,0b01100}},
  {'K', {0b10001,0b10010,0b10100,0b11000,0b10100,0b10010,0b10001}},
  {'L', {0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b11111}},
  {'M', {0b10001,0b11011,0b10101,0b10101,0b10001,0b10001,0b10001}},
  {'N', {0b10001,0b11001,0b10101,0b10011,0b10001,0b10001,0b10001}},
  {'O', {0b01110,0b10001,0b10001,0b10001,0b10001,0b10001,0b01110}},
  {'P', {0b11110,0b10001,0b10001,0b11110,0b10000,0b10000,0b10000}},
  {'Q', {0b01110,0b10001,0b10001,0b10001,0b10101,0b10010,0b01101}},
  {'R', {0b11110,0b10001,0b10001,0b11110,0b10100,0b10010,0b10001}},
  {'S', {0b01111,0b10000,0b10000,0b01110,0b00001,0b00001,0b11110}},
  {'T', {0b11111,0b00100,0b00100,0b00100,0b00100,0b00100,0b00100}},
  {'U', {0b10001,0b10001,0b10001,0b10001,0b10001,0b10001,0b01110}},
  {'V', {0b10001,0b10001,0b10001,0b10001,0b10001,0b01010,0b00100}},
  {'W', {0b10001,0b10001,0b10001,0b10101,0b10101,0b10101,0b01010}},
  {'X', {0b10001,0b10001,0b01010,0b00100,0b01010,0b10001,0b10001}},
  {'Y', {0b10001,0b10001,0b01010,0b00100,0b00100,0b00100,0b00100}},
  {'Z', {0b11111,0b00001,0b00010,0b00100,0b01000,0b10000,0b11111}},
};

static inline char toUpperAscii(char c) {
  if (c >= 'a' && c <= 'z') return (char)(c - 'a' + 'A');
  return c;
}

static inline const uint8_t* glyphRowsFor(char ch) {
  ch = toUpperAscii(ch);
  const size_t n = sizeof(kFont5x7) / sizeof(kFont5x7[0]);
  for (size_t i = 0; i < n; ++i) {
    if (kFont5x7[i].ch == ch) return kFont5x7[i].row;
  }
  return kFont5x7[0].row; // space
}

// Draw a single character rotated 90 degrees clockwise.
// Bounding box: width=7px, height=5px.
static inline void drawChar5x7Rot90CW(int x0, int y0, char ch) {
  const uint8_t* rows = glyphRowsFor(ch);
  for (int r = 0; r < 7; ++r) {
    const uint8_t bits = rows[r];
    for (int c = 0; c < 5; ++c) {
      const uint8_t mask = (uint8_t)(1u << (4 - c));
      if (!(bits & mask)) continue;
      // Original (c,r) -> rotated CW: (x',y') = (6 - r, c)
      const int x = x0 + (6 - r);
      const int y = y0 + c;
      if (x >= 0 && x < kScreenW && y >= 0 && y < kScreenH) {
        display.setPixel(x, y);
      }
    }
  }
}

// Draw a string rotated 90 degrees clockwise (text runs top-to-bottom).
static inline void drawString5x7Rot90CW(int x0, int y0, const char* s, uint8_t maxChars) {
  if (!s) return;
  const uint8_t adv = 6; // 5px tall (after rotation) + 1px gap
  for (uint8_t i = 0; i < maxChars && s[i] != '\0'; ++i) {
    drawChar5x7Rot90CW(x0, y0 + (int)i * adv, s[i]);
  }
}

// ---------- Public API: gesture colours + flash trigger ----------
// We keep a tiny mapping so each gesture name gets a stable unique colour id.
// (Up to the palette size. After that, ids wrap.)
static constexpr uint8_t kMaxGestureColourEntries = (uint8_t)(sizeof(kGesturePatterns) / sizeof(kGesturePatterns[0]));
static char gGestureColourNames[kMaxGestureColourEntries][33];
static uint8_t gGestureColourCount = 0;

static inline bool streq_n(const char* a, const char* b, size_t n) {
  if (!a || !b) return false;
  for (size_t i = 0; i < n; ++i) {
    const char ca = a[i];
    const char cb = b[i];
    if (ca != cb) return false;
    if (ca == '\0') return true;
  }
  // If both strings are at least n chars and all matched, treat as equal for our purposes.
  return true;
}

uint8_t getGestureColour(const char* gestureName) {
  if (!gestureName || gestureName[0] == '\0') return 0;

  // Look up existing.
  for (uint8_t i = 0; i < gGestureColourCount; ++i) {
    if (streq_n(gGestureColourNames[i], gestureName, sizeof(gGestureColourNames[i]) - 1)) {
      return i;
    }
  }

  // Assign a new colour id.
  const uint8_t id = (gGestureColourCount < kMaxGestureColourEntries)
    ? gGestureColourCount
    : (uint8_t)(gGestureColourCount % kMaxGestureColourEntries);

  // If we still have free slots, store the name. (If we wrapped, we just return an id without storing.)
  if (gGestureColourCount < kMaxGestureColourEntries) {
    strncpy(gGestureColourNames[id], gestureName, sizeof(gGestureColourNames[id]) - 1);
    gGestureColourNames[id][sizeof(gGestureColourNames[id]) - 1] = '\0';
    gGestureColourCount++;
  }

  return id;
}

void viz_flash_sensors(uint8_t sensorMask, uint8_t colourId, uint32_t durationMs) {
  gFlashSensorMask = sensorMask;
  gFlashColourId = colourId;
  gFlashUntilMs = millis() + durationMs;
}

void viz_set_wrist_mag_quat(const VizQuaternion& q) {
  gWristMagQuat = q;
}

void viz_log_command(const char* commandText) {
  if (!commandText || commandText[0] == '\0') {
    return;
  }

  // Build a prefixed line: "<digit>:<command>" where digit is modulo 10.
  char buf[kCmdLineMaxLen];
  const uint8_t d = (uint8_t)(gCmdSeq % 10);
  gCmdSeq++;
  // Keep it simple; truncation is OK.
  snprintf(buf, sizeof(buf), "%u:%s", (unsigned)d, commandText);

  // Shift history down (index 0 = newest).
  for (int i = (int)kCmdHistoryLines - 1; i > 0; --i) {
    strncpy(gCmdHistory[i], gCmdHistory[i - 1], kCmdLineMaxLen - 1);
    gCmdHistory[i][kCmdLineMaxLen - 1] = '\0';
  }
  strncpy(gCmdHistory[0], buf, kCmdLineMaxLen - 1);
  gCmdHistory[0][kCmdLineMaxLen - 1] = '\0';
  if (gCmdHistoryCount < kCmdHistoryLines) {
    gCmdHistoryCount++;
  }
}

// ---------- Mounting corrections (default identity; set from macros in init) ----------
static Q gFingerMountCorr = Q{1,0,0,0};
static Q gWristMountCorr  = Q{1,0,0,0};
static Q gWristFixCorr = Q{1,0,0,0};
static Q gFingerFixCorr = Q{1,0,0,0};

// ---------- Public API ----------
void viz_init(void) {
  Serial.println("[VIZ] init");
  delay(100);
  display.init();
  // display.flipScreenVertically(); // Uncomment if your panel is flipped

  // Infer screen size from driver (defaults are 128x64 on SSD1306)
  kScreenW = display.getWidth();
  kScreenH = display.getHeight();

  // Compute a conservative scale so palm+finger fits with margins
  float totalLen = (kPalmLen + kFingerLen);
  kScale = (0.8f * (float)min(kScreenW, kScreenH)) / (2.0f * totalLen);
  if (kScale < 0.5f) kScale = 0.5f;  // reasonable lower bound

  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "GAG viz ready");
  display.display();

  // Initialize mounting corrections from compile-time Euler degrees
  gFingerMountCorr = q_euler_zyx_deg(
    GAG_FINGER_MOUNT_CORR_EULER_DEG_Z,
    GAG_FINGER_MOUNT_CORR_EULER_DEG_Y,
    GAG_FINGER_MOUNT_CORR_EULER_DEG_X
  );
  gWristMountCorr  = q_euler_zyx_deg(
    GAG_WRIST_MOUNT_CORR_EULER_DEG_Z,
    GAG_WRIST_MOUNT_CORR_EULER_DEG_Y,
    GAG_WRIST_MOUNT_CORR_EULER_DEG_X
  );

  // gWristFixCorr = q_euler_zyx_deg(/*Z*/ -180.0f, /*Y*/ 0.0f, /*X*/ 0.0f);
  // gWristFixCorr = q_euler_zyx_deg(/*Z*/ -90.0f, /*Y*/ 0.0f, /*X*/ 90.0f);
  // gWristFixCorr = q_euler_zyx_deg(/*Z*/ -90.0f, /*Y*/ 0.0f, /*X*/ 180.0f);
  // gWristFixCorr = q_euler_zyx_deg(/*Z*/ 180.0f, /*Y*/ 180.0f, /*X*/ 180.0f);
  gWristFixCorr = q_euler_zyx_deg(/*Z*/ 180.0f, /*Y*/ 0.0f, /*X*/ 0.0f);
  gFingerFixCorr = q_euler_zyx_deg(/*Z*/ -0.0f, /*Y*/ 90.0f, /*X*/ -90.0f);

  #ifdef VIZ_DEBUG
    Serial.println(F("[VIZ] init]"));
    Serial.print(F("[VIZ] OLED addr=0x")); Serial.println(GAG_OLED_ADDR, HEX);
    Serial.print(F("[VIZ] SDA=")); Serial.print(GAG_OLED_SDA);
    Serial.print(F(" SCL=")); Serial.println(GAG_OLED_SCL);
    Serial.print(F("[VIZ] Screen ")); Serial.print(kScreenW); Serial.print('x'); Serial.println(kScreenH);
    Serial.print(F("[VIZ] Scale ")); Serial.print(kScale, 3);
    Serial.print(F(" PalmLen ")); Serial.print(kPalmLen);
    Serial.print(F(" FingerLen ")); Serial.println(kFingerLen);
    Serial.print(F("[VIZ] Palm spacing ")); Serial.print(kPalmDegSpacing);
    Serial.print(F(" deg | Perspective ")); Serial.println(kUsePerspective ? F("ON") : F("OFF"));
  #endif
}

void viz_set_deg_spacing(float degrees) {
  kPalmDegSpacing = degrees;
  #ifdef VIZ_DEBUG
    Serial.print(F("[VIZ] set spacing ")); Serial.print(degrees); Serial.println(F(" deg"));
  #endif
}

void viz_use_perspective(bool enable) {
  
  kUsePerspective = enable;
  #ifdef VIZ_DEBUG
    Serial.print(F("[VIZ] perspective ")); Serial.println(enable ? F("ON") : F("OFF"));
  #endif
}

static Q gWristRemapCorr = Q{1,0,0,0};

// Expect sensors in order [0..4]=fingers, [5]=wrist(HG)
void viz_draw_frame(const VizQuaternion q_in[GAG_NUM_SENSORS]) {
  // -------- Convert to internal Q and normalize --------
  Q q[GAG_NUM_SENSORS];
  for (int i = 0; i < GAG_NUM_SENSORS; ++i) {
    q[i] = Q{ q_in[i].w, q_in[i].x, q_in[i].y, q_in[i].z };
    float n = q[i].w*q[i].w + q[i].x*q[i].x + q[i].y*q[i].y + q[i].z*q[i].z;
    if (n > 0.00001f) {
      float inv = 1.0f / sqrtf(n);
      q[i].w *= inv; q[i].x *= inv; q[i].y *= inv; q[i].z *= inv;
    } else {
      q[i] = Q{1,0,0,0};
    }
  }

  // -------- Apply mounting corrections (local sensor frame) --------
  // q_adj = q_measured * q_mountCorr
  // Q qCorr[GAG_NUM_SENSORS];
  // for (int i = 0; i < GAG_NUM_SENSORS; ++i) {
  //   if (i == GAG_WRIST_INDEX) {
  //     qCorr[i] = q_mul(q[i], gWristMountCorr);
  //   } else {
  //     qCorr[i] = q_mul(q[i], gFingerMountCorr);
  //   }
  // }
  Q qCorr[GAG_NUM_SENSORS];
  for (int i = 0; i < GAG_NUM_SENSORS; ++i) {
    if (i == GAG_WRIST_INDEX) {
      // Apply: q_adj = q_meas * gWristMountCorr * gWristFixCorr
      // Order matters: rightmost (Rx 180°) happens first, then Rz -90°
      // Q t = q_mul(q[i], gWristMountCorr);

      // Inside viz_draw_frame(), where you build qCorr[] for the wrist:
      // const Q Rx180 = q_euler_zyx_deg(/*Z*/0.0f, /*Y*/0.0f, /*X*/180.0f);
      // // (Ry180 works too; either X or Y flips yaw.)

      // // Compose mounting corr first, then conjugate by Rx180:  q' = Rx180 * q * Rx180^{-1}
      // // For 180°, inverse equals the same rotation up to sign, so using Rx180 twice is fine.
      // Q t = q_mul(q[GAG_WRIST_INDEX], gWristMountCorr);     // local mounting corr
      // Q u = q_mul(Rx180, t);                                // pre-multiply (world side of conjugation)
      // // qCorr[GAG_WRIST_INDEX] = q_mul(u, q_conj(Rx180));     // post-multiply (local side)
      // Q u2 = q_mul(u, q_conj(Rx180));     // post-multiply (local side)
      // // qCorr[GAG_WRIST_INDEX] = q_mul(u, q_conj(Rx180));     // post-multiply (local side)
      // qCorr[i] = q_mul(u2, gWristFixCorr);


      // Inside viz_draw_frame(), where you build qCorr[] for the wrist:
      // const Q Rx180 = q_euler_zyx_deg(/*Z*/0.0f, /*Y*/0.0f, /*X*/180.0f);
      // // (Ry180 works too; either X or Y flips yaw.)

      // // Compose mounting corr first, then conjugate by Rx180:  q' = Rx180 * q * Rx180^{-1}
      // // For 180°, inverse equals the same rotation up to sign, so using Rx180 twice is fine.
      // Q t = q_mul(q[GAG_WRIST_INDEX], gWristMountCorr);     // local mounting corr
      // Q u = q_mul(Rx180, t);                                // pre-multiply (world side of conjugation)
      // qCorr[GAG_WRIST_INDEX] = q_mul(u, q_conj(Rx180));     // post-multiply (local side)
      // gWristMountCorr = Q{ 0.0f, 0.70710678f, 0.70710678f, 0.0f };
      // gWristRemapCorr = q_from_axis_angle(V3{1.0f, 1.0f, 0.0f}, 180.0f);
      // // q_adj = q_meas * (existing wrist mount corr) * (remap corr)  // post-multiply = local frame
      // // if (i == GAG_WRIST_INDEX) {
      //   Q t = q_mul(q[i], gWristMountCorr);
      //   qCorr[i] = q_mul(t, q_mul(gWristRemapCorr, q_conj(Rx180)));   // <-- swap X/Y and invert Z
      // } else {
        // qCorr[i] = q_mul(q[i], gFingerMountCorr);
      // }

      gWristRemapCorr = q_from_axis_angle(V3{0.0f, 0.0f, 1.0f}, 180.0f);

      // qCorr[i] = q_mul(t, q_mul(gWristRemapCorr, q_conj(Rx180)));   // <-- swap X/Y and invert Z
      // const Q Rx180 = q_euler_zyx_deg(/*Z*/180.0f, /*Y*/0.0f, /*X*/0.0f);
      Q t = q_mul(q[i], gWristMountCorr);
      Q u = q_mul(t, gWristRemapCorr);
      qCorr[i] = Q{u.w,u.y,u.x,-u.z};

    } else {
      // gFingerFixCorr
      // const Q Rx180 = q_euler_zyx_deg(/*Z*/0.0f, /*Y*/90.0f, /*X*/0.0f);
      const Q Rx180 = q_euler_zyx_deg(/*Z*/0.0f, /*Y*/0.0f, /*X*/90.0f);
      // (Ry180 works too; either X or Y flips yaw.)

      // Compose mounting corr first, then conjugate by Rx180:  q' = Rx180 * q * Rx180^{-1}
      // For 180°, inverse equals the same rotation up to sign, so using Rx180 twice is fine.
      Q u = q_mul(q[i], Rx180);     // local mounting corr
      // Q u = q_mul(Rx180, t);                                // pre-multiply (world side of conjugation)
      // Q t = q_mul(q[i], gFingerMountCorr);
      // qCorr[i] = q_mul(t, gFingerFixCorr);
      // qCorr[i] = t; // q_mul(t, gFingerFixCorr);
        // gWristRemapCorr = q_from_axis_angle(V3{0.0f, 0.0f, 1.0f}, 180.0f);
      Q t = u;
      qCorr[i] = Q{t.w,t.y,t.x,t.z};

      // Q t = q_mul(q[i], gFingerMountCorr);
      // qCorr[i] = q_mul(t, gFingerFixCorr);
    }
  }

  // -------- Global transform requests --------
  // 1) Move entire hand skeleton 30px left on screen
  const int kHandOffsetXpx = -30;
  const int kHandOffsetYpx = 0;

  // 2) Rotate entire hand skeleton by 180° around X (world-space)
  const Q qWorldFlipX = q_from_axis_angle(V3{1,0,0}, 180.0f);

  // Model: wrist at origin (0,0,0). Base palm & finger ray points along +X.
  const V3 basePalmDir = V3{ 1.0f, 0.0f, 0.0f };
  const V3 baseFingDir = V3{ 1.0f, 0.0f, 0.0f };

  // -------- Gesture highlight window --------
  const uint32_t nowMs = millis();
  const bool flashActive = (gFlashUntilMs != 0) && !timeReached(nowMs, gFlashUntilMs);
  if (!flashActive) {
    gFlashSensorMask = 0;
  }
  const uint8_t flashMask = flashActive ? gFlashSensorMask : 0;
  const uint8_t flashColour = gFlashColourId;

  // Blink state used for "X" markers (inside cubes + on skeleton tips).
  // About ~8 Hz (toggle every 64 ms).
  const bool blinkOn = flashActive && (((nowMs >> 6) & 1u) == 0u);

  display.clear();
  display.setColor(WHITE);

  // Draw a small marker at the wrist origin for reference (with 30px left offset).
  // When the wrist is highlighted by a recognized gesture, blink a larger "X".
  {
    int cx, cy;
    if (project_with_offset(V3{0,0,0}, cx, cy, kHandOffsetXpx, kHandOffsetYpx)) {
      const bool hlWrist = (flashMask & (1u << (uint8_t)GAG_WRIST_INDEX)) != 0;
      if (!hlWrist) {
        display.setPixel(cx, cy);
        if (cx+1 < kScreenW) display.setPixel(cx+1, cy);
        if (cy+1 < kScreenH) display.setPixel(cx, cy+1);
        if (cx-1 >= 0)        display.setPixel(cx-1, cy);
        if (cy-1 >= 0)        display.setPixel(cx, cy-1);
      } else {
        // Gesture marker (0.5x larger than the old one): blink a bigger "X".
        if (blinkOn) {
          const int r = 5;
          drawLineStyled(cx - r, cy - r, cx + r, cy + r, true, flashColour);
          drawLineStyled(cx - r, cy + r, cx + r, cy - r, true, flashColour);
        }
      }
    }
  }

  // -------- Debug throttling --------
  #ifdef VIZ_DEBUG
    bool doLog = false;
    uint32_t now = nowMs;
    if (nowMs - g_lastDebugMs >= VIZ_DEBUG_INTERVAL_MS) {
      g_lastDebugMs = nowMs;
      doLog = true;
      Serial.print(F("[VIZ] frame t=")); Serial.print(now);
      Serial.print(F("ms scale=")); Serial.print(kScale, 3);
      Serial.print(F(" spacing=")); Serial.print(kPalmDegSpacing, 1);
      Serial.print(F(" persp=")); Serial.println(kUsePerspective ? F("1") : F("0"));
    }
  #endif

  // ================= Skeleton (palm + fingers) =================
  // Fan the palm rays around Z by ±2*spacing (five rays total)
  for (int i = 0; i < 5; ++i) {
    float k = (float)(i - 2); // -2..+2
    float deg = k * kPalmDegSpacing;

    // Local palm direction rotated around Z for the fan, then by corrected wrist to world
    V3 palmLocal  = rotZ_apply(deg, basePalmDir);
    V3 palmWorld  = q_rotate(qCorr[GAG_WRIST_INDEX], palmLocal);

    // Apply the global 180° X-rotation to the palm direction
    palmWorld = q_rotate(qWorldFlipX, palmWorld);

    // Palm segment endpoints
    V3 P0 = V3{0,0,0};
    V3 P1 = V3{ palmWorld.x * kPalmLen, palmWorld.y * kPalmLen, palmWorld.z * kPalmLen };

    // Finger orientation: absolute, corrected
    V3 fingWorld = q_rotate(qCorr[i], baseFingDir);

    // (3) Switch the finger X and Z axes
    float tmp = fingWorld.x;
    fingWorld.x = fingWorld.z;
    fingWorld.z = tmp;

    // Apply the same global 180° X-rotation to the finger direction
    fingWorld = q_rotate(qWorldFlipX, fingWorld);

    V3 F1 = V3{
      P1.x + fingWorld.x * kFingerLen,
      P1.y + fingWorld.y * kFingerLen,
      P1.z + fingWorld.z * kFingerLen
    };

    // Project (with 30px-left offset) & draw
    int x0,y0,x1,y1,xf,yf;
    bool okP0 = project_with_offset(P0, x0, y0, kHandOffsetXpx, kHandOffsetYpx);
    bool okP1 = project_with_offset(P1, x1, y1, kHandOffsetXpx, kHandOffsetYpx);
    bool okF1 = project_with_offset(F1, xf, yf, kHandOffsetXpx, kHandOffsetYpx);

    const bool hlFinger = (flashMask & (1u << (uint8_t)i)) != 0;
    if (okP0 && okP1) drawLineStyled(x0, y0, x1, y1, hlFinger, flashColour); // wrist -> knuckle
    if (okP1 && okF1) drawLineStyled(x1, y1, xf, yf, hlFinger, flashColour); // knuckle -> fingertip

    // Blink an "X" marker at the fingertip when this sensor is highlighted.
    if (hlFinger && blinkOn && okF1) {
      const int r = 4;
      drawLineStyled(xf - r, yf - r, xf + r, yf + r, true, flashColour);
      drawLineStyled(xf - r, yf + r, xf + r, yf - r, true, flashColour);
    }

    #ifdef VIZ_DEBUG
      if (doLog) {
        Serial.print(F("  [ray ")); Serial.print(i); Serial.print(F("] deg=")); Serial.print(deg, 1);
        Serial.print(F(" P1=(")); Serial.print(P1.x, 2); Serial.print(',');
        Serial.print(P1.y, 2); Serial.print(','); Serial.print(P1.z, 2); Serial.print(')');
        Serial.print(F(" F1=(")); Serial.print(F1.x, 2); Serial.print(',');
        Serial.print(F1.y, 2); Serial.print(','); Serial.print(F1.z, 2); Serial.print(')');
        Serial.print(F(" | scr P0=(")); Serial.print(x0); Serial.print(','); Serial.print(y0); Serial.print(')');
        Serial.print(F(" P1=(")); Serial.print(okP1 ? x1 : -1); Serial.print(',');
        Serial.print(okP1 ? y1 : -1); Serial.print(')');
        Serial.print(F(" F1=(")); Serial.print(okF1 ? xf : -1); Serial.print(',');
        Serial.print(okF1 ? yf : -1); Serial.println(')');
      }
    #endif
  }

  // ================= Rotated command history text area =================
  // This is the space to the LEFT of the cubes visualization.
  const int rightHalfLeft = (kScreenW / 2) + 1;

  // ================= 3D wire cubes for each sensor =================
  // Layout region: a compact grid on the right, arranged 3 rows × 2 columns.
  {
    const int cols = 2, rows = 3;

    // Reserve a small bottom-right square for the magnetometer cube.
    const int magBoxPx = 16;
    const int gridTopPx = 1;
    const int gridBottomPx = max(gridTopPx, kScreenH - magBoxPx - 2); // 1px gap above mag cube
    const int gridH = max(0, gridBottomPx - gridTopPx + 1);
    const int cellH = max(1, gridH / rows);

    // Choose a fixed-ish grid width so we have space left of the grid for text.
    // 3 rotated command lines need ~21px (3*7px) and we keep ~1px gap.
    const int minTextW = 22;
    const int gridRightMargin = 1;
    const int maxGridW = max(0, kScreenW - gridRightMargin - rightHalfLeft - minTextW);
    const int gridW = min(40, maxGridW); // 2 columns => 20px per cell by default

    const int gridLeftPx = max(rightHalfLeft, kScreenW - gridRightMargin - gridW);
    const int cellW = max(1, gridW / cols);

    // ---- Draw the rotated command history in the left strip ----
    {
      const int textW = max(0, gridLeftPx - rightHalfLeft);
      const int lineW = 7; // rotated glyph width
      const int maxLinesFit = (lineW > 0) ? (textW / lineW) : 0;
      const int linesToDraw = min((int)kCmdHistoryLines, maxLinesFit);
      const uint8_t maxChars = (uint8_t)min(12, (kScreenH / 6)); // safety

      // We render newest on the RIGHT-most column, older to the left.
      for (int li = 0; li < linesToDraw; ++li) {
        const int histIdx = li; // gCmdHistory[0] is newest
        if (histIdx >= (int)gCmdHistoryCount) break;
        const int x = (gridLeftPx - (linesToDraw - li) * lineW);
        drawString5x7Rot90CW(x, 0, gCmdHistory[histIdx], maxChars);
      }
    }

    // ---- Draw the 6 sensor cubes (with labels + blinking X markers) ----
    if (gridW > 6 && gridH > 6) {
      // Cube half-size in pixels:
      //   base: ~0.4 * min(cellW, cellH)
      //   old scaling: 2/3
      //   requested: 3/4 of current size => (2/3)*(3/4) = 1/2
      const int   minCell    = min(cellW, cellH);
      const float baseHalfPx = (float)max(3, (minCell * 8) / 20); // 0.4 * minCell
      const float halfPx     = baseHalfPx * 0.5f;                // 1/2 of base

      // For screen → world mapping at a fixed z
      const float cxScr  = (float)kScreenW * 0.5f;
      const float cyScr  = (float)kScreenH * 0.5f;
      const float zWorld = 0.0f;
      const float denom  = kUsePerspective ? (zWorld + kZ0) : 1.0f;

      // 12 cube edges (indices into 8-vertex list)
      static const uint8_t edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0},
        {4,5},{5,6},{6,7},{7,4},
        {0,4},{1,5},{2,6},{3,7}
      };

      auto sensorLabel = [](int s) -> char {
        switch (s) {
          case 0: return 'T'; // thumb
          case 1: return 'I'; // index
          case 2: return 'M'; // middle
          case 3: return 'R'; // ring
          case 4: return 'L'; // little
          case 5: return 'W'; // wrist
          default: return '?';
        }
      };

      // Labels use the library font (single char).
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_10);

      for (int s = 0; s < GAG_NUM_SENSORS; ++s) {
        const int r = s / cols; // 0..2
        const int c = s % cols; // 0..1
        const int cxPx = gridLeftPx + c * cellW + cellW / 2;
        const int cyPx = gridTopPx  + r * cellH + cellH / 2;

        // Label (top-left of the cell)
        {
          const int lx = gridLeftPx + c * cellW + 1;
          const int ly = gridTopPx  + r * cellH;
          char tmp[2] = { sensorLabel(s), 0 };
          display.drawString(lx, ly, String(tmp));
        }

        // Convert cell-center screen position to world at zWorld
        const float u = ((float)cxPx - cxScr) / kScale;
        const float v = (cyScr - (float)cyPx) / kScale;
        const V3 center = V3{ u * denom, v * denom, zWorld };

        // Convert half-size in pixels to world units (respect perspective)
        const float h = (halfPx / kScale) * denom;

        // Local cube vertices centered at origin
        V3 vLocal[8] = {
          V3{-h,-h,-h}, V3{ h,-h,-h}, V3{ h, h,-h}, V3{-h, h,-h},
          V3{-h,-h, h}, V3{ h,-h, h}, V3{ h, h, h}, V3{-h, h, h}
        };

        // Rotate by corrected sensor orientation, then translate to world center
        V3 vWorld[8];
        for (int vi = 0; vi < 8; ++vi) {
          V3 rloc = q_rotate(qCorr[s], vLocal[vi]);
          vWorld[vi] = V3{ center.x + rloc.x, center.y + rloc.y, center.z + rloc.z };
        }

        // Project and draw edges
        const bool hlCube = (flashMask & (1u << (uint8_t)s)) != 0;
        for (int e = 0; e < 12; ++e) {
          int a = edges[e][0], b = edges[e][1];
          int x0,y0,x1,y1;
          bool okA = project(vWorld[a], x0, y0);
          bool okB = project(vWorld[b], x1, y1);
          if (okA && okB) drawLineStyled(x0, y0, x1, y1, hlCube, flashColour);
        }

        // Blink an "X" inside the cube when highlighted.
        if (hlCube && blinkOn) {
          const int rpx = (int)max(3.0f, halfPx * 0.9f);
          drawLineStyled(cxPx - rpx, cyPx - rpx, cxPx + rpx, cyPx + rpx, true, flashColour);
          drawLineStyled(cxPx - rpx, cyPx + rpx, cxPx + rpx, cyPx - rpx, true, flashColour);
        }
      }
    }

    // ---- Magnetometer cube (bottom-right), label 'w' ----
    {
      // Normalize the stored mag quaternion and apply the same wrist correction used for the wrist cube.
      Q qm = Q{ gWristMagQuat.w, gWristMagQuat.x, gWristMagQuat.y, gWristMagQuat.z };
      float n = qm.w*qm.w + qm.x*qm.x + qm.y*qm.y + qm.z*qm.z;
      if (n > 0.00001f) {
        float inv = 1.0f / sqrtf(n);
        qm.w *= inv; qm.x *= inv; qm.y *= inv; qm.z *= inv;
      } else {
        qm = Q{1,0,0,0};
      }

      // Reuse the wrist correction convention from the wrist sensor (index 5).
      const Q wristRemap = q_from_axis_angle(V3{0.0f, 0.0f, 1.0f}, 180.0f);
      Q t = q_mul(qm, gWristMountCorr);
      Q u = q_mul(t, wristRemap);
      Q qMagCorr = Q{u.w, u.y, u.x, -u.z};

      const int cxPx = kScreenW - (magBoxPx / 2) - 1;
      const int cyPx = kScreenH - (magBoxPx / 2) - 1;

      // Label
      {
        const int lx = kScreenW - magBoxPx + 1;
        const int ly = kScreenH - magBoxPx;
        display.drawString(lx, ly, String("w"));
      }

      const float cxScr  = (float)kScreenW * 0.5f;
      const float cyScr  = (float)kScreenH * 0.5f;
      const float zWorld = 0.0f;
      const float denom  = kUsePerspective ? (zWorld + kZ0) : 1.0f;

      const float u0 = ((float)cxPx - cxScr) / kScale;
      const float v0 = (cyScr - (float)cyPx) / kScale;
      const V3 center = V3{ u0 * denom, v0 * denom, zWorld };

      const int minCell = magBoxPx;
      const float baseHalfPx = (float)max(3, (minCell * 8) / 20);
      const float halfPx = baseHalfPx * 0.5f;
      const float h = (halfPx / kScale) * denom;

      V3 vLocal[8] = {
        V3{-h,-h,-h}, V3{ h,-h,-h}, V3{ h, h,-h}, V3{-h, h,-h},
        V3{-h,-h, h}, V3{ h,-h, h}, V3{ h, h, h}, V3{-h, h, h}
      };

      static const uint8_t edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0},
        {4,5},{5,6},{6,7},{7,4},
        {0,4},{1,5},{2,6},{3,7}
      };

      V3 vWorld[8];
      for (int vi = 0; vi < 8; ++vi) {
        V3 rloc = q_rotate(qMagCorr, vLocal[vi]);
        vWorld[vi] = V3{ center.x + rloc.x, center.y + rloc.y, center.z + rloc.z };
      }

      for (int e = 0; e < 12; ++e) {
        int a = edges[e][0], b = edges[e][1];
        int x0,y0,x1,y1;
        bool okA = project(vWorld[a], x0, y0);
        bool okB = project(vWorld[b], x1, y1);
        if (okA && okB) {
          // Mag cube is never "highlighted" by sensor masks; keep it solid.
          display.drawLine(x0, y0, x1, y1);
        }
      }
    }
  }

  display.display();
}

// #endif // USE_VISUALIZATION
