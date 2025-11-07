// #include "definitions.h"
#define USE_VISUALIZATION 1 
#ifdef USE_VISUALIZATION

#include "gag_display.h"
#include <math.h>

// ---------- Display ----------
static SSD1306Wire display(GAG_OLED_ADDR, GAG_OLED_SDA, GAG_OLED_SCL);

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

// ---------- Small math helpers ----------
struct V3 { float x, y, z; };
struct Q  { float w, x, y, z; };

static inline Q q_conj(const Q& q) { return Q{ q.w, -q.x, -q.y, -q.z }; }

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

static inline float deg2rad(float d) { return d * 0.017453292519943295f; } // pi/180

static inline V3 rotZ_apply(float deg, const V3& v) {
  float r = deg2rad(deg);
  float c = cosf(r), s = sinf(r);
  return V3{ c * v.x - s * v.y, s * v.x + c * v.y, v.z };
}

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
  // Cull outside a generous boundary to avoid draw overflows
  return (x > -10 && x < kScreenW + 10 && y > -10 && y < kScreenH + 10);
}

static inline void drawLineSafe(int x0, int y0, int x1, int y1) {
  // Thin lines can be hard to see on fast motion; we could double-stroke if needed.
  display.drawLine(x0, y0, x1, y1);
}

// ---------- Public API ----------
void viz_init() {
  display.init();
  // Optional: flip if your panel is mounted upside-down
  // display.flipScreenVertically();

  // Infer screen size from driver (defaults are 128x64 on SSD1306)
  kScreenW = display.getWidth();
  kScreenH = display.getHeight();

  // Compute a conservative scale so palm+finger fits with margins
  float totalLen = (kPalmLen + kFingerLen);
  // Leave some margin; fit to min dimension
  kScale = (0.8f * (float)min(kScreenW, kScreenH)) / (2.0f * totalLen);
  if (kScale < 0.5f) kScale = 0.5f;  // reasonable lower bound

  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "GAG viz ready");
  display.display();
}

void viz_set_deg_spacing(float degrees) { kPalmDegSpacing = degrees; }
void viz_use_perspective(bool enable)   { kUsePerspective  = enable; }

// Expect sensors in order [0..4]=fingers, [5]=wrist(HG)
void viz_draw_frame(const VizQuaternion q_in[GAG_NUM_SENSORS]) {
  // Convert to internal Q
  Q q[GAG_NUM_SENSORS];
  for (int i = 0; i < GAG_NUM_SENSORS; ++i) {
    q[i] = Q{ q_in[i].w, q_in[i].x, q_in[i].y, q_in[i].z };
    // Basic normalization guard (IMU outputs are usually close already)
    float n = q[i].w*q[i].w + q[i].x*q[i].x + q[i].y*q[i].y + q[i].z*q[i].z;
    if (n > 0.00001f) {
      float inv = 1.0f / sqrtf(n);
      q[i].w *= inv; q[i].x *= inv; q[i].y *= inv; q[i].z *= inv;
    } else {
      q[i] = Q{1,0,0,0};
    }
  }

  const Q& qWrist = q[GAG_WRIST_INDEX];

  // Model: wrist at origin (0,0,0). Base palm ray points along +X in local frame.
  const V3 basePalmDir = V3{ 1.0f, 0.0f, 0.0f };   // local +X
  const V3 baseFingDir = V3{ 1.0f, 0.0f, 0.0f };   // local +X

  display.clear();

  // Draw a tiny cross at the wrist origin for reference
  {
    int cx, cy;
    if (project(V3{0,0,0}, cx, cy)) {
      display.setPixel(cx, cy);
      if (cx+1 < kScreenW) display.setPixel(cx+1, cy);
      if (cy+1 < kScreenH) display.setPixel(cx, cy+1);
      if (cx-1 >= 0)        display.setPixel(cx-1, cy);
      if (cy-1 >= 0)        display.setPixel(cx, cy-1);
    }
  }

  // Fan the palm rays around Z by ±2*spacing (five rays total)
  // Map indices [0..4] to angles [-2,-1,0,1,2]*spacing
  for (int i = 0; i < 5; ++i) {
    float k = (float)(i - 2); // -2..+2
    float deg = k * kPalmDegSpacing;

    // Local palm direction rotated around Z for the fan, then rotate by wrist orientation to world
    V3 palmLocal  = rotZ_apply(deg, basePalmDir);
    V3 palmWorld  = q_rotate(qWrist, palmLocal);

    // Palm segment endpoints
    V3 P0 = V3{0,0,0};
    V3 P1 = V3{ palmWorld.x * kPalmLen, palmWorld.y * kPalmLen, palmWorld.z * kPalmLen };

    // Finger orientation: by default use the finger sensor's absolute quaternion
    const Q& qFinger = q[i];
    V3 fingWorld = q_rotate(qFinger, baseFingDir);

    V3 F1 = V3{
      P1.x + fingWorld.x * kFingerLen,
      P1.y + fingWorld.y * kFingerLen,
      P1.z + fingWorld.z * kFingerLen
    };

    // Project & draw
    int x0,y0,x1,y1,xf,yf;
    if (project(P0, x0, y0) && project(P1, x1, y1)) {
      drawLineSafe(x0, y0, x1, y1);     // wrist -> knuckle (palm ray)
    }
    if (project(P1, x1, y1) && project(F1, xf, yf)) {
      drawLineSafe(x1, y1, xf, yf);     // knuckle -> fingertip (finger ray)
    }
  }

  // Optional: a tiny HUD (centered dot already shows origin)
  // display.drawString(0, 0, ""); // keep clean for max FPS

  display.display();
}

#endif // USE_VISUALIZATION
