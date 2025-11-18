#define USE_VISUALIZATION 1
#ifdef USE_VISUALIZATION

#include "gag_display.h"
#include <math.h>

// ================== Debug controls ==================
// Comment out to silence all debug prints from this module
// #define VIZ_DEBUG 1
// Throttle debug output to at most once per this many milliseconds
#ifndef VIZ_DEBUG_INTERVAL_MS
#define VIZ_DEBUG_INTERVAL_MS 200
#endif
// ====================================================

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

// Debug timing
#ifdef VIZ_DEBUG
static uint32_t g_lastDebugMs = 0;
#endif

// ---------- Small math helpers ----------
struct V3 { float x, y, z; };
struct Q  { float w, x, y, z; };

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

// static inline bool project(const V3& p, int& x, int& y) {
//   float u, v;
//   if (kUsePerspective) {
//     float denom = (p.z + kZ0);
//     if (denom < 1.0f) return false;  // behind camera; drop
//     u = p.x / denom;
//     v = p.y / denom;
//   } else {
//     u = p.x;
//     v = p.y;
//   }
//   x = (int)(kScreenW * 0.5f + kScale * u + 0.5f);
//   y = (int)(kScreenH * 0.5f - kScale * v + 0.5f);
//   return (x > -10 && x < kScreenW + 10 && y > -10 && y < kScreenH + 10);
// }

static inline bool project(const V3& p_in, int& x, int& y) {
  // Rotate WORLD by 180° about the vertical (Y) axis
  // Quaternion for 180° about +Y: cos(pi/2)=0, sin(pi/2)=1  ->  {w=0, x=0, y=1, z=0}
  const Q kWorldYaw180 = Q{0.0f, 0.0f, 1.0f, 0.0f};
  V3 p = q_rotate(kWorldYaw180, p_in);

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


static inline void drawLineSafe(int x0, int y0, int x1, int y1) {
  display.drawLine(x0, y0, x1, y1);
}

// ---------- Public API ----------
// void viz_init() {
void viz_init(void) {
  // Serial.println(F("[VIZ] init"));
  Serial.println("[VIZ] init");
  delay(100);
  display.init();
  // display.flipScreenVertically(); // Uncomment if your panel is flipped
  Serial.println(F("[VIZ] done"));

  // Infer screen size from driver (defaults are 128x64 on SSD1306)
  kScreenW = display.getWidth();
  kScreenH = display.getHeight();

  // Compute a conservative scale so palm+finger fits with margins
  float totalLen = (kPalmLen + kFingerLen);
  kScale = (0.8f * (float)min(kScreenW, kScreenH)) / (2.0f * totalLen);
  if (kScale < 0.5f) kScale = 0.5f;  // reasonable lower bound

  Serial.println(F("[VIZ] clear"));

  display.clear();
  Serial.println(F("[VIZ] font"));
  display.setFont(ArialMT_Plain_10);
  Serial.println(F("[VIZ] string"));
  display.drawString(0, 0, "GAG viz ready");
  Serial.println(F("[VIZ] display"));
  display.display();

  #ifdef VIZ_DEBUG
    Serial.println(F("[VIZ] init"));
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

// // Expect sensors in order [0..4]=fingers, [5]=wrist(HG)
// void viz_draw_frame(const VizQuaternion q_in[GAG_NUM_SENSORS]) {
//   // Convert to internal Q and normalize
//   Q q[GAG_NUM_SENSORS];
//   for (int i = 0; i < GAG_NUM_SENSORS; ++i) {
//     q[i] = Q{ q_in[i].w, q_in[i].x, q_in[i].y, q_in[i].z };
//     float n = q[i].w*q[i].w + q[i].x*q[i].x + q[i].y*q[i].y + q[i].z*q[i].z;
//     if (n > 0.00001f) {
//       float inv = 1.0f / sqrtf(n);
//       q[i].w *= inv; q[i].x *= inv; q[i].y *= inv; q[i].z *= inv;
//     } else {
//       q[i] = Q{1,0,0,0};
//     }
//   }

//   const Q& qWrist = q[GAG_WRIST_INDEX];

//   // Model: wrist at origin (0,0,0). Base palm & finger ray points along +X in local frame.
//   const V3 basePalmDir = V3{ 1.0f, 0.0f, 0.0f };
//   const V3 baseFingDir = V3{ 1.0f, 0.0f, 0.0f };

//   display.clear();

//   // Draw a tiny cross at the wrist origin for reference
//   {
//     int cx, cy;
//     if (project(V3{0,0,0}, cx, cy)) {
//       display.setPixel(cx, cy);
//       if (cx+1 < kScreenW) display.setPixel(cx+1, cy);
//       if (cy+1 < kScreenH) display.setPixel(cx, cy+1);
//       if (cx-1 >= 0)        display.setPixel(cx-1, cy);
//       if (cy-1 >= 0)        display.setPixel(cx, cy-1);
//     }
//   }

//   // Debug throttling
//   #ifdef VIZ_DEBUG
//     bool doLog = false;
//     uint32_t now = millis();
//     if (now - g_lastDebugMs >= VIZ_DEBUG_INTERVAL_MS) {
//       g_lastDebugMs = now;
//       doLog = true;
//       Serial.print(F("[VIZ] frame t=")); Serial.print(now);
//       Serial.print(F("ms scale=")); Serial.print(kScale, 3);
//       Serial.print(F(" spacing=")); Serial.print(kPalmDegSpacing, 1);
//       Serial.print(F(" persp=")); Serial.println(kUsePerspective ? F("1") : F("0"));
//     }
//   #endif

//   // Fan the palm rays around Z by ±2*spacing (five rays total)
//   // Map indices [0..4] to angles [-2,-1,0,1,2]*spacing
//   for (int i = 0; i < 5; ++i) {
//     float k = (float)(i - 2); // -2..+2
//     float deg = k * kPalmDegSpacing;

//     // Local palm direction rotated around Z for the fan, then rotate by wrist orientation to world
//     V3 palmLocal  = rotZ_apply(deg, basePalmDir);
//     V3 palmWorld  = q_rotate(qWrist, palmLocal);

//     // Palm segment endpoints
//     V3 P0 = V3{0,0,0};
//     V3 P1 = V3{ palmWorld.x * kPalmLen, palmWorld.y * kPalmLen, palmWorld.z * kPalmLen };

//     // Finger orientation: interpreted as absolute orientation in world
//     const Q& qFinger = q[i];
//     V3 fingWorld = q_rotate(qFinger, baseFingDir);

//     V3 F1 = V3{
//       P1.x + fingWorld.x * kFingerLen,
//       P1.y + fingWorld.y * kFingerLen,
//       P1.z + fingWorld.z * kFingerLen
//     };

//     // Project & draw
//     int x0,y0,x1,y1,xf,yf;
//     bool okP0 = project(P0, x0, y0);
//     bool okP1 = project(P1, x1, y1);
//     bool okF1 = project(F1, xf, yf);

//     if (okP0 && okP1) drawLineSafe(x0, y0, x1, y1); // wrist -> knuckle
//     if (okP1 && okF1) drawLineSafe(x1, y1, xf, yf); // knuckle -> fingertip

//     #ifdef VIZ_DEBUG
//       if (doLog) {
//         Serial.print(F("  [ray ")); Serial.print(i); Serial.print(F("] deg=")); Serial.print(deg, 1);
//         Serial.print(F(" P1=(")); Serial.print(P1.x, 2); Serial.print(',');
//         Serial.print(P1.y, 2); Serial.print(','); Serial.print(P1.z, 2); Serial.print(')');
//         Serial.print(F(" F1=(")); Serial.print(F1.x, 2); Serial.print(',');
//         Serial.print(F1.y, 2); Serial.print(','); Serial.print(F1.z, 2); Serial.print(')');
//         Serial.print(F(" | scr P0=(")); Serial.print(x0); Serial.print(','); Serial.print(y0); Serial.print(')');
//         Serial.print(F(" P1=(")); Serial.print(okP1 ? x1 : -1); Serial.print(',');
//         Serial.print(okP1 ? y1 : -1); Serial.print(')');
//         Serial.print(F(" F1=(")); Serial.print(okF1 ? xf : -1); Serial.print(',');
//         Serial.print(okF1 ? yf : -1); Serial.println(')');
//       }
//     #endif
//   }

//   display.display();
// }


// // Expect sensors in order [0..4]=fingers, [5]=wrist(HG)
// void viz_draw_frame(const VizQuaternion q_in[GAG_NUM_SENSORS]) {
//   // --- Small local helpers (no globals needed) ---
//   auto qMul  = [](const Q& a, const Q& b) -> Q {
//     return Q{
//       a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
//       a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
//       a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
//       a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
//     };
//   };
//   auto qConj = [](const Q& q) -> Q { return Q{ q.w, -q.x, -q.y, -q.z }; };

//   // Convert to internal Q and normalize
//   Q q[GAG_NUM_SENSORS];
//   for (int i = 0; i < GAG_NUM_SENSORS; ++i) {
//     q[i] = Q{ q_in[i].w, q_in[i].x, q_in[i].y, q_in[i].z };
//     float n = q[i].w*q[i].w + q[i].x*q[i].x + q[i].y*q[i].y + q[i].z*q[i].z;
//     if (n > 0.00001f) {
//       float inv = 1.0f / sqrtf(n);
//       q[i].w *= inv; q[i].x *= inv; q[i].y *= inv; q[i].z *= inv;
//     } else {
//       q[i] = Q{1,0,0,0};
//     }
//   }

//   const Q& qWrist = q[GAG_WRIST_INDEX];

//   // ----- Fixed finger mount offset: +90° about Z in the WRIST frame -----
//   const float half90 = 0.5f * deg2rad(90.0f);
//   const float s90    = sinf(half90), c90 = cosf(half90);
//   const Q kC        = Q{ c90, 0.0f, 0.0f,  s90 }; // +90° Z
//   const Q kC_inv    = Q{ c90, 0.0f, 0.0f, -s90 }; //  -90° Z (to remove offset)

//   // ----- 180° rotation about Y (vertical) to apply to FINGERS only -----
//   // World-axis Y flip:
//   const Q kRY180 = Q{ 0.0f, 0.0f, 1.0f, 0.0f };

//   // Model: wrist at origin; base directions in local frame
//   const V3 basePalmDir = V3{ 1.0f, 0.0f, 0.0f };
//   const V3 baseFingDir = V3{ 1.0f, 0.0f, 0.0f };

//   display.clear();

//   // Tiny cross at wrist origin
//   {
//     int cx, cy;
//     if (project(V3{0,0,0}, cx, cy)) {
//       display.setPixel(cx, cy);
//       if (cx+1 < kScreenW) display.setPixel(cx+1, cy);
//       if (cy+1 < kScreenH) display.setPixel(cx, cy+1);
//       if (cx-1 >= 0)        display.setPixel(cx-1, cy);
//       if (cy-1 >= 0)        display.setPixel(cx, cy-1);
//     }
//   }

//   // Debug throttling
//   #ifdef VIZ_DEBUG
//     bool doLog = false;
//     uint32_t now = millis();
//     if (now - g_lastDebugMs >= VIZ_DEBUG_INTERVAL_MS) {
//       g_lastDebugMs = now;
//       doLog = true;
//       Serial.print(F("[VIZ] frame t=")); Serial.print(now);
//       Serial.print(F("ms scale=")); Serial.print(kScale, 3);
//       Serial.print(F(" spacing=")); Serial.print(kPalmDegSpacing, 1);
//       Serial.print(F(" persp=")); Serial.println(kUsePerspective ? F("1") : F("0"));
//     }
//   #endif

//   // Fan the palm rays around Z by ±2*spacing (five rays total)
//   for (int i = 0; i < 5; ++i) {
//     float k = (float)(i - 2); // -2..+2
//     float deg = k * kPalmDegSpacing;

//     // Palm direction
//     V3 palmLocal = rotZ_apply(deg, basePalmDir);
//     V3 palmWorld = q_rotate(qWrist, palmLocal);

//     // Segment endpoints for palm
//     V3 P0 = V3{0,0,0};
//     V3 P1 = V3{ palmWorld.x * kPalmLen, palmWorld.y * kPalmLen, palmWorld.z * kPalmLen };

//     // --------- Finger orientation pipeline ----------
//     // qRel       = inv(qWrist) * qFinger           (finger in wrist frame)
//     // qRelCorr   = kC_inv * qRel                   (remove +90° Z mount offset)
//     // qWorldF    = qWrist * qRelCorr               (back to world)
//     // qWorldF    = kRY180 * qWorldF                (flip fingers 180° about WORLD Y)  <-- (Option A, default)
//     Q qRel     = qMul(qConj(qWrist), q[i]);
//     Q qRelCorr = qMul(kC_inv, qRel);
//     Q qWorldF  = qMul(qWrist, qRelCorr);

//     // Apply 180° Y to fingers ONLY, in WORLD coordinates:
//     qWorldF = qMul(kRY180, qWorldF);

//     // ---- If you instead want the 180° to be in the WRIST (local) frame, use this line instead and remove the previous one:
//     // qWorldF = qMul(qWrist, qMul(kRY180, qRelCorr)); // (Option B: local Y in wrist frame)

//     // Direction to fingertip
//     V3 fingWorld = q_rotate(qWorldF, baseFingDir);

//     V3 F1 = V3{
//       P1.x + fingWorld.x * kFingerLen,
//       P1.y + fingWorld.y * kFingerLen,
//       P1.z + fingWorld.z * kFingerLen
//     };

//     // Project & draw
//     int x0,y0,x1,y1,xf,yf;
//     bool okP0 = project(P0, x0, y0);
//     bool okP1 = project(P1, x1, y1);
//     bool okF1 = project(F1, xf, yf);

//     if (okP0 && okP1) drawLineSafe(x0, y0, x1, y1); // wrist -> knuckle
//     if (okP1 && okF1) drawLineSafe(x1, y1, xf, yf); // knuckle -> fingertip

//     #ifdef VIZ_DEBUG
//       if (doLog) {
//         Serial.print(F("  [ray ")); Serial.print(i); Serial.print(F("] deg=")); Serial.print(deg, 1);
//         Serial.print(F(" P1=(")); Serial.print(P1.x, 2); Serial.print(',');
//         Serial.print(P1.y, 2); Serial.print(','); Serial.print(P1.z, 2); Serial.print(')');
//         Serial.print(F(" F1=(")); Serial.print(F1.x, 2); Serial.print(',');
//         Serial.print(F1.y, 2); Serial.print(','); Serial.print(F1.z, 2); Serial.print(')');
//         Serial.println();
//       }
//     #endif
//   }

//   display.display();
// }


// ---- Finger offset controls (degrees) ----
// Apply to all fingers. Typical values: 0, ±90, ±180
static float gFingerOffDegX = 0.0f;
static float gFingerOffDegY = 0.0f;
static float gFingerOffDegZ = 0.0f;

// When true (default), offsets are applied in the wrist frame (recommended).
// If false, offsets are applied in the finger sensor's local frame.
static bool  gFingerOffsetsInWristFrame = true;

void viz_set_finger_offsets(float offX, float offY, float offZ) {
  gFingerOffDegX = offX; gFingerOffDegY = offY; gFingerOffDegZ = offZ;
}

void viz_finger_offsets_use_wrist_frame(bool enable) {
  gFingerOffsetsInWristFrame = enable;
}


void viz_draw_frame(const VizQuaternion q_in[GAG_NUM_SENSORS]) {
  // ---------- Convert to internal Q and normalize ----------
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

  const Q& qWrist = q[GAG_WRIST_INDEX];

  // ---------- Local quaternion helpers ----------
  struct QM {
    static inline Q mul(const Q& a, const Q& b) {
      return Q{
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
      };
    }
    static inline Q conj(const Q& q) { return Q{ q.w, -q.x, -q.y, -q.z }; }
    static inline Q fromAxisAngle(const V3& axis, float degrees) {
      float half = 0.5f * deg2rad(degrees);
      float s = sinf(half), c = cosf(half);
      float n = sqrtf(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
      if (n < 1e-6f) return Q{1,0,0,0};
      return Q{ c, (axis.x/n)*s, (axis.y/n)*s, (axis.z/n)*s };
    }
  };

  // ---------- Precompute the finger offset quaternion (X then Y then Z) ----------
  // Apply X, then Y, then Z to the vector  =>  qOffset = Rz * Ry * Rx
  const Q qOffX = QM::fromAxisAngle(V3{1,0,0}, gFingerOffDegX);
  const Q qOffY = QM::fromAxisAngle(V3{0,1,0}, gFingerOffDegY);
  const Q qOffZ = QM::fromAxisAngle(V3{0,0,1}, gFingerOffDegZ);
  const Q qOffset = QM::mul(QM::mul(qOffZ, qOffY), qOffX);

  // ---------- Drawing prep ----------
  display.clear();

  // Wrist origin cross
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

  // Debug throttling
  #ifdef VIZ_DEBUG
    bool doLog = false;
    uint32_t now = millis();
    if (now - g_lastDebugMs >= VIZ_DEBUG_INTERVAL_MS) {
      g_lastDebugMs = now; doLog = true;
      Serial.print(F("[VIZ] frame t=")); Serial.print(now);
      Serial.print(F(" scale=")); Serial.print(kScale, 3);
      Serial.print(F(" spacing=")); Serial.print(kPalmDegSpacing, 1);
      Serial.print(F(" persp=")); Serial.println(kUsePerspective ? F("1") : F("0"));
      Serial.print(F(" fingerOff (x,y,z)=("));
      Serial.print(gFingerOffDegX,0); Serial.print(',');
      Serial.print(gFingerOffDegY,0); Serial.print(',');
      Serial.print(gFingerOffDegZ,0); Serial.println(')');
    }
  #endif

  // Base directions in local frame
  const V3 basePalmDir = V3{ 1.0f, 0.0f, 0.0f };
  const V3 baseFingDir = V3{ 1.0f, 0.0f, 0.0f };

  // ---------- Five palm rays ----------
  for (int i = 0; i < 5; ++i) {
    float k = (float)(i - 2);           // -2..+2
    float deg = k * kPalmDegSpacing;    // fan around Z

    // Palm ray in world
    V3 palmLocal = rotZ_apply(deg, basePalmDir);
    V3 palmWorld = q_rotate(qWrist, palmLocal);

    // Wrist -> knuckle segment
    V3 P0 = V3{0,0,0};
    V3 P1 = V3{ palmWorld.x * kPalmLen, palmWorld.y * kPalmLen, palmWorld.z * kPalmLen };

    // ---------- Finger orientation with configurable offsets ----------
    // Get raw finger orientation
    const Q& qFingerRaw = q[i];

    // Relative finger orientation to wrist
    Q qRel = QM::mul(QM::conj(qWrist), qFingerRaw);

    // Apply offsets either in wrist frame (pre-multiply) or sensor-local frame (post-multiply)
    Q qRelCorr = gFingerOffsetsInWristFrame
                 ? QM::mul(qOffset, qRel)   // offsets in wrist frame
                 : QM::mul(qRel, qOffset);  // offsets in sensor-local frame

    // Back to world
    Q qFingerWorld = QM::mul(qWrist, qRelCorr);

    // Fingertip direction & endpoint
    V3 fingWorld = q_rotate(qFingerWorld, baseFingDir);
    V3 F1 = V3{
      P1.x + fingWorld.x * kFingerLen,
      P1.y + fingWorld.y * kFingerLen,
      P1.z + fingWorld.z * kFingerLen
    };

    // Project & draw
    int x0,y0,x1,y1,xf,yf;
    bool okP0 = project(P0, x0, y0);
    bool okP1 = project(P1, x1, y1);
    bool okF1 = project(F1, xf, yf);

    if (okP0 && okP1) drawLineSafe(x0, y0, x1, y1); // wrist -> knuckle
    if (okP1 && okF1) drawLineSafe(x1, y1, xf, yf); // knuckle -> fingertip

    #ifdef VIZ_DEBUG
      if (doLog) {
        Serial.print(F("  [ray ")); Serial.print(i);
        Serial.print(F("] deg=")); Serial.print(deg,1);
        Serial.print(F(" P1=(")); Serial.print(P1.x,2); Serial.print(',');
        Serial.print(P1.y,2); Serial.print(','); Serial.print(P1.z,2); Serial.print(')');
        Serial.print(F(" F1=(")); Serial.print(F1.x,2); Serial.print(',');
        Serial.print(F1.y,2); Serial.print(','); Serial.print(F1.z,2); Serial.print(')');
        Serial.println();
      }
    #endif
  }

  display.display();
}



#endif // USE_VISUALIZATION
