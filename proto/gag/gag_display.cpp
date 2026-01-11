// #ifdef USE_VISUALIZATION

#include "gag_display.h"
#include <math.h>

// ================== Debug controls ==================
// Uncomment to enable serial debug prints
// #define VIZ_DEBUG 1
#ifndef VIZ_DEBUG_INTERVAL_MS
#define VIZ_DEBUG_INTERVAL_MS 200
#endif
// ====================================================

// ---------- OLED driver ----------
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

static inline void drawLineSafe(int x0, int y0, int x1, int y1) {
  display.drawLine(x0, y0, x1, y1);
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

  display.clear();

  // Draw a tiny cross at the wrist origin for reference (with 30px left offset)
  {
    int cx, cy;
    if (project_with_offset(V3{0,0,0}, cx, cy, kHandOffsetXpx, kHandOffsetYpx)) {
      display.setPixel(cx, cy);
      if (cx+1 < kScreenW) display.setPixel(cx+1, cy);
      if (cy+1 < kScreenH) display.setPixel(cx, cy+1);
      if (cx-1 >= 0)        display.setPixel(cx-1, cy);
      if (cy-1 >= 0)        display.setPixel(cx, cy-1);
    }
  }

  // -------- Debug throttling --------
  #ifdef VIZ_DEBUG
    bool doLog = false;
    uint32_t now = millis();
    if (now - g_lastDebugMs >= VIZ_DEBUG_INTERVAL_MS) {
      g_lastDebugMs = now;
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

    if (okP0 && okP1) drawLineSafe(x0, y0, x1, y1); // wrist -> knuckle
    if (okP1 && okF1) drawLineSafe(x1, y1, xf, yf); // knuckle -> fingertip

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

  // ================= 3D wire cubes for each sensor =================
  // Layout region: to the RIGHT of screen center, arranged 2 rows × 3 columns.
  {
    const int cols = 3, rows = 2;
    const int leftPx   = (kScreenW / 2) + 1;                 // start just right of center
    const int rightW   = max(0, kScreenW - leftPx - 1);      // available width on right
    const int totalH   = max(0, kScreenH - 2);               // small vertical margins

    if (rightW > 6 && totalH > 6) {
      const int cellW = rightW / cols;
      const int cellH = totalH / rows;
      const int topPx = 1;

      // Cube half-size in pixels:
      //   previous: ~0.4 * min(cellW, cellH)  (with >=3px minimum)
      //   NEW: scale to 2/3 of the previous size
      const int   minCell    = min(cellW, cellH);
      const float baseHalfPx = (float)max(3, (minCell * 8) / 20); // 0.4 * minCell
      const float halfPx     = baseHalfPx * (2.0f / 3.0f);        // <-- 2/3 size

      // For screen → world mapping at a fixed z
      const float cxScr  = (float)kScreenW * 0.5f;
      const float cyScr  = (float)kScreenH * 0.5f;
      const float zWorld = 0.0f;                         // cubes lie in z=0 plane
      const float denom  = kUsePerspective ? (zWorld + kZ0) : 1.0f;

      // 12 cube edges (indices into 8-vertex list)
      static const uint8_t edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0},  // bottom
        {4,5},{5,6},{6,7},{7,4},  // top
        {0,4},{1,5},{2,6},{3,7}   // uprights
      };

      for (int s = 0; s < GAG_NUM_SENSORS; ++s) {
        const int r = s / cols;                    // row: 0..1
        const int c = s % cols;                    // col: 0..2
        const int cxPx = leftPx + c*cellW + cellW/2;
        const int cyPx = topPx  + r*cellH + cellH/2;

        // Convert cell-center screen position to world at zWorld
        const float u = ((float)cxPx - cxScr) / kScale;
        const float v = (cyScr - (float)cyPx) / kScale; // note screen Y flips
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
        for (int e = 0; e < 12; ++e) {
          int a = edges[e][0], b = edges[e][1];
          int x0,y0,x1,y1;
          bool okA = project(vWorld[a], x0, y0);
          bool okB = project(vWorld[b], x1, y1);
          if (okA && okB) drawLineSafe(x0, y0, x1, y1);
        }
      }
    }
  }

  display.display();
}

// #endif // USE_VISUALIZATION
