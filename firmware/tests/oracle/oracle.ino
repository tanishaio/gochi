// oracle.ino — "The Desk Oracle": a standalone shake-to-divine fortune ball.
//
// A self-contained sketch (no gochi daemon, no host tether). Shake it awake,
// the *kind* of shake picks the question, how *hard* you shake colours the
// answer, and after a suspenseful beat it reveals a verdict with expressive
// faces, animated dice/cards, and themed buzzer jingles.
//
//   side-to-side  → YES / NO       (a goofy Magic-8-Ball answer set)
//   up-and-down   → OPTION 1–4      (lands as a rolling die with pips)
//   shake like crazy → SURF'S UP 🛹 (a Subway-Surfers-style easter egg)
//
// The two gestures are told apart by projecting linear accel onto the live
// gravity vector (so it works at any resting tilt): vertical-dominant swing
// = up-and-down (option), otherwise = side-to-side (yes/no).
//
// Hardware (all from src/config.h — nothing hard-coded here):
//   OLED  SSD1306 128x64 on hardware I2C  SDA=GPIO5  SCL=GPIO6  (U8g2)
//   MPU-6050 on a *bit-banged* I2C bus     SDA=GPIO7  SCL=GPIO8
//   Passive piezo buzzer on               PIN_BUZZER=GPIO10  (LEDC)
//
// The C3 has one hardware I2C controller (the OLED's), so the IMU rides a
// software bus — same split the main firmware uses (src/imu/mpu6050.cpp).
// The onboard LED (GPIO8) is left alone: it shares the MPU's SCL pin.
//
// Build:  arduino-cli compile --profile c3   (or `make test-oracle`)
// Flash:  make test-oracle      Watch values: make monitor  (115200)
//
// Main tuning dials are grouped under "Tuning" below.

#include <Arduino.h>
#include <U8g2lib.h>
#include <math.h>

#include "../../src/config.h"

// =====================================================================
//  Display — identical constructor to the main firmware / oled test.
// =====================================================================
#ifdef ROTATED_DISPLAY
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R2, U8X8_PIN_NONE, PIN_SCL, PIN_SDA);
#else
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, PIN_SCL, PIN_SDA);
#endif

// =====================================================================
//  Types — declared up front. The Arduino .ino preprocessor injects
//  auto-generated prototypes *above the first function*, so any type used
//  in a signature must already be visible here.
// =====================================================================
struct Note {
  uint16_t freq;  // Hz; 0 = rest
  uint16_t ms;
};
enum class State : uint8_t { Sleeping, Charging, Thinking, Reveal };
enum class Mode : uint8_t { YesNo, Option };  // side-to-side / up-down

// One Magic-8-Ball answer. `tone` drives the face + jingle:
//   +2 emphatic yes, +1 yes, 0 non-committal, -1 no, -2 emphatic no.
struct Answer {
  const char* big;
  const char* sub;
  int8_t tone;
};

// =====================================================================
//  MPU-6050 — bit-banged I2C driver (inline copy of src/imu/mpu6050.cpp,
//  trimmed to begin()/read()). Toggles pinMode on PIN_MPU_SDA/SCL; never
//  touches Wire, so it coexists with the OLED's hardware I2C.
// =====================================================================
namespace mpu {

struct Sample {
  float ax, ay, az;  // g
  float gx, gy, gz;  // °/s
};

namespace {
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

constexpr float ACCEL_LSB_PER_G = 8192.0f;   // ±4 g
constexpr float GYRO_LSB_PER_DPS = 65.5f;    // ±500 °/s
constexpr uint32_t HALF_BIT_US = 4;          // ~100 kHz bit-bang

bool ready = false;
bool pinsUp = false;
uint8_t addr = MPU_ADDR;  // resolved at begin() — 0x68 or 0x69

inline void sclHigh() { pinMode(PIN_MPU_SCL, INPUT); }
inline void sclLow() {
  pinMode(PIN_MPU_SCL, OUTPUT);
  digitalWrite(PIN_MPU_SCL, LOW);
}
inline void sdaHigh() { pinMode(PIN_MPU_SDA, INPUT); }
inline void sdaLow() {
  pinMode(PIN_MPU_SDA, OUTPUT);
  digitalWrite(PIN_MPU_SDA, LOW);
}
inline bool sdaRead() { return digitalRead(PIN_MPU_SDA) == HIGH; }

void busBringUp() {
  if (pinsUp) return;
  sdaHigh();
  sclHigh();
  delayMicroseconds(HALF_BIT_US * 2);
  pinsUp = true;
}

void i2cStart() {
  sdaHigh();
  sclHigh();
  delayMicroseconds(HALF_BIT_US);
  sdaLow();
  delayMicroseconds(HALF_BIT_US);
  sclLow();
  delayMicroseconds(HALF_BIT_US);
}

void i2cStop() {
  sdaLow();
  delayMicroseconds(HALF_BIT_US);
  sclHigh();
  delayMicroseconds(HALF_BIT_US);
  sdaHigh();
  delayMicroseconds(HALF_BIT_US);
}

bool i2cWriteByte(uint8_t b) {
  for (int i = 7; i >= 0; --i) {
    (b & (1u << i)) ? sdaHigh() : sdaLow();
    delayMicroseconds(HALF_BIT_US);
    sclHigh();
    delayMicroseconds(HALF_BIT_US);
    sclLow();
  }
  sdaHigh();  // release for ACK
  delayMicroseconds(HALF_BIT_US);
  sclHigh();
  delayMicroseconds(HALF_BIT_US);
  bool ack = !sdaRead();
  sclLow();
  delayMicroseconds(HALF_BIT_US);
  return ack;
}

uint8_t i2cReadByte(bool sendAck) {
  uint8_t b = 0;
  sdaHigh();
  for (int i = 7; i >= 0; --i) {
    delayMicroseconds(HALF_BIT_US);
    sclHigh();
    delayMicroseconds(HALF_BIT_US);
    if (sdaRead()) b |= (1u << i);
    sclLow();
  }
  sendAck ? sdaLow() : sdaHigh();
  delayMicroseconds(HALF_BIT_US);
  sclHigh();
  delayMicroseconds(HALF_BIT_US);
  sclLow();
  delayMicroseconds(HALF_BIT_US);
  sdaHigh();
  return b;
}

bool writeReg(uint8_t reg, uint8_t value) {
  i2cStart();
  bool ok = i2cWriteByte((addr << 1) | 0) && i2cWriteByte(reg) && i2cWriteByte(value);
  i2cStop();
  return ok;
}

bool readReg(uint8_t reg, uint8_t* out) {
  i2cStart();
  if (!i2cWriteByte((addr << 1) | 0) || !i2cWriteByte(reg)) {
    i2cStop();
    return false;
  }
  i2cStart();  // repeated start
  if (!i2cWriteByte((addr << 1) | 1)) {
    i2cStop();
    return false;
  }
  *out = i2cReadByte(/*sendAck=*/false);
  i2cStop();
  return true;
}

bool readBurst(uint8_t reg, uint8_t* buf, size_t n) {
  i2cStart();
  if (!i2cWriteByte((addr << 1) | 0) || !i2cWriteByte(reg)) {
    i2cStop();
    return false;
  }
  i2cStart();
  if (!i2cWriteByte((addr << 1) | 1)) {
    i2cStop();
    return false;
  }
  for (size_t i = 0; i < n; ++i) buf[i] = i2cReadByte(/*sendAck=*/i < n - 1);
  i2cStop();
  return true;
}

bool probe(uint8_t a) {
  i2cStart();
  bool ack = i2cWriteByte((a << 1) | 0);
  i2cStop();
  return ack;
}
}  // namespace

bool begin() {
  busBringUp();
  // Auto-detect address: try 0x68, then 0x69 (AD0 high). Saves the user
  // from caring which way their GY-521's AD0 jumper is set.
  if (probe(0x68)) addr = 0x68;
  else if (probe(0x69)) addr = 0x69;
  else { ready = false; return false; }

  uint8_t who = 0;
  if (!readReg(REG_WHO_AM_I, &who) || who == 0x00 || who == 0xFF) {
    ready = false;
    return false;
  }
  bool ok = writeReg(REG_PWR_MGMT_1, 0x80);  // reset
  delay(100);
  ok = ok && writeReg(REG_PWR_MGMT_1, 0x01);   // wake + PLL
  ok = ok && writeReg(REG_CONFIG, 0x03);       // 44 Hz DLPF
  ok = ok && writeReg(REG_SMPLRT_DIV, 0x04);   // 200 Hz
  ok = ok && writeReg(REG_GYRO_CONFIG, 0x08);  // ±500 °/s
  ok = ok && writeReg(REG_ACCEL_CONFIG, 0x08); // ±4 g
  ready = ok;
  return ok;
}

uint8_t address() { return addr; }

bool read(Sample& out) {
  if (!ready) return false;
  uint8_t buf[14];
  if (!readBurst(REG_ACCEL_XOUT_H, buf, sizeof(buf))) return false;
  int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
  int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
  int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
  int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);
  out.ax = ax / ACCEL_LSB_PER_G;
  out.ay = ay / ACCEL_LSB_PER_G;
  out.az = az / ACCEL_LSB_PER_G;
  out.gx = gx / GYRO_LSB_PER_DPS;
  out.gy = gy / GYRO_LSB_PER_DPS;
  out.gz = gz / GYRO_LSB_PER_DPS;
  return true;
}

}  // namespace mpu

// =====================================================================
//  Buzzer — LEDC, same approach as the buzzer bring-up test.
// =====================================================================
static void buzz(uint16_t freq) { ledcWriteTone(PIN_BUZZER, freq); }
static void silence() { ledcWriteTone(PIN_BUZZER, 0); }
static void playMelody(const Note* notes, uint8_t n) {
  for (uint8_t i = 0; i < n; ++i) {
    buzz(notes[i].freq);
    delay(notes[i].ms);
  }
  silence();
}

// =====================================================================
//  Tuning ===============================================================
// =====================================================================
static const float WAKE_LIN_G = 0.45f;       // kinetic accel that wakes it
static const float CALM_LIN_G = 0.18f;        // below this = "shaking stopped"
static const uint32_t CALM_HOLD_MS = 360;     // calm must persist this long
static const uint32_t THINK_MIN_MS = 1000;    // suspense beat (randomised up to +)
static const uint32_t THINK_RAND_MS = 800;
static const uint32_t REVEAL_MS = 4500;       // how long the verdict shows
static const float EMPHATIC_ENERGY = 10.0f;   // shake energy → emphatic yes/no
static const float CHARGE_FULL = 14.0f;       // energy that fills the charge bar
static const uint8_t YESNO_SPLIT = 52;        // <split = yes, slight yes bias
static const uint8_t NEUTRAL_PCT = 12;        // % "the spirits are buffering"
static const uint8_t OPTION_COUNT = 4;        // up-down picks 1..OPTION_COUNT
static const float MEGA_ENERGY = 17.0f;       // shake THIS hard → SURF'S UP 🛹

// =====================================================================
//  Runtime state ========================================================
// =====================================================================
static State state = State::Sleeping;
static uint32_t stateSince = 0;

// Gravity low-pass (per axis) — finds "down" so gestures work at any tilt.
static float gX = 0, gY = 0, gZ = 1;
static bool gSeeded = false;
static const float GRAVITY_ALPHA = 0.02f;

// Gyro bias, learned while the device sleeps (it has a few °/s of offset).
static float biasGx = 0, biasGy = 0, biasGz = 0;

// Per-shake accumulators (reset on wake). Peaks discriminate the gesture;
// shakeEnergy (running sum) drives intensity, the charge bar, and emphasis.
static float peakVert = 0, peakHoriz = 0;
static float shakeEnergy = 0;
static uint32_t calmSince = 0;

static uint32_t thinkDuration = THINK_MIN_MS;

// xorshift32 PRNG, seeded from motion/timing entropy.
static uint32_t rngState = 0x1234567u;
static uint32_t rng() {
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return rngState;
}
static uint8_t rngBelow(uint8_t n) { return (uint8_t)(rng() % n); }

// Reveal payload.
static Mode mode = Mode::YesNo;
static char verdict[16] = "";
static char flavor[24] = "";
static int8_t revealTone = 0;
static uint8_t dieN = 1;   // for Option mode (1..OPTION_COUNT)
static bool megaShake = false;  // shook hard enough for the Subway easter egg

// =====================================================================
//  Magic-8-Ball answer pools ===========================================
// =====================================================================
static const Answer YES_STRONG[] = {
    {"YES!!", "bet the house \\o/", 2}, {"100%", "the universe winked ;)", 2},
    {"HECK YES", "go go go!!", 2},      {"OBVIOUSLY", "duh, legend 8)", 2}};
static const Answer YES_SOFT[] = {
    {"yeah", "vibes check out ^_^", 1}, {"sure", "why not lol", 1},
    {"yep", "i'd risk it :P", 1},       {"likely", "stars say prob yes", 1}};
static const Answer NEUTRAL[] = {
    {"HMMM", "ask again, napping -_-", 0}, {"DUNNO", "shrug \\_( )_/", 0},
    {"BUFFERING", "loading fate... :/", 0}, {"42", "answer to everything", 0},
    {"ASK MOM", "im just a chip lol", 0}};
static const Answer NO_SOFT[] = {
    {"nah", "i wouldn't tbh :/", -1},  {"meh", "outlook = potato", -1},
    {"nope", "doubt it chief", -1},    {"eh no", "bad vibes >_<", -1}};
static const Answer NO_STRONG[] = {
    {"NO WAY", "not in a million yrs", -2}, {"NOPE", "hard pass X(", -2},
    {"DENIED", "the void laughs >:)", -2}, {"ABSOLUTELY", "...not. sorry XD", -2}};

template <typename T, size_t N>
static const Answer& pick(const T (&pool)[N]) {
  return pool[rngBelow((uint8_t)N)];
}

// =====================================================================
//  Drawing helpers ======================================================
// =====================================================================
static const uint8_t* const WORD_FONTS[] = {
    u8g2_font_ncenB24_tr, u8g2_font_ncenB18_tr,
    u8g2_font_ncenB14_tr, u8g2_font_ncenB10_tr};

// Draw a word centred at baseline `y`, using the largest font that fits.
static void drawWord(const char* s, int y) {
  for (uint8_t i = 0; i < 4; ++i) {
    oled.setFont(WORD_FONTS[i]);
    int w = oled.getStrWidth(s);
    if (w <= OLED_W - 4 || i == 3) {
      oled.drawStr((OLED_W - w) / 2, y, s);
      return;
    }
  }
}

static void drawCenteredSmall(const char* s, int y) {
  oled.setFont(u8g2_font_ncenB08_tr);
  oled.drawStr((OLED_W - oled.getStrWidth(s)) / 2, y, s);
}

// A simple mouth arc: smile (up=true) dips low in the middle; frown lifts.
static void drawMouth(int cx, int cy, int halfW, int amp, bool up) {
  for (int dx = -halfW; dx <= halfW; ++dx) {
    float t = (float)dx / halfW;          // -1..1
    int yy = up ? cy + (int)(amp * (1 - t * t)) : cy - (int)(amp * (1 - t * t));
    oled.drawPixel(cx + dx, yy);
    oled.drawPixel(cx + dx, yy + 1);
  }
}

// A pair of eyes with a highlight. `open` 0..1 scales them.
static void drawEyes(int cx, int cy, int r, bool happy) {
  int dx = 22;
  for (int s = -1; s <= 1; s += 2) {
    int ex = cx + s * dx;
    oled.drawDisc(ex, cy, r);
    oled.setDrawColor(0);
    oled.drawDisc(ex - r / 3, cy - r / 3, (r + 1) / 3);  // glint
    oled.setDrawColor(1);
    if (happy) {  // little upper-lid curve for a smiling squint
      oled.drawHLine(ex - r, cy - r - 1, 2 * r);
    }
  }
}

// Render a die face (1..6) as pips on a rounded square centred at (cx,cy).
static void drawDie(int cx, int cy, int half, uint8_t n) {
  oled.drawRFrame(cx - half, cy - half, 2 * half, 2 * half, 4);
  int o = half / 2;       // pip offset from centre
  int pr = half / 6 + 1;  // pip radius
  auto pip = [&](int px, int py) { oled.drawDisc(cx + px, cy + py, pr); };
  switch (n) {
    case 1: pip(0, 0); break;
    case 2: pip(-o, -o); pip(o, o); break;
    case 3: pip(-o, -o); pip(0, 0); pip(o, o); break;
    case 4: pip(-o, -o); pip(o, -o); pip(-o, o); pip(o, o); break;
    case 5: pip(-o, -o); pip(o, -o); pip(0, 0); pip(-o, o); pip(o, o); break;
    default: pip(-o, -o); pip(o, -o); pip(-o, 0); pip(o, 0); pip(-o, o); pip(o, o); break;
  }
}

// =====================================================================
//  Faces / animations ===================================================
// =====================================================================
static void faceSleeping(uint32_t now) {
  oled.clearBuffer();
  float breath = sinf(now / 900.0f);
  int yoff = (int)(breath * 2.0f);
  int cy = OLED_H / 2 + yoff - 4;
  // Occasional blink-twitch: eyes pop to tiny dots for a beat.
  bool twitch = ((now / 97) % 53) == 0;
  if (twitch) {
    oled.drawDisc(44, cy, 2);
    oled.drawDisc(84, cy, 2);
  } else {
    oled.drawHLine(34, cy, 22);  // closed eye
    oled.drawHLine(72, cy, 22);
  }
  // A drifting dream bubble every so often.
  uint32_t phase = (now / 60) % 200;
  if (phase < 40) {
    oled.drawDisc(96, cy - 14 - phase / 8, 2);
    oled.drawDisc(102, cy - 22 - phase / 8, 1);
  }
  // Hint cycles through sleepy one-liners so it reads as a bored pet.
  static const char* HINTS[] = {"z z z", "shake me!", "im bored...",
                                "feed me chaos", "boop me", "(-_-) zzz"};
  drawCenteredSmall(HINTS[(now / 1600) % 6], OLED_H - 4);
  oled.sendBuffer();
}

// Charging: eyes snap open and grow, an expanding aura ring pulses, and a
// charge meter fills along the bottom as energy accumulates.
static void faceCharging(uint32_t now, float intensity, float charge) {
  oled.clearBuffer();
  int r = 6 + (int)(intensity * 12.0f);
  if (r > 18) r = 18;
  int jitter = (intensity > 0.5f) ? (int)(rng() % 5) - 2 : 0;
  int cy = OLED_H / 2 - 4 + jitter;
  // aura rings
  int aura = (now / 40) % 16;
  oled.drawCircle(64, cy, 24 + aura);
  if (aura > 8) oled.drawCircle(64, cy, aura - 8);
  oled.drawDisc(44, cy, r);
  oled.drawDisc(84, cy, r);
  oled.setDrawColor(0);
  oled.drawDisc(44, cy, r / 3);
  oled.drawDisc(84, cy, r / 3);
  oled.setDrawColor(1);
  // charge meter
  int w = (int)(charge * (OLED_W - 8));
  if (w > OLED_W - 8) w = OLED_W - 8;
  oled.drawFrame(4, OLED_H - 7, OLED_W - 8, 6);
  oled.drawBox(4, OLED_H - 7, w, 6);
  oled.sendBuffer();
}

static void faceThinking(uint32_t now) {
  oled.clearBuffer();
  const int cx = OLED_W / 2, cy = OLED_H / 2 - 4, R = 18;
  int lead = (now / 70) % 12;
  for (int i = 0; i < 12; ++i) {
    float a = (i / 12.0f) * 2.0f * PI;
    int x = cx + (int)(cosf(a) * R);
    int y = cy + (int)(sinf(a) * R);
    int d = (i == lead) ? 4 : ((i == (lead + 11) % 12) ? 3 : 1);
    oled.drawDisc(x, y, d);
  }
  static const char* MUSE[] = {"consulting the void", "asking the wifi...",
                               "bribing fate", "shaking my 8-ball",
                               "googling it", "channeling spirits"};
  drawCenteredSmall(MUSE[(now / 600) % 6], OLED_H - 3);
  oled.sendBuffer();
}

// Steady reveal frame (the entrance zoom is handled by playReveal()).
static void drawReveal() {
  oled.clearBuffer();
  switch (mode) {
    case Mode::Option:
      drawDie(OLED_W / 2, 26, 20, dieN);
      drawCenteredSmall(flavor, OLED_H - 3);
      break;
    case Mode::YesNo: {
      // Expressive face up top, verdict word below.
      int fcy = 16;
      drawEyes(OLED_W / 2, fcy, 5, revealTone > 0);
      if (revealTone > 0) drawMouth(OLED_W / 2, fcy + 12, 12, 5, true);
      else if (revealTone < 0) drawMouth(OLED_W / 2, fcy + 16, 12, 5, false);
      else oled.drawHLine(OLED_W / 2 - 10, fcy + 14, 20);
      drawWord(verdict, 50);
      drawCenteredSmall(flavor, OLED_H - 2);
      break;
    }
  }
  oled.sendBuffer();
}

// =====================================================================
//  Jingles ==============================================================
// =====================================================================
static void jingleWake() {
  static const Note n[] = {{660, 50}, {880, 70}};
  playMelody(n, 2);
}
// Original arcade-hype chiptune in a fast, bouncy endless-runner spirit
// (an homage, not a transcription of any copyrighted track). Plays as the
// boot theme and as the megaShake reward. 🛹
static void jingleSubway() {
  static const Note n[] = {
      // bouncy hook
      {659, 110}, {659, 90}, {784, 120}, {880, 150}, {784, 90}, {659, 170}, {0, 50},
      // answer phrase
      {587, 110}, {659, 110}, {784, 150}, {659, 100}, {587, 170}, {0, 50},
      // lift + run-up
      {880, 110}, {988, 110}, {880, 110}, {784, 140}, {659, 110}, {784, 130},
      // landing
      {988, 120}, {1175, 150}, {988, 110}, {784, 110}, {659, 240},
  };
  playMelody(n, sizeof(n) / sizeof(n[0]));
}
static void jingleByTone(int8_t tone) {
  switch (tone) {
    case 2: {  // triumphant fanfare
      static const Note n[] = {{523, 90}, {659, 90}, {784, 90}, {1047, 130}, {1319, 220}};
      playMelody(n, 5);
      break;
    }
    case 1: {  // bright ascending
      static const Note n[] = {{523, 110}, {659, 110}, {784, 110}, {1047, 200}};
      playMelody(n, 4);
      break;
    }
    case 0: {  // wobbly shrug
      static const Note n[] = {{700, 120}, {500, 120}, {700, 120}, {500, 200}};
      playMelody(n, 4);
      break;
    }
    case -1: {  // descending
      static const Note n[] = {{494, 130}, {392, 130}, {294, 240}};
      playMelody(n, 3);
      break;
    }
    default: {  // -2 glum "wah-wah"
      static const Note n[] = {{392, 160}, {370, 160}, {294, 360}};
      playMelody(n, 3);
      break;
    }
  }
}

// =====================================================================
//  Reveal entrance animations ==========================================
// =====================================================================
static void animateOption() {
  // A rolling die (1..OPTION_COUNT) that decelerates with a blip per
  // tumble, then lands on the chosen option.
  uint16_t step = 40;
  for (uint8_t i = 0; i < 14; ++i) {
    uint8_t face = 1 + rngBelow(OPTION_COUNT);
    oled.clearBuffer();
    drawDie(OLED_W / 2, 26, 20, face);
    drawCenteredSmall("rolling the dice...", OLED_H - 3);
    oled.sendBuffer();
    buzz(900 + face * 60);
    delay(step);
    silence();
    step += 12;  // decelerate
  }
}
static void animateWordPop() {
  // Quick swoop up in pitch while the word grows into place.
  for (uint8_t i = 0; i < 4; ++i) {
    oled.clearBuffer();
    oled.setFont(WORD_FONTS[3 - i]);
    int w = oled.getStrWidth(verdict);
    if (w > OLED_W - 4) w = OLED_W - 4;
    oled.drawStr((OLED_W - oled.getStrWidth(verdict)) / 2, 40, verdict);
    oled.sendBuffer();
    buzz(500 + i * 200);
    delay(45);
  }
  silence();
}

// =====================================================================
//  Decide ===============================================================
// =====================================================================
static void classifyAndDecide() {
  // Fold fresh timing entropy into the PRNG each time.
  rngState ^= micros() * 2654435761u;
  if (rngState == 0) rngState = 0xA5A5A5A5u;

  megaShake = shakeEnergy > MEGA_ENERGY;

  // Two gestures only: up-and-down (vertical accel dominates) = pick an
  // option 1..4; anything else = a yes/no shake.
  mode = (peakVert > peakHoriz) ? Mode::Option : Mode::YesNo;

  flavor[0] = '\0';
  switch (mode) {
    case Mode::YesNo: {
      const Answer* a;
      bool emphatic = shakeEnergy > EMPHATIC_ENERGY;
      if (rngBelow(100) < NEUTRAL_PCT) {
        a = &pick(NEUTRAL);
      } else if (rngBelow(100) < YESNO_SPLIT) {
        a = emphatic ? &pick(YES_STRONG) : &pick(YES_SOFT);
      } else {
        a = emphatic ? &pick(NO_STRONG) : &pick(NO_SOFT);
      }
      snprintf(verdict, sizeof(verdict), "%s", a->big);
      snprintf(flavor, sizeof(flavor), "%s", a->sub);
      revealTone = a->tone;
      break;
    }
    case Mode::Option: {
      dieN = 1 + rngBelow(OPTION_COUNT);
      static const char* QUIPS[] = {"go with option %u", "option %u, trust me 8)",
                                    "the dice gods say %u", "lucky number %u ^_^"};
      snprintf(verdict, sizeof(verdict), "%u", dieN);
      snprintf(flavor, sizeof(flavor), QUIPS[rngBelow(4)], dieN);
      revealTone = 1;
      break;
    }
  }
}

static void playReveal() {
  if (mode == Mode::Option) animateOption();
  else animateWordPop();
  jingleByTone(revealTone);

  // Easter egg: if you shook like your life depended on it, SURF'S UP. 🛹
  if (megaShake) {
    oled.clearBuffer();
    drawWord("SURF'S UP", 38);
    drawCenteredSmall("\\o/ you went HAM \\o/", OLED_H - 4);
    oled.sendBuffer();
    jingleSubway();
  }
}

// =====================================================================
//  Lifecycle ============================================================
// =====================================================================
static void enter(State s, uint32_t now) {
  state = s;
  stateSince = now;
}
static void resetShake() {
  peakVert = peakHoriz = 0;
  shakeEnergy = 0;
  calmSince = 0;
  megaShake = false;
}

void setup() {
  Serial.begin(115200);
  delay(400);

  oled.setBusClock(400000);
  oled.begin();
  oled.clearBuffer();
  drawWord("ORACLE", 36);
  drawCenteredSmall("wake the spirits o_O", OLED_H - 4);
  oled.sendBuffer();

  ledcAttach(PIN_BUZZER, 2000, 10);
  silence();

  Serial.println("🔮 oracle: booting...");
  if (!mpu::begin()) {
    Serial.println("💀 oracle: MPU not found — check SDA=GPIO7 / SCL=GPIO8 / VCC / GND");
    oled.clearBuffer();
    drawCenteredSmall("no MPU on GPIO7/8 :(", 28);
    drawCenteredSmall("check wiring", 44);
    oled.sendBuffer();
    while (true) delay(1000);
  }
  Serial.printf("🔮 oracle: MPU @ 0x%02X — ready!\n", mpu::address());
  Serial.println("   side-to-side = yes/no 🎱   up-down = option 1-4 🎲   shake HARD = 🛹");
  jingleSubway();  // boot theme 🛹
  delay(300);
  enter(State::Sleeping, millis());
}

void loop() {
  static uint32_t lastSample = 0;
  uint32_t now = millis();
  if (now - lastSample < 12) return;  // ~80 Hz
  lastSample = now;

  mpu::Sample s;
  if (!mpu::read(s)) return;

  // Bias-correct the gyro (offset learned while sleeping).
  float cgx = s.gx - biasGx, cgy = s.gy - biasGy, cgz = s.gz - biasGz;

  // Gravity tracking → linear (kinetic) accel.
  if (!gSeeded) {
    gX = s.ax; gY = s.ay; gZ = s.az;
    gSeeded = true;
  } else {
    gX += GRAVITY_ALPHA * (s.ax - gX);
    gY += GRAVITY_ALPHA * (s.ay - gY);
    gZ += GRAVITY_ALPHA * (s.az - gZ);
  }
  float lx = s.ax - gX, ly = s.ay - gY, lz = s.az - gZ;
  float linMag = sqrtf(lx * lx + ly * ly + lz * lz);
  float gyroMag = sqrtf(cgx * cgx + cgy * cgy + cgz * cgz);

  // Live readout (~4 Hz) for tuning on `make monitor`.
  static uint32_t lastLog = 0;
  if (now - lastLog >= 250) {
    lastLog = now;
    Serial.printf("ORACLE state=%d lin=%.3f gyro=%.1f wake=%.2f\n",
                  (int)state, (double)linMag, (double)gyroMag, (double)WAKE_LIN_G);
  }

  switch (state) {
    case State::Sleeping: {
      faceSleeping(now);
      // While calm, slowly learn the gyro bias.
      if (gyroMag < 12.0f && linMag < 0.08f) {
        biasGx += 0.01f * (s.gx - biasGx);
        biasGy += 0.01f * (s.gy - biasGy);
        biasGz += 0.01f * (s.gz - biasGz);
      }
      if (linMag > WAKE_LIN_G) {
        rngState ^= (uint32_t)(linMag * 100000.0f) ^ (micros() << 3) ^
                    (uint32_t)(gyroMag * 1000.0f);
        if (rngState == 0) rngState = 0xA5A5A5A5u;
        resetShake();
        jingleWake();
        enter(State::Charging, now);
      }
      break;
    }
    case State::Charging: {
      shakeEnergy += linMag;
      float gmag = sqrtf(gX * gX + gY * gY + gZ * gZ);
      if (gmag < 1e-3f) gmag = 1.0f;
      float ux = gX / gmag, uy = gY / gmag, uz = gZ / gmag;
      float vLin = lx * ux + ly * uy + lz * uz;
      float hx = lx - vLin * ux, hy = ly - vLin * uy, hz = lz - vLin * uz;
      float hLin = sqrtf(hx * hx + hy * hy + hz * hz);
      if (fabsf(vLin) > peakVert) peakVert = fabsf(vLin);
      if (hLin > peakHoriz) peakHoriz = hLin;

      float intensity = linMag / 1.2f;
      if (intensity > 1.0f) intensity = 1.0f;
      float charge = shakeEnergy / CHARGE_FULL;
      if (charge > 1.0f) charge = 1.0f;
      faceCharging(now, intensity, charge);
      buzz(400 + (uint16_t)(intensity * 900.0f));

      if (linMag < CALM_LIN_G) {
        if (calmSince == 0) calmSince = now;
        if (now - calmSince >= CALM_HOLD_MS) {
          silence();
          classifyAndDecide();
          thinkDuration = THINK_MIN_MS + (rng() % THINK_RAND_MS);
          Serial.printf("oracle: %s verdict=%s | vert=%.2f horiz=%.2f energy=%.1f%s\n",
                        mode == Mode::Option ? "OPTION(up-down)" : "YES/NO(side)",
                        verdict, (double)peakVert, (double)peakHoriz,
                        (double)shakeEnergy, megaShake ? " [MEGA!]" : "");
          enter(State::Thinking, now);
        }
      } else {
        calmSince = 0;
      }
      break;
    }
    case State::Thinking: {
      faceThinking(now);
      // suspense ticks
      static uint32_t lastTick = 0;
      if (now - lastTick > 220) {
        lastTick = now;
        buzz(1500);
        delay(8);
        silence();
      }
      if (now - stateSince >= thinkDuration) {
        playReveal();  // blocking entrance animation + jingle
        enter(State::Reveal, now);
      }
      break;
    }
    case State::Reveal: {
      drawReveal();
      if (now - stateSince >= REVEAL_MS) {
        gSeeded = false;  // re-seed gravity for the next round
        enter(State::Sleeping, now);
      }
      break;
    }
  }
}
