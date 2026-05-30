// oracle.ino — standalone "shake-to-divine" fortune oracle.
//
// A self-contained sketch (no gochi daemon, no host tether): shake the
// device to wake it, the *kind* of shake picks the question mode, the
// energy you put in seeds the randomness, and after a suspenseful beat it
// reveals a verdict on the OLED with a themed buzzer jingle.
//
//   side-to-side  → YES / NO          (shake hard for an emphatic verdict)
//   up-and-down   → a number 1–6
//   twist / spin  → option A / B / C
//
// Hardware (all from src/config.h — nothing is hard-coded here):
//   OLED  SSD1306 128x64 on hardware I2C  SDA=GPIO5  SCL=GPIO6  (U8g2)
//   MPU-6050 on a *bit-banged* I2C bus     SDA=GPIO7  SCL=GPIO8
//   Passive piezo buzzer on               PIN_BUZZER=GPIO10  (LEDC)
//
// Why bit-bang the MPU: the C3 has a single hardware I2C controller and
// U8g2 owns it for the OLED, so the IMU rides its own software bus. This
// is the same split the main firmware uses (see src/imu/mpu6050.cpp); the
// driver below is a trimmed inline copy because arduino-cli only compiles
// sources that live in the sketch folder.
//
// NOTE: the SuperMini's onboard LED is GPIO8 — the same pin as the MPU's
// SCL line — so this oracle deliberately leaves the LED alone and puts all
// its visual drama on the OLED. (To use the LED you'd have to move the MPU
// off GPIO8 and rewire it; not worth it here.)
//
// Build:  arduino-cli compile --profile c3   (or `make test-oracle`)
// Flash:  arduino-cli upload  --profile c3 --port /dev/cu.usbmodem<…>
//
// Tuning (watch live values on the Serial Monitor @ 115200):
//   WAKE_LIN_G        — how hard a nudge wakes it (raise if it wakes too easily)
//   EMPHATIC_ENERGY   — shake energy above which yes/no becomes "YES!!"/"NO WAY"
//   YESNO_SPLIT       — 0..99 cutoff for yes vs no (53 = slightly yes-biased)

#include <Arduino.h>
#include <U8g2lib.h>
#include <math.h>

#include "../../src/config.h"

// =====================================================================
//  Display — identical constructor to the main firmware / oled test, so
//  the ROTATED_DISPLAY build switch behaves the same here.
// =====================================================================
#ifdef ROTATED_DISPLAY
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R2, U8X8_PIN_NONE, PIN_SCL, PIN_SDA);
#else
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, PIN_SCL, PIN_SDA);
#endif

// =====================================================================
//  Types — declared up front. The Arduino .ino preprocessor injects
//  auto-generated function prototypes *above the first function*, so any
//  type named in a function signature must already be visible here.
// =====================================================================
struct Note {
  uint16_t freq;
  uint16_t ms;
};
enum class State : uint8_t { Sleeping, Charging, Thinking, Reveal };
enum class Mode : uint8_t { YesNo, Number, Letter };

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
  bool ok = i2cWriteByte((MPU_ADDR << 1) | 0) && i2cWriteByte(reg) && i2cWriteByte(value);
  i2cStop();
  return ok;
}

bool readReg(uint8_t reg, uint8_t* out) {
  i2cStart();
  if (!i2cWriteByte((MPU_ADDR << 1) | 0) || !i2cWriteByte(reg)) {
    i2cStop();
    return false;
  }
  i2cStart();  // repeated start
  if (!i2cWriteByte((MPU_ADDR << 1) | 1)) {
    i2cStop();
    return false;
  }
  *out = i2cReadByte(/*sendAck=*/false);
  i2cStop();
  return true;
}

bool readBurst(uint8_t reg, uint8_t* buf, size_t n) {
  i2cStart();
  if (!i2cWriteByte((MPU_ADDR << 1) | 0) || !i2cWriteByte(reg)) {
    i2cStop();
    return false;
  }
  i2cStart();
  if (!i2cWriteByte((MPU_ADDR << 1) | 1)) {
    i2cStop();
    return false;
  }
  for (size_t i = 0; i < n; ++i) buf[i] = i2cReadByte(/*sendAck=*/i < n - 1);
  i2cStop();
  return true;
}
}  // namespace

bool begin() {
  busBringUp();
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
//  Oracle state machine
// =====================================================================
// --- Tuning dials -----------------------------------------------------
static const float WAKE_LIN_G = 0.45f;      // kinetic accel that wakes it
static const float CALM_LIN_G = 0.18f;      // below this = "shaking stopped"
static const uint32_t CALM_HOLD_MS = 380;   // calm must persist this long
static const uint32_t THINK_MS = 1300;      // suspense beat before reveal
static const uint32_t REVEAL_MS = 4000;     // how long the verdict shows
static const float EMPHATIC_ENERGY = 22.0f; // shake energy → emphatic yes/no
static const uint8_t YESNO_SPLIT = 53;      // <split = yes, slight yes bias
static const uint8_t ASK_AGAIN_PCT = 6;     // % chance of a cop-out reply

// --- Runtime state ----------------------------------------------------
static State state = State::Sleeping;
static uint32_t stateSince = 0;

// Gravity low-pass (per axis) — lets us find which axis points "down" so
// gesture classification works at any orientation. Same idea as motion.cpp.
static float gX = 0, gY = 0, gZ = 1;
static bool gSeeded = false;
static const float GRAVITY_ALPHA = 0.02f;

// Per-shake accumulators (reset on wake).
static float energyX = 0, energyY = 0, energyZ = 0;  // Σ|linear accel| per axis
static float energyGyro = 0;                         // Σ|gyro|
static float shakeEnergy = 0;                        // Σ|linear| total (intensity)
static uint32_t calmSince = 0;

// xorshift32 PRNG, seeded from gyro/timing entropy at wake.
static uint32_t rngState = 0x1234567u;
static uint32_t rng() {
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return rngState;
}
static uint8_t rngBelow(uint8_t n) { return (uint8_t)(rng() % n); }

// --- Reveal payload ---------------------------------------------------
static Mode mode = Mode::YesNo;
static char verdict[16] = "";   // big centered text
static char flavor[20] = "";    // small line under it
static bool revealPositive = true;

// =====================================================================
//  Faces / animations on the OLED
// =====================================================================
static void faceSleeping(uint32_t now) {
  oled.clearBuffer();
  // Breathing: closed eyes (thin lines) that drift up/down slowly.
  float breath = sinf(now / 900.0f);
  int yoff = (int)(breath * 2.0f);
  int cy = OLED_H / 2 + yoff;
  oled.drawHLine(34, cy, 22);   // left closed eye
  oled.drawHLine(72, cy, 22);   // right closed eye
  oled.setFont(u8g2_font_ncenB08_tr);
  const char* z = "z z z";
  oled.drawStr((OLED_W - oled.getStrWidth(z)) / 2, OLED_H - 6, z);
  oled.sendBuffer();
}

static void faceCharging(uint32_t now, float intensity) {
  oled.clearBuffer();
  // Eyes snap open and grow with intensity (0..1). Jitter with energy.
  int r = 6 + (int)(intensity * 12.0f);
  if (r > 18) r = 18;
  int jitter = (intensity > 0.5f) ? (int)(rng() % 5) - 2 : 0;
  int cy = OLED_H / 2 + jitter;
  oled.drawDisc(44, cy, r);
  oled.drawDisc(84, cy, r);
  // White pupils so the eyes read as "wide awake", not solid blobs.
  oled.setDrawColor(0);
  oled.drawDisc(44, cy, r / 3);
  oled.drawDisc(84, cy, r / 3);
  oled.setDrawColor(1);
  oled.sendBuffer();
}

static void faceThinking(uint32_t now) {
  oled.clearBuffer();
  // A ring of dots, one bright "comet" sweeping round = "consulting…".
  const int cx = OLED_W / 2, cy = OLED_H / 2 - 4, R = 18;
  int lead = (now / 70) % 12;
  for (int i = 0; i < 12; ++i) {
    float a = (i / 12.0f) * 2.0f * PI;
    int x = cx + (int)(cosf(a) * R);
    int y = cy + (int)(sinf(a) * R);
    int d = (i == lead) ? 4 : ((i == (lead + 11) % 12) ? 3 : 1);
    oled.drawDisc(x, y, d);
  }
  oled.setFont(u8g2_font_ncenB08_tr);
  const char* t = "consulting...";
  oled.drawStr((OLED_W - oled.getStrWidth(t)) / 2, OLED_H - 3, t);
  oled.sendBuffer();
}

static void drawReveal() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_ncenB24_tr);
  oled.setFontPosCenter();
  int vw = oled.getStrWidth(verdict);
  oled.drawStr((OLED_W - vw) / 2, OLED_H / 2 - 6, verdict);
  oled.setFontPosBaseline();
  if (flavor[0]) {
    oled.setFont(u8g2_font_ncenB08_tr);
    oled.drawStr((OLED_W - oled.getStrWidth(flavor)) / 2, OLED_H - 4, flavor);
  }
  oled.sendBuffer();
}

// =====================================================================
//  Jingles
// =====================================================================
static void jingleWake() {
  static const Note n[] = {{660, 60}, {880, 90}};
  playMelody(n, 2);
}
static void jingleYes() {  // ascending, bright
  static const Note n[] = {{523, 110}, {659, 110}, {784, 110}, {1047, 220}};
  playMelody(n, 4);
}
static void jingleNo() {  // descending, glum
  static const Note n[] = {{494, 130}, {392, 130}, {294, 260}};
  playMelody(n, 3);
}
static void jingleNeutral() {  // for numbers / letters / ask-again
  static const Note n[] = {{660, 90}, {660, 90}, {880, 180}};
  playMelody(n, 3);
}

// =====================================================================
//  Mode classification + verdict selection
// =====================================================================
static void classifyAndDecide() {
  // Which axis is vertical right now? The one with the largest gravity
  // component. Up-and-down shakes dump their energy onto that axis.
  float agx = fabsf(gX), agy = fabsf(gY), agz = fabsf(gZ);
  int vertAxis = 0;  // 0=x 1=y 2=z
  if (agy >= agx && agy >= agz) vertAxis = 1;
  else if (agz >= agx && agz >= agy) vertAxis = 2;

  float vertEnergy = (vertAxis == 0) ? energyX : (vertAxis == 1) ? energyY : energyZ;
  float horizEnergy = (energyX + energyY + energyZ) - vertEnergy;

  // Rotation-dominant motion (a twist/spin) shows up far more in the
  // gyro than in linear accel. Scale gyro down (°/s are large numbers)
  // before comparing against the linear-accel energy.
  float rotScore = energyGyro * 0.02f;
  float linScore = energyX + energyY + energyZ;

  if (rotScore > linScore * 0.9f) {
    mode = Mode::Letter;
  } else if (vertEnergy > horizEnergy) {
    mode = Mode::Number;
  } else {
    mode = Mode::YesNo;
  }

  flavor[0] = '\0';
  switch (mode) {
    case Mode::YesNo: {
      uint8_t roll = rngBelow(100);
      if (rngBelow(100) < ASK_AGAIN_PCT) {
        snprintf(verdict, sizeof(verdict), "HMM");
        snprintf(flavor, sizeof(flavor), "ask again");
        revealPositive = true;  // neutral jingle handled below
        mode = Mode::Letter;    // route to neutral jingle
        return;
      }
      bool yes = roll < YESNO_SPLIT;
      bool emphatic = shakeEnergy > EMPHATIC_ENERGY;
      revealPositive = yes;
      if (yes) {
        snprintf(verdict, sizeof(verdict), emphatic ? "YES!!" : "yes");
        snprintf(flavor, sizeof(flavor), emphatic ? "no doubt" : "the stars say so");
      } else {
        snprintf(verdict, sizeof(verdict), emphatic ? "NO WAY" : "no");
        snprintf(flavor, sizeof(flavor), emphatic ? "not a chance" : "don't count on it");
      }
      break;
    }
    case Mode::Number: {
      uint8_t d = 1 + rngBelow(6);
      snprintf(verdict, sizeof(verdict), "%u", d);
      snprintf(flavor, sizeof(flavor), "a number 1-6");
      break;
    }
    case Mode::Letter: {
      char c = (char)('A' + rngBelow(3));
      snprintf(verdict, sizeof(verdict), "%c", c);
      snprintf(flavor, sizeof(flavor), "option %c", c);
      break;
    }
  }
}

static void playRevealJingle() {
  if (mode == Mode::YesNo) {
    revealPositive ? jingleYes() : jingleNo();
  } else {
    jingleNeutral();
  }
}

// =====================================================================
//  Lifecycle
// =====================================================================
static void enter(State s, uint32_t now) {
  state = s;
  stateSince = now;
}

static void resetShake() {
  energyX = energyY = energyZ = 0;
  energyGyro = 0;
  shakeEnergy = 0;
  calmSince = 0;
}

void setup() {
  Serial.begin(115200);
  delay(400);  // let USB CDC enumerate before the first log line

  oled.setBusClock(400000);
  oled.begin();
  oled.setFont(u8g2_font_ncenB14_tr);
  oled.clearBuffer();
  const char* boot = "oracle";
  oled.drawStr((OLED_W - oled.getStrWidth(boot)) / 2, OLED_H / 2 + 5, boot);
  oled.sendBuffer();

  ledcAttach(PIN_BUZZER, 2000, 10);
  silence();

  Serial.println("oracle: booting");
  if (!mpu::begin()) {
    Serial.println("oracle: MPU not found — check SDA=GPIO7 / SCL=GPIO8 / VCC / GND");
    oled.clearBuffer();
    oled.setFont(u8g2_font_ncenB08_tr);
    oled.drawStr(4, 28, "no MPU on GPIO7/8");
    oled.drawStr(4, 44, "check wiring");
    oled.sendBuffer();
    while (true) delay(1000);
  }
  Serial.println("oracle: ready — shake to divine. fields: ORACLE,linMag,energy,state");
  delay(600);
  enter(State::Sleeping, millis());
}

void loop() {
  static uint32_t lastSample = 0;
  uint32_t now = millis();
  if (now - lastSample < 12) return;  // ~80 Hz
  lastSample = now;

  mpu::Sample s;
  if (!mpu::read(s)) return;

  // Track gravity, derive linear (kinetic) accel — orientation-robust.
  if (!gSeeded) {
    gX = s.ax;
    gY = s.ay;
    gZ = s.az;
    gSeeded = true;
  } else {
    gX += GRAVITY_ALPHA * (s.ax - gX);
    gY += GRAVITY_ALPHA * (s.ay - gY);
    gZ += GRAVITY_ALPHA * (s.az - gZ);
  }
  float lx = s.ax - gX, ly = s.ay - gY, lz = s.az - gZ;
  float linMag = sqrtf(lx * lx + ly * ly + lz * lz);
  float gyroMag = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);

  switch (state) {
    case State::Sleeping: {
      faceSleeping(now);
      if (linMag > WAKE_LIN_G) {
        // Seed the PRNG from whatever entropy the wake motion carries.
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
      // Accumulate per-axis energy + intensity; rising buzzer pitch.
      energyX += fabsf(lx);
      energyY += fabsf(ly);
      energyZ += fabsf(lz);
      energyGyro += gyroMag;
      shakeEnergy += linMag;
      float intensity = linMag / 1.2f;
      if (intensity > 1.0f) intensity = 1.0f;
      faceCharging(now, intensity);
      buzz(400 + (uint16_t)(intensity * 900.0f));  // 400..1300 Hz

      if (linMag < CALM_LIN_G) {
        if (calmSince == 0) calmSince = now;
        if (now - calmSince >= CALM_HOLD_MS) {
          silence();
          classifyAndDecide();
          Serial.printf("oracle: decided mode=%d verdict=%s energy=%.1f\n",
                        (int)mode, verdict, shakeEnergy);
          enter(State::Thinking, now);
        }
      } else {
        calmSince = 0;
      }
      break;
    }
    case State::Thinking: {
      faceThinking(now);
      if (now - stateSince >= THINK_MS) {
        drawReveal();
        playRevealJingle();
        enter(State::Reveal, now);
      }
      break;
    }
    case State::Reveal: {
      drawReveal();  // static, but cheap to re-send
      if (now - stateSince >= REVEAL_MS) {
        gSeeded = false;  // re-seed gravity for the next round
        enter(State::Sleeping, now);
      }
      break;
    }
  }
}
