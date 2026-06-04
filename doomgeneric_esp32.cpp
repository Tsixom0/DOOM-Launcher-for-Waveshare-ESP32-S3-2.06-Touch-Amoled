#include <Arduino.h>
#include <Wire.h>
#include <memory>
#include <math.h>

#include "pin_config.h"

#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "esp_heap_caps.h"

#include "ESP_I2S.h"
#include "esp_check.h"
#include "es8311.h"
#include "XPowersLib.h"

#ifndef DG_DEBUG_LOGS
#define DG_DEBUG_LOGS 1
#endif

#if DG_DEBUG_LOGS
#define DG_LOG(...) Serial.printf(__VA_ARGS__)
#else
#define DG_LOG(...) do {} while (0)
#endif

extern "C" {
  #include <doomgeneric.h>
  #include <doomkeys.h>
  #include <i_sound.h>
  #include <w_wad.h>
  #include <z_zone.h>

  // Chocolate/doomgeneric sound config globals.
  // i_sound.c references these when FEATURE_SOUND is enabled.
  // We do not use libsamplerate on ESP32, so keep it disabled.
  int use_libsamplerate = 0;
  float libsamplerate_scale = 0.65f;

  // Runtime settings supplied by DoomWatch_Run.ino.
  extern int dg_audio_volume;
  extern int dg_battery_hud_enabled;
  extern int dg_control_sensitivity;
  extern int dg_screen_brightness;
}

#if DOOMGENERIC_RESX != 320
  #error "Edit Documents/Arduino/libraries/doomgeneric/src/doomgeneric.h and set DOOMGENERIC_RESX to 320"
#endif

#if DOOMGENERIC_RESY != 200
  #error "Edit Documents/Arduino/libraries/doomgeneric/src/doomgeneric.h and set DOOMGENERIC_RESY to 200"
#endif

#ifndef KEY_RIGHTARROW
#define KEY_RIGHTARROW 0xae
#endif
#ifndef KEY_LEFTARROW
#define KEY_LEFTARROW 0xac
#endif
#ifndef KEY_UPARROW
#define KEY_UPARROW 0xad
#endif
#ifndef KEY_DOWNARROW
#define KEY_DOWNARROW 0xaf
#endif
#ifndef KEY_ESCAPE
#define KEY_ESCAPE 27
#endif
#ifndef KEY_ENTER
#define KEY_ENTER 13
#endif
#ifndef KEY_SPACE
#define KEY_SPACE 32
#endif
#ifndef KEY_FIRE
#define KEY_FIRE 0x80
#endif
#ifndef KEY_USE
#define KEY_USE 0x81
#endif

// ---------------- Display ----------------
static Arduino_DataBus *bus = nullptr;
static Arduino_CO5300 *gfx = nullptr;

// Optional shared display from the WAD picker .ino.
// If the picker already initialized CO5300, DG_Init reuses it instead of
// calling Arduino_ESP32QSPI::begin() a second time.
extern Arduino_CO5300 *g_sharedGfx;
extern bool g_sharedDisplayReady;

#define DOOM_W DOOMGENERIC_RESX
#define DOOM_H DOOMGENERIC_RESY

// RGB565 wide mode:
// DOOM renders internally at 320x200 RGB565, then we scale to 410x256.
// This fills the screen width while keeping pixel workload close to 400x250.
#define VIEW_W LCD_WIDTH
#define VIEW_H 256
#define VIEW_X 0
#define VIEW_Y ((LCD_HEIGHT - VIEW_H) / 2)

static uint16_t *frame565 = nullptr;
static uint16_t scaleXMap[VIEW_W];
static uint16_t scaleYMap[VIEW_H];
static bool scaleMapsReady = false;

// ---------------- Touch ----------------
static std::shared_ptr<Arduino_IIC_DriveBus> IICBus;
static std::unique_ptr<Arduino_IIC> touch;
static bool touchReady = false;
static volatile bool touchIRQ = false;

static void touchInterrupt() {
  touchIRQ = true;
  if (touch) {
    touch->IIC_Interrupt_Flag = true;
  }
}

// ---------------- IMU / QMI8658 ----------------
#ifndef QMI8658_L_SLAVE_ADDRESS
#define QMI8658_L_SLAVE_ADDRESS 0x6B
#endif

#define QMI_ADDR QMI8658_L_SLAVE_ADDRESS

#define QMI_REG_WHO_AM_I 0x00
#define QMI_REG_REVISION 0x01
#define QMI_REG_CTRL1    0x02
#define QMI_REG_CTRL2    0x03
#define QMI_REG_CTRL3    0x04
#define QMI_REG_CTRL5    0x06
#define QMI_REG_CTRL7    0x08
#define QMI_REG_ACC_X_L  0x35

#define QMI_CTRL1_AUTO_INC 0x40
#define QMI_ACC_RANGE_4G   0x10
#define QMI_ACC_ODR_125HZ  0x06
#define QMI_GYR_RANGE_512  0x50
#define QMI_GYR_ODR_125HZ  0x06
#define QMI_CTRL7_ACC_EN   0x01
#define QMI_CTRL7_GYR_EN   0x02

#define ACC_LSB_PER_G      8192.0f

// Tune these after testing in-game.
#define IMU_DEADZONE_G       0.13f

// Asymmetric tilt triggers tuned for wrist use.
// Your mapping:
// forward  = X-
// backward = X+
// left     = Y+
// right    = Y-
#define IMU_TRIGGER_FWD_G   0.26f
#define IMU_TRIGGER_BACK_G  0.18f
#define IMU_TRIGGER_TURN_G  0.19f

#define IMU_STRONG_G        0.45f
#define IMU_FILTER_ALPHA    0.28f

static bool imuReady = false;
static float imuBaseX = 0.0f;
static float imuBaseY = 0.0f;
static float imuBaseZ = 0.0f;
static float imuFiltX = 0.0f;
static float imuFiltY = 0.0f;
static float imuFiltZ = 0.0f;

static float imuDX = 0.0f;
static float imuDY = 0.0f;
static float imuDZ = 0.0f;

// ---------------- Colors ----------------
static uint16_t C_BLACK;
static uint16_t C_WHITE;
static uint16_t C_GRAY;
static uint16_t C_DARK;
static uint16_t C_RED;
static uint16_t C_GREEN;
static uint16_t C_BLUE;
static uint16_t C_ORANGE;
static uint16_t C_PURPLE;
static uint16_t C_CYAN;
static uint16_t C_YELLOW;
static uint16_t C_CARD;
static uint16_t C_CARD2;
static uint16_t C_LINE;
static uint16_t C_MUTED;
static uint16_t C_ACCENT;
static uint16_t C_ACCENT2;

static uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  if (!gfx) return 0;
  return gfx->color565(r, g, b);
}

static void initColors() {
  // Same warm OLED UI language as the launcher, but kept very subtle in-game.
  C_BLACK   = rgb(0, 0, 0);
  C_WHITE   = rgb(255, 250, 238);
  C_GRAY    = rgb(145, 132, 112);
  C_DARK    = rgb(20, 17, 13);
  C_RED     = rgb(255, 88, 82);
  C_GREEN   = rgb(86, 228, 123);
  C_BLUE    = rgb(40, 80, 220);
  C_ORANGE  = rgb(234, 108, 55);
  C_PURPLE  = rgb(170, 80, 255);
  C_CYAN    = rgb(0, 218, 255);
  C_YELLOW  = rgb(255, 213, 86);
  C_CARD    = rgb(16, 13, 9);
  C_CARD2   = rgb(32, 25, 18);
  C_LINE    = rgb(72, 56, 38);
  C_MUTED   = rgb(171, 151, 121);
  C_ACCENT  = C_ORANGE;
  C_ACCENT2 = rgb(255, 185, 75);
}

// ---------------- Key queue ----------------
struct KeyEvent {
  bool pressed;
  unsigned char key;
};

static const int KEY_QUEUE_SIZE = 32;
static KeyEvent keyQueue[KEY_QUEUE_SIZE];
static int keyHead = 0;
static int keyTail = 0;

static bool keyQueueEmpty() {
  return keyHead == keyTail;
}

static bool pushKey(bool pressed, unsigned char key) {
  int next = (keyHead + 1) % KEY_QUEUE_SIZE;

  if (next == keyTail) {
    return false;
  }

  keyQueue[keyHead].pressed = pressed;
  keyQueue[keyHead].key = key;
  keyHead = next;

  return true;
}

static bool popKey(int *pressed, unsigned char *key) {
  if (keyQueueEmpty()) {
    return false;
  }

  *pressed = keyQueue[keyTail].pressed ? 1 : 0;
  *key = keyQueue[keyTail].key;

  keyTail = (keyTail + 1) % KEY_QUEUE_SIZE;

  return true;
}

static void pushKeyTap(unsigned char key) {
  pushKey(true, key);
  pushKey(false, key);
}

static void pushTextMacro(const char *text) {
  if (!text) {
    return;
  }

  while (*text) {
    pushKeyTap((unsigned char)*text);
    text++;
  }
}

static void injectSaveNameMacro() {
  // A short all-caps name is easiest to read and works in DOOM text entry.
  pushTextMacro("Tsixom");
  pushKeyTap(KEY_ENTER);
  Serial.println("[CTRL] Save-name macro injected: Tsixom + ENTER");
}

// Top-left weapon control: tap to cycle to the next weapon.
// No in-game overlay is drawn so FPS stays close to the clean renderer.
static const unsigned char WEAPON_CYCLE_KEYS[] = {'2', '1', '3', '4', '5', '6', '7'};
static const char *WEAPON_CYCLE_NAMES[] = {
  "PISTOL",
  "FIST",
  "SHOTGUN",
  "CHAINGUN",
  "ROCKET",
  "PLASMA",
  "BFG"
};

// DOOM starts on pistol by default.
static int weaponCycleIndex = 0;

// Weapon keys must stay pressed for a short time.
// A down+up in the same input drain can be missed by gameplay weapon logic.
static bool weaponKeyHeld = false;
static unsigned char weaponHeldKey = 0;
static uint32_t weaponReleaseAtMs = 0;
#define WEAPON_KEY_HOLD_MS 140

static int weaponCycleCount() {
  return sizeof(WEAPON_CYCLE_KEYS) / sizeof(WEAPON_CYCLE_KEYS[0]);
}

static void releaseHeldWeaponKeyNow() {
  if (weaponKeyHeld) {
    pushKey(false, weaponHeldKey);
    weaponKeyHeld = false;
  }
}

static void serviceWeaponKeyRelease() {
  if (weaponKeyHeld && millis() >= weaponReleaseAtMs) {
    pushKey(false, weaponHeldKey);
    weaponKeyHeld = false;
    Serial.printf("[CTRL] Weapon key released: %c\n", weaponHeldKey);
  }
}

static void queueWeaponIndex(int index) {
  const int count = weaponCycleCount();

  if (count <= 0) {
    return;
  }

  index = constrain(index, 0, count - 1);
  releaseHeldWeaponKeyNow();
  weaponCycleIndex = index;

  unsigned char key = WEAPON_CYCLE_KEYS[weaponCycleIndex];

  weaponHeldKey = key;
  weaponKeyHeld = true;
  weaponReleaseAtMs = millis() + WEAPON_KEY_HOLD_MS;

  // Press now; release happens after a short hold in serviceWeaponKeyRelease().
  pushKey(true, key);

  Serial.printf("[CTRL] Weapon selected: %s (%c)\n",
                WEAPON_CYCLE_NAMES[weaponCycleIndex],
                key);
}

static void queueNextWeaponCycleKey() {
  const int count = weaponCycleCount();

  if (count <= 0) {
    return;
  }

  int next = weaponCycleIndex + 1;
  if (next >= count) {
    next = 0;
  }

  queueWeaponIndex(next);
}

// ---------------- Control state ----------------
struct ControlState {
  bool forward;
  bool back;
  bool left;
  bool right;
  bool fire;
  bool use;
  bool menu;
  bool weapon;
  bool touching;
  int x;
  int y;
};

static ControlState ctrl;
static ControlState oldCtrl;

static uint32_t lastTouchSeenMs = 0;
static uint32_t lastFpsMs = 0;
static uint32_t framesThisSecond = 0;
static uint32_t lastControlPollMs = 0;

// Button idle levels are calibrated at startup.
// This lets PWR work even if its idle level is LOW on this board.
static int bootIdleLevel = HIGH;
static int pwrIdleLevel = HIGH;

// Battery / PMIC HUD.
static XPowersAXP2101 power;
static bool pmuReady = false;
static int lastBatteryPercent = -999;
static bool lastBatteryCharging = false;
static uint32_t lastBatteryDrawMs = 0;

#define SAFE_X 24
#define SAFE_Y 30
#define BATT_HUD_X SAFE_X
#define BATT_HUD_Y SAFE_Y
#define BATT_HUD_W 118
#define BATT_HUD_H 20

// Save-name helper:
// DOOM's save menu asks for a typed savegame name.
// We have no keyboard, so pressing BOOT + PWR together injects:
//   Tsixom + ENTER
// Use this only when the save slot name cursor is visible.
static uint32_t lastSaveNameMacroMs = 0;
#define SAVE_NAME_MACRO_COOLDOWN_MS 1500

// Startup control hints are drawn over the DOOM frame briefly,
// so users can discover the invisible corner controls.
static uint32_t gameplayStartedAtMs = 0;
#define GAMEPLAY_HINTS_MS 3600

// Invisible touch-drag joystick.
// Top corners are reserved:
//   top-left  = weapon
//   top-right = menu / escape
// Everything else becomes a temporary joystick origin on touch-down.
#define TOUCH_CORNER_SIZE_PX   90
#define TOUCH_JOY_FWD_PX       24
#define TOUCH_JOY_BACK_PX      22
#define TOUCH_JOY_TURN_PX      24
#define TOUCH_JOY_RELEASE_MS   140

static bool joyActive = false;
static int joyBaseX = -1;
static int joyBaseY = -1;
static int joyDX = 0;
static int joyDY = 0;

static int controlSensitivitySafe() {
  return constrain(dg_control_sensitivity, 0, 2);
}

static int joyFwdThreshold() {
  switch (controlSensitivitySafe()) {
    case 0: return 34;  // Calm
    case 2: return 16;  // Quick
    default: return 24; // Normal
  }
}

static int joyBackThreshold() {
  switch (controlSensitivitySafe()) {
    case 0: return 32;
    case 2: return 16;
    default: return 22;
  }
}

static int joyTurnThreshold(bool moving) {
  switch (controlSensitivitySafe()) {
    case 0: return moving ? 72 : 58;
    case 2: return moving ? 46 : 34;
    default: return moving ? 58 : 46;
  }
}

// ---------------- IMU helpers ----------------
static bool imuWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(QMI_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool imuReadRegs(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(QMI_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t got = Wire.requestFrom((uint8_t)QMI_ADDR, (uint8_t)len);
  if (got != len) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }

  return true;
}

static uint8_t imuReadReg(uint8_t reg) {
  uint8_t v = 0xFF;
  imuReadRegs(reg, &v, 1);
  return v;
}

static int16_t imuLE16(uint8_t lo, uint8_t hi) {
  return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}

static bool imuReadAccel(float &ax, float &ay, float &az) {
  uint8_t b[6];

  if (!imuReadRegs(QMI_REG_ACC_X_L, b, sizeof(b))) {
    return false;
  }

  int16_t axRaw = imuLE16(b[0], b[1]);
  int16_t ayRaw = imuLE16(b[2], b[3]);
  int16_t azRaw = imuLE16(b[4], b[5]);

  ax = axRaw / ACC_LSB_PER_G;
  ay = ayRaw / ACC_LSB_PER_G;
  az = azRaw / ACC_LSB_PER_G;

  return true;
}

static bool initIMU() {
  Serial.println("[DG] IMU init...");

  uint8_t who = imuReadReg(QMI_REG_WHO_AM_I);
  uint8_t rev = imuReadReg(QMI_REG_REVISION);

  Serial.printf("[DG] IMU WHO=0x%02X REV=0x%02X\n", who, rev);

  if (who == 0xFF || who == 0x00) {
    Serial.println("[DG] IMU not responding. Continuing without IMU.");
    imuReady = false;
    return false;
  }

  // Enable auto-increment.
  if (!imuWriteReg(QMI_REG_CTRL1, QMI_CTRL1_AUTO_INC)) {
    Serial.println("[DG] IMU CTRL1 failed");
    imuReady = false;
    return false;
  }

  // Accel: +/-4g @ 125Hz.
  if (!imuWriteReg(QMI_REG_CTRL2, QMI_ACC_RANGE_4G | QMI_ACC_ODR_125HZ)) {
    Serial.println("[DG] IMU CTRL2 failed");
    imuReady = false;
    return false;
  }

  // Gyro configured too, though v0.6 only uses accel.
  imuWriteReg(QMI_REG_CTRL3, QMI_GYR_RANGE_512 | QMI_GYR_ODR_125HZ);
  imuWriteReg(QMI_REG_CTRL5, 0x00);

  if (!imuWriteReg(QMI_REG_CTRL7, QMI_CTRL7_ACC_EN | QMI_CTRL7_GYR_EN)) {
    Serial.println("[DG] IMU CTRL7 failed");
    imuReady = false;
    return false;
  }

  delay(100);

  imuReady = true;
  Serial.println("[DG] IMU OK");
  return true;
}

static void calibrateIMU() {
  if (!imuReady) {
    return;
  }

  Serial.println("[DG] IMU calibration: hold normal playing position...");

  delay(700);

  const int samples = 80;
  float sx = 0.0f;
  float sy = 0.0f;
  float sz = 0.0f;
  int good = 0;

  for (int i = 0; i < samples; i++) {
    float ax, ay, az;

    if (imuReadAccel(ax, ay, az)) {
      sx += ax;
      sy += ay;
      sz += az;
      good++;
    }

    delay(15);
  }

  if (good == 0) {
    Serial.println("[DG] IMU calibration failed");
    imuReady = false;
    return;
  }

  imuBaseX = sx / good;
  imuBaseY = sy / good;
  imuBaseZ = sz / good;

  imuFiltX = imuBaseX;
  imuFiltY = imuBaseY;
  imuFiltZ = imuBaseZ;

  imuDX = 0.0f;
  imuDY = 0.0f;
  imuDZ = 0.0f;

  Serial.printf("[DG] IMU base X=%.3f Y=%.3f Z=%.3f\n", imuBaseX, imuBaseY, imuBaseZ);
}

static void updateIMU() {
  if (!imuReady) {
    return;
  }

  float ax, ay, az;

  if (!imuReadAccel(ax, ay, az)) {
    return;
  }

  imuFiltX = imuFiltX * (1.0f - IMU_FILTER_ALPHA) + ax * IMU_FILTER_ALPHA;
  imuFiltY = imuFiltY * (1.0f - IMU_FILTER_ALPHA) + ay * IMU_FILTER_ALPHA;
  imuFiltZ = imuFiltZ * (1.0f - IMU_FILTER_ALPHA) + az * IMU_FILTER_ALPHA;

  imuDX = imuFiltX - imuBaseX;
  imuDY = imuFiltY - imuBaseY;
  imuDZ = imuFiltZ - imuBaseZ;
}

static bool imuForward() {
  return imuReady && imuDX < -IMU_TRIGGER_FWD_G;
}

static bool imuBack() {
  return imuReady && imuDX > IMU_TRIGGER_BACK_G;
}

static bool imuLeft() {
  return imuReady && imuDY > IMU_TRIGGER_TURN_G;
}

static bool imuRight() {
  return imuReady && imuDY < -IMU_TRIGGER_TURN_G;
}

// ---------------- Display init ----------------
static bool initDisplay() {
  Serial.println("[DG] Display init...");

  if (g_sharedDisplayReady && g_sharedGfx != nullptr) {
    Serial.println("[DG] Reusing picker display instance");
    gfx = g_sharedGfx;
    initColors();
    gfx->fillScreen(C_BLACK);
    Serial.println("[DG] Display OK");
    return true;
  }

  bus = new Arduino_ESP32QSPI(
    LCD_CS,
    LCD_SCLK,
    LCD_SDIO0,
    LCD_SDIO1,
    LCD_SDIO2,
    LCD_SDIO3
  );

  gfx = new Arduino_CO5300(
    bus,
    LCD_RESET,
    0,
    LCD_WIDTH,
    LCD_HEIGHT,
    22,
    0,
    6,
    6
  );

  if (!gfx->begin()) {
    Serial.println("[DG] Display begin failed");
    return false;
  }

  initColors();

  gfx->fillScreen(C_BLACK);

  Serial.println("[DG] Display OK");
  return true;
}

// ---------------- Touch init ----------------
static bool initTouch() {
  Serial.println("[DG] Touch init...");

  // Wire is also used by IMU. begin() is safe here before both.
  Wire.begin(IIC_SDA, IIC_SCL);
  Wire.setClock(400000);

  pinMode(TP_RESET, OUTPUT);
  digitalWrite(TP_RESET, LOW);
  delay(10);
  digitalWrite(TP_RESET, HIGH);
  delay(80);

  IICBus = std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

  touch = std::unique_ptr<Arduino_IIC>(
    new Arduino_FT3x68(
      IICBus,
      FT3168_DEVICE_ADDRESS,
      0xFF,
      TP_INT,
      touchInterrupt
    )
  );

  bool ok = false;

  for (int i = 0; i < 10; i++) {
    if (touch->begin()) {
      ok = true;
      break;
    }

    delay(150);
  }

  if (!ok) {
    Serial.println("[DG] Touch begin failed. Continuing without touch.");
    touchReady = false;
    return false;
  }

  pinMode(TP_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TP_INT), touchInterrupt, FALLING);

  touchReady = true;

  Serial.println("[DG] Touch OK");
  return true;
}

// ---------------- Touch read ----------------
static bool readTouchPoint(int &x, int &y) {
  if (!touchReady || !touch) {
    return false;
  }

  bool hasIRQ = touchIRQ || touch->IIC_Interrupt_Flag;
  bool pinActive = digitalRead(TP_INT) == LOW;

  if (!hasIRQ && !pinActive) {
    return false;
  }

  touchIRQ = false;
  touch->IIC_Interrupt_Flag = false;

  int32_t rawX = touch->IIC_Read_Device_Value(
    Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X
  );

  int32_t rawY = touch->IIC_Read_Device_Value(
    Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y
  );

  if (rawX < 0 || rawY < 0) {
    return false;
  }

  int mx = map(rawX, 0, 390, 0, LCD_WIDTH - 1);
  int my = map(rawY, 0, 490, 0, LCD_HEIGHT - 1);

  x = constrain(mx, 0, LCD_WIDTH - 1);
  y = constrain(my, 0, LCD_HEIGHT - 1);

  return true;
}

// ---------------- Control helpers ----------------
static void clearControlState(ControlState &s) {
  s.forward = false;
  s.back = false;
  s.left = false;
  s.right = false;
  s.fire = false;
  s.use = false;
  s.menu = false;
  s.weapon = false;
  s.touching = false;
  s.x = -1;
  s.y = -1;
}

static void setKeyIfChanged(bool now, bool before, unsigned char key) {
  if (now != before) {
    pushKey(now, key);
  }
}

static bool readButtonPressed(int pin, int idleLevel) {
  return digitalRead(pin) != idleLevel;
}

static void updateControls() {
  uint32_t nowMs = millis();

  // DG_DrawFrame() and DG_GetKey() can be called close together; avoid
  // re-reading touch/buttons multiple times inside the same millisecond.
  if (nowMs == lastControlPollMs) {
    serviceWeaponKeyRelease();
    return;
  }
  lastControlPollMs = nowMs;

  oldCtrl = ctrl;

  // Release any weapon key that was pressed on a previous frame.
  serviceWeaponKeyRelease();

  ctrl.forward = false;
  ctrl.back = false;
  ctrl.left = false;
  ctrl.right = false;
  ctrl.fire = false;
  ctrl.use = false;
  ctrl.menu = false;
  ctrl.weapon = false;

  int x;
  int y;

  bool gotTouch = readTouchPoint(x, y);

  if (gotTouch) {
    ctrl.touching = true;
    ctrl.x = x;
    ctrl.y = y;
    lastTouchSeenMs = millis();
  } else {
    if (millis() - lastTouchSeenMs > TOUCH_JOY_RELEASE_MS) {
      ctrl.touching = false;
      ctrl.x = -1;
      ctrl.y = -1;
      joyActive = false;
      joyBaseX = -1;
      joyBaseY = -1;
      joyDX = 0;
      joyDY = 0;
    }
  }

  if (ctrl.touching) {
    x = ctrl.x;
    y = ctrl.y;

    bool touchStarted = !oldCtrl.touching;
    bool topLeftCorner = (x < TOUCH_CORNER_SIZE_PX && y < (VIEW_Y + TOUCH_CORNER_SIZE_PX));
    bool topRightCorner = (x > LCD_WIDTH - TOUCH_CORNER_SIZE_PX && y < (VIEW_Y + TOUCH_CORNER_SIZE_PX));

    if (topRightCorner) {
      // Use/open is now a touch action so the PWR button can become menu/back.
      ctrl.use = true;
      joyActive = false;
    } else if (topLeftCorner) {
      // Weapon switch: one tap advances to the next weapon.
      if (touchStarted) {
        queueNextWeaponCycleKey();
      }

      ctrl.weapon = true;
      joyActive = false;
    } else {
      // Invisible drag joystick.
      if (!oldCtrl.touching || !joyActive) {
        joyActive = true;
        joyBaseX = x;
        joyBaseY = y;
        joyDX = 0;
        joyDY = 0;
      } else {
        joyDX = x - joyBaseX;
        joyDY = y - joyBaseY;
      }

      // Screen coordinates: up is negative Y.
      if (joyDY <= -joyFwdThreshold()) {
        ctrl.forward = true;
      }

      if (joyDY >= joyBackThreshold()) {
        ctrl.back = true;
      }

      // When moving forward/back, require a little more side drag.
      // This prevents accidental over-turning while walking.
      int turnThreshold = joyTurnThreshold(ctrl.forward || ctrl.back);

      if (joyDX <= -turnThreshold) {
        ctrl.left = true;
      }

      if (joyDX >= turnThreshold) {
        ctrl.right = true;
      }
    }
  }

  // Physical buttons:
  // BOOT = fire/select
  // PWR  = menu/back
  // BOOT + PWR together = type save name and press Enter.
  bool bootPressed = readButtonPressed(BTN_BOOT, bootIdleLevel);
  bool pwrPressed = readButtonPressed(BTN_PWR, pwrIdleLevel);

  bool saveMacroCombo = bootPressed && pwrPressed;

  if (saveMacroCombo) {
    uint32_t now = millis();

    if (now - lastSaveNameMacroMs > SAVE_NAME_MACRO_COOLDOWN_MS) {
      lastSaveNameMacroMs = now;
      injectSaveNameMacro();
    }

    // Do not also send normal FIRE/MENU while the combo is held,
    // or the ENTER key could confirm before the name is typed.
  } else {
    if (bootPressed) {
      ctrl.fire = true;
    }

    if (pwrPressed) {
      ctrl.menu = true;
    }
  }

  setKeyIfChanged(ctrl.forward, oldCtrl.forward, KEY_UPARROW);
  setKeyIfChanged(ctrl.back,    oldCtrl.back,    KEY_DOWNARROW);
  setKeyIfChanged(ctrl.left,    oldCtrl.left,    KEY_LEFTARROW);
  setKeyIfChanged(ctrl.right,   oldCtrl.right,   KEY_RIGHTARROW);

  // Gameplay.
  setKeyIfChanged(ctrl.fire,    oldCtrl.fire,    KEY_FIRE);
  setKeyIfChanged(ctrl.use,     oldCtrl.use,     KEY_USE);
  setKeyIfChanged(ctrl.menu,    oldCtrl.menu,    KEY_ESCAPE);

  // Menu selection.
  setKeyIfChanged(ctrl.fire,    oldCtrl.fire,    KEY_ENTER);

  // Weapon switching is handled by the top-left tap above.
}

// ---------------- UI helpers ----------------
static void drawButton(
  int x,
  int y,
  int w,
  int h,
  const char *label,
  bool active,
  uint16_t baseColor
) {
  uint16_t fill = active ? baseColor : C_GRAY;
  uint16_t text = active ? C_WHITE : C_BLACK;

  gfx->fillRoundRect(x, y, w, h, 10, fill);
  gfx->drawRoundRect(x, y, w, h, 10, active ? C_WHITE : C_DARK);

  gfx->setTextSize(1);
  gfx->setTextColor(text, fill);
  gfx->setCursor(x + 8, y + (h / 2) - 4);
  gfx->print(label);
}


// ---------------- Battery HUD ----------------

static bool initPMU() {
  Serial.println("[DG] PMU init...");

  if (!power.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    Serial.println("[DG] PMU init failed. Battery HUD disabled.");
    pmuReady = false;
    return false;
  }

  power.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
  power.clearIrqStatus();

  pmuReady = true;
  Serial.println("[DG] PMU OK");
  return true;
}

static int readBatteryPercentSafe() {
  if (!pmuReady) {
    return -1;
  }

  if (!power.isBatteryConnect()) {
    return -1;
  }

  int pct = power.getBatteryPercent();
  return constrain(pct, 0, 100);
}

static bool readBatteryChargingSafe() {
  if (!pmuReady) {
    return false;
  }

  // XPowersLib versions differ slightly. This method existed in your earlier
  // watch firmware. If this line fails to compile, replace this function body
  // with: return false;
  return power.isCharging();
}

static void drawBatteryHUD(bool force) {
  if (!gfx || !dg_battery_hud_enabled) {
    return;
  }

  uint32_t now = millis();

  if (!force && now - lastBatteryDrawMs < 10000) {
    return;
  }

  lastBatteryDrawMs = now;

  int pct = readBatteryPercentSafe();
  bool charging = readBatteryChargingSafe();

  if (!force && pct == lastBatteryPercent && charging == lastBatteryCharging) {
    return;
  }

  lastBatteryPercent = pct;
  lastBatteryCharging = charging;

  // Clear only the small safe-area HUD, not the game view.
  gfx->fillRect(BATT_HUD_X, BATT_HUD_Y, BATT_HUD_W, BATT_HUD_H, C_BLACK);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GRAY, C_BLACK);
  gfx->setCursor(BATT_HUD_X, BATT_HUD_Y + 5);

  if (pct < 0) {
    gfx->print("BAT --%");
  } else {
    gfx->printf("BAT %d%%%s", pct, charging ? " +" : "");
  }
}

static void drawStaticOverlay() {
  if (!gfx) {
    return;
  }

  // Clean mode:
  // Draw only a black background once at startup.
  // No labels, no control text, no border.
  // This mostly improves visual cleanliness; FPS gain is small because this is not redrawn every frame.
  gfx->fillScreen(C_BLACK);
  drawBatteryHUD(true);
}

// In-game UI overlay removed for FPS. Gameplay rendering stays clean.


static void buildScaleMaps() {
  for (int x = 0; x < VIEW_W; x++) {
    scaleXMap[x] = (uint16_t)((x * DOOM_W) / VIEW_W);
  }

  for (int y = 0; y < VIEW_H; y++) {
    scaleYMap[y] = (uint16_t)((y * DOOM_H) / VIEW_H);
  }

  scaleMapsReady = true;
}

// ---------------- Frame conversion ----------------
static void convertDoomFrameToRGB565() {
  // With "-gfxmode rgb565", doomgeneric/i_video.c writes DG_ScreenBuffer
  // as packed 16-bit RGB565 pixels. Precomputed maps remove two divisions
  // per output pixel from the hot frame path.
  if (!scaleMapsReady) {
    buildScaleMaps();
  }

  const uint16_t *src = (const uint16_t *)DG_ScreenBuffer;

  for (int y = 0; y < VIEW_H; y++) {
    const uint16_t *srcRow = src + ((uint32_t)scaleYMap[y] * DOOM_W);
    uint16_t *dstRow = frame565 + ((uint32_t)y * VIEW_W);

    for (int x = 0; x < VIEW_W; x++) {
      dstRow[x] = srcRow[scaleXMap[x]];
    }
  }
}

// ==========================================================
// ESP32 DOOM SFX AUDIO MODULE
// ==========================================================
//
// doomgeneric's README says sound support needs FEATURE_SOUND and a
// DG_sound_module. This module provides sampled SFX only.
// Music stays stubbed/disabled for now.
//
// Hardware audio path uses the working Waveshare method:
// - ESP_I2S I2SClass
// - official es8311.h driver
// - setPins(BCLK, WS, DOUT, DIN, MCLK)
// - DOUT = GPIO40, DIN = GPIO42
//
// Required sketch files:
// - es8311.h
// - es8311.c
// ==========================================================

#define DOOM_AUDIO_RATE        16000
#define DOOM_AUDIO_FRAMES      256
#define DOOM_AUDIO_CHANNELS    8
#define DOOM_AUDIO_GAIN        1

// Hard-code the known-working Waveshare audio mapping.
// Your older pin_config had ASDOUT/DSDIN swapped.
#define DOOM_AUDIO_DOUT_PIN    40
#define DOOM_AUDIO_DIN_PIN     42

static I2SClass doomI2S;
static bool doomAudioReady = false;
static bool doomAudioTaskRunning = false;
static TaskHandle_t doomAudioTaskHandle = nullptr;
static es8311_handle_t doomEs8311 = nullptr;
static portMUX_TYPE doomAudioMux = portMUX_INITIALIZER_UNLOCKED;

struct DoomAudioChannel {
  uint8_t *data;
  uint32_t length;
  uint32_t pos_fp;
  uint32_t step_fp;
  int leftvol;
  int rightvol;
  bool active;
};

static DoomAudioChannel doomChannels[DOOM_AUDIO_CHANNELS];

static snddevice_t doomSfxDevices[] = {
  SNDDEVICE_SB
};

static snddevice_t doomMusicDevices[] = {
  SNDDEVICE_NONE
};

static uint32_t readLE32(const uint8_t *p) {
  return ((uint32_t)p[0]) |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static uint16_t readLE16(const uint8_t *p) {
  return ((uint16_t)p[0]) | ((uint16_t)p[1] << 8);
}

static void freeDoomChannelLocked(int ch) {
  if (ch < 0 || ch >= DOOM_AUDIO_CHANNELS) {
    return;
  }

  if (doomChannels[ch].data != nullptr) {
    heap_caps_free(doomChannels[ch].data);
  }

  doomChannels[ch].data = nullptr;
  doomChannels[ch].length = 0;
  doomChannels[ch].pos_fp = 0;
  doomChannels[ch].step_fp = 0;
  doomChannels[ch].leftvol = 0;
  doomChannels[ch].rightvol = 0;
  doomChannels[ch].active = false;
}

static void calcStereoVol(int vol, int sep, int &left, int &right) {
  vol = constrain(vol, 0, 127);
  sep = constrain(sep, 0, 254);

  // DOOM sep is 128-ish center.
  left = (vol * (255 - sep)) / 128;
  right = (vol * sep) / 128;

  left = constrain(left, 0, 127);
  right = constrain(right, 0, 127);
}

static bool initDoomES8311() {
  Serial.println("[SFX] ES8311 official driver init...");

  doomEs8311 = es8311_create(0, ES8311_ADDRRES_0);
  if (!doomEs8311) {
    Serial.println("[SFX] es8311_create failed");
    return false;
  }

  const es8311_clock_config_t es_clk = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency = DOOM_AUDIO_RATE * 256,
    .sample_frequency = DOOM_AUDIO_RATE
  };

  esp_err_t err;

  err = es8311_init(doomEs8311, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
  if (err != ESP_OK) {
    Serial.printf("[SFX] es8311_init failed: %d\n", err);
    return false;
  }

  err = es8311_sample_frequency_config(doomEs8311, es_clk.mclk_frequency, es_clk.sample_frequency);
  if (err != ESP_OK) {
    Serial.printf("[SFX] es8311_sample_frequency_config failed: %d\n", err);
    return false;
  }

  // Disable mic path in ES8311; we only need DAC/speaker for DOOM.
  es8311_microphone_config(doomEs8311, false);

  err = es8311_voice_volume_set(doomEs8311, constrain(dg_audio_volume, 0, 100), NULL);
  if (err != ESP_OK) {
    Serial.printf("[SFX] es8311_voice_volume_set failed: %d\n", err);
    return false;
  }

  Serial.printf("[SFX] ES8311 OK, volume=%d\n", constrain(dg_audio_volume, 0, 100));
  return true;
}

static bool initDoomI2S() {
  Serial.println("[SFX] I2SClass init...");

  pinMode(PA_CTRL, OUTPUT);
  digitalWrite(PA_CTRL, HIGH);

  // Wire is shared by touch + IMU + codec.
  Wire.begin(IIC_SDA, IIC_SCL);
  Wire.setClock(400000);

  Serial.printf("[SFX] pins BCLK=%d WS=%d DOUT=%d DIN=%d MCLK=%d PA=%d\n",
                I2S_SCLK,
                I2S_LRCK,
                DOOM_AUDIO_DOUT_PIN,
                DOOM_AUDIO_DIN_PIN,
                I2S_MCLK,
                PA_CTRL);

  // Correct ESP_I2S order:
  // setPins(BCLK, WS, DOUT, DIN, MCLK)
  doomI2S.setPins(
    I2S_SCLK,
    I2S_LRCK,
    DOOM_AUDIO_DOUT_PIN,
    DOOM_AUDIO_DIN_PIN,
    I2S_MCLK
  );

  bool ok = doomI2S.begin(
    I2S_MODE_STD,
    DOOM_AUDIO_RATE,
    I2S_DATA_BIT_WIDTH_16BIT,
    I2S_SLOT_MODE_STEREO,
    I2S_STD_SLOT_BOTH
  );

  if (!ok) {
    Serial.println("[SFX] I2S begin failed");
    return false;
  }

  Serial.println("[SFX] I2S OK");
  return true;
}

static void doomAudioTask(void *param) {
  int16_t mix[DOOM_AUDIO_FRAMES * 2];

  doomAudioTaskRunning = true;

  while (doomAudioTaskRunning) {
    for (int i = 0; i < DOOM_AUDIO_FRAMES * 2; i++) {
      mix[i] = 0;
    }

    portENTER_CRITICAL(&doomAudioMux);

    for (int ch = 0; ch < DOOM_AUDIO_CHANNELS; ch++) {
      DoomAudioChannel &c = doomChannels[ch];

      if (!c.active || c.data == nullptr || c.length == 0) {
        continue;
      }

      for (int i = 0; i < DOOM_AUDIO_FRAMES; i++) {
        uint32_t idx = c.pos_fp >> 16;

        if (idx >= c.length) {
          freeDoomChannelLocked(ch);
          break;
        }

        int sample = ((int)c.data[idx] - 128) << 8;

        int l = (sample * c.leftvol * DOOM_AUDIO_GAIN) / 127;
        int r = (sample * c.rightvol * DOOM_AUDIO_GAIN) / 127;

        int li = i * 2 + 0;
        int ri = i * 2 + 1;

        int ml = (int)mix[li] + l;
        int mr = (int)mix[ri] + r;

        mix[li] = (int16_t)constrain(ml, -32768, 32767);
        mix[ri] = (int16_t)constrain(mr, -32768, 32767);

        c.pos_fp += c.step_fp;
      }
    }

    portEXIT_CRITICAL(&doomAudioMux);

    doomI2S.write((uint8_t *)mix, sizeof(mix));

    // Keep task cooperative.
    vTaskDelay(1);
  }

  vTaskDelete(NULL);
}

static boolean DG_SFX_Init(boolean use_sfx_prefix) {
  Serial.println("[SFX] DG_SFX_Init");

  for (int i = 0; i < DOOM_AUDIO_CHANNELS; i++) {
    doomChannels[i].data = nullptr;
    doomChannels[i].active = false;
  }

  if (!initDoomI2S()) {
    return false;
  }

  if (!initDoomES8311()) {
    return false;
  }

  BaseType_t ok = xTaskCreatePinnedToCore(
    doomAudioTask,
    "doom_sfx",
    4096,
    NULL,
    2,
    &doomAudioTaskHandle,
    0
  );

  if (ok != pdPASS) {
    Serial.println("[SFX] audio task create failed");
    return false;
  }

  doomAudioReady = true;
  Serial.println("[SFX] Ready");
  return true;
}

static void DG_SFX_Shutdown(void) {
  doomAudioTaskRunning = false;
  doomAudioTaskHandle = nullptr;

  portENTER_CRITICAL(&doomAudioMux);
  for (int i = 0; i < DOOM_AUDIO_CHANNELS; i++) {
    freeDoomChannelLocked(i);
  }
  portEXIT_CRITICAL(&doomAudioMux);

  doomAudioReady = false;
}

static int DG_SFX_GetSfxLumpNum(sfxinfo_t *sfxinfo) {
  char namebuf[9];

  // DOOM digital sound lumps are DSxxxxxx.
  snprintf(namebuf, sizeof(namebuf), "ds%s", sfxinfo->name);

  int lump = W_CheckNumForName(namebuf);

  if (lump < 0) {
    // Some builds use uppercase lookup.
    snprintf(namebuf, sizeof(namebuf), "DS%s", sfxinfo->name);
    lump = W_CheckNumForName(namebuf);
  }

  return lump;
}

static void DG_SFX_Update(void) {
  // Audio is mixed by the FreeRTOS audio task.
}

static void DG_SFX_UpdateSoundParams(int channel, int vol, int sep) {
  if (channel < 0 || channel >= DOOM_AUDIO_CHANNELS) {
    return;
  }

  int left, right;
  calcStereoVol(vol, sep, left, right);

  portENTER_CRITICAL(&doomAudioMux);
  doomChannels[channel].leftvol = left;
  doomChannels[channel].rightvol = right;
  portEXIT_CRITICAL(&doomAudioMux);
}

static int DG_SFX_StartSound(sfxinfo_t *sfxinfo, int channel, int vol, int sep) {
  if (!doomAudioReady) {
    return -1;
  }

  if (channel < 0) {
    channel = 0;
  }

  channel = channel % DOOM_AUDIO_CHANNELS;

  int lump = sfxinfo->lumpnum;

  if (lump < 0) {
    lump = DG_SFX_GetSfxLumpNum(sfxinfo);
  }

  if (lump < 0) {
    return -1;
  }

  int lumpLen = W_LumpLength(lump);

  if (lumpLen <= 8) {
    return -1;
  }

  const uint8_t *raw = (const uint8_t *)W_CacheLumpNum(lump, PU_STATIC);

  if (raw == nullptr) {
    return -1;
  }

  // DMX sound header:
  // 0..1 = type
  // 2..3 = sample rate
  // 4..7 = sample count
  // 8..  = unsigned 8-bit PCM samples
  uint16_t sampleRate = readLE16(raw + 2);
  uint32_t sampleCount = readLE32(raw + 4);

  if (sampleRate < 4000 || sampleRate > 48000) {
    sampleRate = 11025;
  }

  uint32_t available = lumpLen - 8;

  if (sampleCount == 0 || sampleCount > available) {
    sampleCount = available;
  }

  if (sampleCount == 0) {
    W_ReleaseLumpNum(lump);
    return -1;
  }

  uint8_t *copy = (uint8_t *)heap_caps_malloc(
    sampleCount,
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );

  if (copy == nullptr) {
    copy = (uint8_t *)malloc(sampleCount);
  }

  if (copy == nullptr) {
    W_ReleaseLumpNum(lump);
    return -1;
  }

  memcpy(copy, raw + 8, sampleCount);
  W_ReleaseLumpNum(lump);

  int left, right;
  calcStereoVol(vol, sep, left, right);

  portENTER_CRITICAL(&doomAudioMux);

  freeDoomChannelLocked(channel);

  doomChannels[channel].data = copy;
  doomChannels[channel].length = sampleCount;
  doomChannels[channel].pos_fp = 0;
  doomChannels[channel].step_fp = ((uint32_t)sampleRate << 16) / DOOM_AUDIO_RATE;
  doomChannels[channel].leftvol = left;
  doomChannels[channel].rightvol = right;
  doomChannels[channel].active = true;

  portEXIT_CRITICAL(&doomAudioMux);

  return channel;
}

static void DG_SFX_StopSound(int channel) {
  if (channel < 0 || channel >= DOOM_AUDIO_CHANNELS) {
    return;
  }

  portENTER_CRITICAL(&doomAudioMux);
  freeDoomChannelLocked(channel);
  portEXIT_CRITICAL(&doomAudioMux);
}

static boolean DG_SFX_SoundIsPlaying(int channel) {
  if (channel < 0 || channel >= DOOM_AUDIO_CHANNELS) {
    return false;
  }

  boolean playing;

  portENTER_CRITICAL(&doomAudioMux);
  playing = doomChannels[channel].active ? true : false;
  portEXIT_CRITICAL(&doomAudioMux);

  return playing;
}

static void DG_SFX_CacheSounds(sfxinfo_t *sounds, int num_sounds) {
  // Do not precache on ESP32-S3. We load/copy SFX on demand into PSRAM.
  Serial.printf("[SFX] CacheSounds skipped: %d sounds\n", num_sounds);
}

// Music stub. Music remains disabled with -nomusic.
static boolean DG_Music_Init(void) { return true; }
static void DG_Music_Shutdown(void) {}
static void DG_Music_SetMusicVolume(int volume) {}
static void DG_Music_PauseMusic(void) {}
static void DG_Music_ResumeMusic(void) {}
static void *DG_Music_RegisterSong(void *data, int len) { return NULL; }
static void DG_Music_UnRegisterSong(void *handle) {}
static void DG_Music_PlaySong(void *handle, boolean looping) {}
static void DG_Music_StopSong(void) {}
static boolean DG_Music_MusicIsPlaying(void) { return false; }
static void DG_Music_Poll(void) {}

extern "C" sound_module_t DG_sound_module = {
  doomSfxDevices,
  1,
  DG_SFX_Init,
  DG_SFX_Shutdown,
  DG_SFX_GetSfxLumpNum,
  DG_SFX_Update,
  DG_SFX_UpdateSoundParams,
  DG_SFX_StartSound,
  DG_SFX_StopSound,
  DG_SFX_SoundIsPlaying,
  DG_SFX_CacheSounds
};

extern "C" music_module_t DG_music_module = {
  doomMusicDevices,
  1,
  DG_Music_Init,
  DG_Music_Shutdown,
  DG_Music_SetMusicVolume,
  DG_Music_PauseMusic,
  DG_Music_ResumeMusic,
  DG_Music_RegisterSong,
  DG_Music_UnRegisterSong,
  DG_Music_PlaySong,
  DG_Music_StopSong,
  DG_Music_MusicIsPlaying,
  DG_Music_Poll
};


// ==========================================================
// doomgeneric required platform functions
// ==========================================================

extern "C" void DG_Init() {
  Serial.println("[DG] DG_Init");

  pinMode(BTN_BOOT, INPUT_PULLUP);
  pinMode(BTN_PWR, INPUT_PULLUP);

  if (!initDisplay()) {
    Serial.println("[DG] FATAL: display init failed");

    while (true) {
      delay(1000);
      yield();
    }
  }

  // One I2C bus: initialize Wire once, then touch/codec.
  Wire.begin(IIC_SDA, IIC_SCL);
  Wire.setClock(400000);

  initPMU();
  initTouch();
  // IMU is intentionally not used in this v1.2 control build.

  frame565 = (uint16_t *)heap_caps_malloc(
    VIEW_W * VIEW_H * sizeof(uint16_t),
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );

  if (!frame565) {
    Serial.println("[DG] PSRAM framebuffer failed, trying malloc");
    frame565 = (uint16_t *)malloc(VIEW_W * VIEW_H * sizeof(uint16_t));
  }

  if (!frame565) {
    gfx->fillScreen(C_BLACK);
    gfx->setTextSize(2);
    gfx->setTextColor(C_RED, C_BLACK);
    gfx->setCursor(20, 220);
    gfx->print("Frame alloc failed");

    Serial.println("[DG] FATAL: frame565 allocation failed");

    while (true) {
      delay(1000);
      yield();
    }
  }

  buildScaleMaps();

  clearControlState(ctrl);
  clearControlState(oldCtrl);

  // Calibrate button idle levels. Do not press BOOT/PWR during startup.
  delay(80);
  bootIdleLevel = digitalRead(BTN_BOOT);
  pwrIdleLevel = digitalRead(BTN_PWR);

  Serial.printf("[DG] Button idle levels: BOOT=%d PWR=%d\n", bootIdleLevel, pwrIdleLevel);

  drawStaticOverlay();
  gameplayStartedAtMs = millis();

  Serial.printf("[DG] frame565 allocated: %u bytes for %dx%d RGB565 direct view\n",
                (unsigned)(VIEW_W * VIEW_H * 2),
                VIEW_W,
                VIEW_H);
  Serial.printf("[DG] DG_ScreenBuffer ptr: %p\n", DG_ScreenBuffer);
  Serial.printf("[DG] free PSRAM %.2f MB | free heap %.2f KB\n",
                ESP.getFreePsram() / 1024.0 / 1024.0,
                ESP.getFreeHeap() / 1024.0);
}

extern "C" void DG_DrawFrame() {
  if (!gfx || !frame565 || !DG_ScreenBuffer) {
    return;
  }

  updateControls();
  convertDoomFrameToRGB565();

  gfx->draw16bitRGBBitmap(
    VIEW_X,
    VIEW_Y,
    frame565,
    VIEW_W,
    VIEW_H
  );

  // The 410x256 game view is centered lower on the screen, so this small
  // safe-area HUD is not overwritten by the DOOM frame.
  drawBatteryHUD(false);

  framesThisSecond++;

  if (millis() - lastFpsMs >= 3000) {
    Serial.printf("[DG FPS avg/3s] %u | joyDX=%d joyDY=%d | free heap %.2f KB\n",
                  (unsigned)(framesThisSecond / 3),
                  joyDX,
                  joyDY,
                  ESP.getFreeHeap() / 1024.0);

    framesThisSecond = 0;
    lastFpsMs = millis();
  }
}

extern "C" void DG_SleepMs(uint32_t ms) {
  delay(ms);
}

extern "C" uint32_t DG_GetTicksMs() {
  return millis();
}

extern "C" int DG_GetKey(int *pressed, unsigned char *doomKey) {
  updateControls();

  if (popKey(pressed, doomKey)) {
    return 1;
  }

  return 0;
}

extern "C" void DG_SetWindowTitle(const char *title) {
  Serial.print("[DG TITLE] ");
  Serial.println(title ? title : "");
}
