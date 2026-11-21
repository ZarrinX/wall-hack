/*
  wall-hack v1.0 firmware for the LCDWiki E32R40T, LD2450, and touchscreen.

  Board: LCDWiki 4.0-inch ESP32-32E (E32R40T), portrait 320x480.
  The UI reserves the top 15 pixels for a menu button and battery percentage.
  This sketch expects TFT_eSPI configured with the user's
  Setup_E32R40T.h, as described in E32R40T_LD2450_Agent_Handoff.md.

  LD2450:
    TX -> GPIO19 (UART2 RX)
    RX -> GPIO23 (UART2 TX)
    VCC -> PowerBoost 500C 5V output
    GND -> common ground

  PowerBoost 500C EN -> GPIO25 (HIGH on, LOW off).
  Onboard LCD backlight is GPIO27, active high. GPIO34 is the onboard battery
  ADC input. Its divider ratio and calibration remain assumptions in this
  firmware; voltage-derived SOC is only a rough estimate.

  GPIO19/GPIO23 are shared with the board's microSD SPI wiring. Leave the SD
  card unused while the LD2450 is attached to these UART pins.

  First boot for this portrait UI runs a separate TFT_eSPI touch calibration
  and saves it in Preferences, so landscape calibration data is not reused.
  Send 'c' over USB serial to run calibration again. Use MENU > Shut Down to
  stop UART, tri-state the radar pins, disable and hold boost EN low, sleep the
  TFT, and enter ESP32 deep sleep. The board RESET button restarts the ESP32.

  Power and battery behavior must be measured on the assembled hardware.
  This sketch does not establish charger termination, safe charge behavior,
  accurate battery SOC, or actual deep-sleep current.
*/

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <math.h>

static constexpr char FIRMWARE_VERSION[] = "1.0";

static constexpr int LD2450_RX_PIN = 19;
static constexpr int LD2450_TX_PIN = 23;
static constexpr uint32_t LD2450_BAUD = 256000;
static constexpr int BOOST_EN_PIN = 25; // E32R40T I2C pins are not used by this sketch.
static constexpr int LCD_BACKLIGHT_PIN = 27;
static constexpr int BATTERY_ADC_PIN = 34;

static constexpr int SCREEN_WIDTH = 320;
static constexpr int STATUS_BAR_HEIGHT = 15;
static constexpr uint32_t UI_CHECK_INTERVAL_MS = 100;
static constexpr uint32_t BATTERY_SAMPLE_INTERVAL_MS = 1000;
static constexpr uint32_t FRAME_TIMEOUT_MS = 3000;
static constexpr uint16_t MAX_RANGE_MM = 6000;
static constexpr uint8_t TARGET_COUNT = 3;
static constexpr uint8_t LD2450_FRAME_SIZE = 30;

// These values are provisional until the divider and ADC are measured.
static constexpr float BATTERY_DIVIDER_RATIO = 2.0f;
static constexpr float BATTERY_CALIBRATION = 1.0f;

static constexpr uint16_t COLOR_BG = TFT_BLACK;
static constexpr uint16_t COLOR_PANEL = 0x0100;
static constexpr uint16_t COLOR_GRID = 0x0320;
static constexpr uint16_t COLOR_TEXT = 0xC7F5;
static constexpr uint16_t COLOR_MUTED = 0x4EAA;
static constexpr uint16_t COLOR_ACCENT = TFT_GREEN;
static constexpr uint16_t TARGET_COLORS[TARGET_COUNT] = {TFT_GREEN, TFT_RED, TFT_BLUE};

static constexpr int RADAR_CX = 160;
static constexpr int RADAR_CY = 315;
static constexpr int RADAR_RADIUS = 142;
static constexpr int MENU_BUTTON_X = 4;
static constexpr int MENU_BUTTON_Y = 1;
static constexpr int MENU_BUTTON_W = 42;
static constexpr int MENU_BUTTON_H = STATUS_BAR_HEIGHT - 2;
static constexpr int MENU_TOUCH_X = 0;
static constexpr int MENU_TOUCH_Y = 0;
static constexpr int MENU_TOUCH_W = 64;
static constexpr int MENU_TOUCH_H = STATUS_BAR_HEIGHT;
static constexpr int BATTERY_STATUS_X = 264;
static constexpr int BATTERY_STATUS_W = SCREEN_WIDTH - BATTERY_STATUS_X;
static constexpr int MENU_ITEM_X = 50;
static constexpr int MENU_ITEM_Y = 112;
static constexpr int MENU_ITEM_W = 220;
static constexpr int MENU_ITEM_H = 54;

HardwareSerial RadarSerial(2);
TFT_eSPI tft;
Preferences preferences;

struct RadarTarget {
  bool active = false;
  int16_t xMm = 0;
  int16_t yMm = 0;
  int16_t velocityCmPerSec = 0;
  uint16_t rangeResolutionMm = 0;
};

struct RadarReading {
  bool valid = false;
  RadarTarget targets[TARGET_COUNT];
  uint32_t observedAtMs = 0;
};

struct TargetMarker {
  bool visible = false;
  int16_t x = 0;
  int16_t y = 0;
  uint16_t color = COLOR_TEXT;
};

class Ld2450Parser {
public:
  bool readFrame(HardwareSerial &serial, RadarReading &reading) {
    while (serial.available() > 0) {
      const uint8_t byteValue = static_cast<uint8_t>(serial.read());
      if (consume(byteValue, reading)) return true;
    }
    return false;
  }

private:
  bool consume(uint8_t byteValue, RadarReading &reading) {
    static constexpr uint8_t HEADER[] = {0xAA, 0xFF, 0x03, 0x00};
    if (index_ < sizeof(HEADER)) {
      if (byteValue == HEADER[index_]) {
        frame_[index_++] = byteValue;
      } else {
        index_ = byteValue == HEADER[0] ? 1 : 0;
        if (index_ == 1) frame_[0] = byteValue;
      }
      return false;
    }

    frame_[index_++] = byteValue;
    if (index_ < LD2450_FRAME_SIZE) return false;
    const bool parsed = parseFrame(reading);
    index_ = 0;
    return parsed;
  }

  bool parseFrame(RadarReading &reading) {
    if (frame_[28] != 0x55 || frame_[29] != 0xCC) return false;
    for (uint8_t i = 0; i < TARGET_COUNT; i++) {
      const uint8_t *record = &frame_[4 + i * 8];
      const uint16_t rawX = readLe16(record);
      const uint16_t rawY = readLe16(record + 2);
      const uint16_t rawVelocity = readLe16(record + 4);
      const uint16_t resolution = readLe16(record + 6);
      RadarTarget &target = reading.targets[i];
      target.xMm = decodeSignedMagnitude(rawX);
      target.yMm = decodeSignedMagnitude(rawY);
      target.velocityCmPerSec = decodeSignedMagnitude(rawVelocity);
      target.rangeResolutionMm = resolution;
      target.active = rawX != 0 || rawY != 0 || rawVelocity != 0 || resolution != 0;
    }
    reading.valid = true;
    reading.observedAtMs = millis();
    return true;
  }

  uint16_t readLe16(const uint8_t *bytes) const {
    return bytes[0] | (static_cast<uint16_t>(bytes[1]) << 8);
  }

  int16_t decodeSignedMagnitude(uint16_t rawValue) const {
    const int16_t magnitude = static_cast<int16_t>(rawValue & 0x7FFF);
    return (rawValue & 0x8000) ? magnitude : -magnitude;
  }

  uint8_t frame_[LD2450_FRAME_SIZE] = {};
  uint8_t index_ = 0;
};

Ld2450Parser parser;
RadarReading latestReading;
RadarReading lastRenderedReading;
TargetMarker lastRenderedMarkers[TARGET_COUNT];
uint16_t touchCalibration[5] = {};
uint32_t lastUiCheckMs = 0;
uint32_t lastBatterySampleMs = 0;
uint16_t batteryPinMilliVolts = 0;
float batteryVoltsEstimate = 0.0f;
uint8_t batterySocEstimate = 0;
bool touchWasDown = false;
bool sleepRequested = false;
bool uiInitialized = false;
bool lastRadarOnline = false;
bool menuOpen = false;
int lastDrawnBatterySoc = -1;

uint16_t activeTargetCount(const RadarReading &reading) {
  uint16_t count = 0;
  for (uint8_t i = 0; i < TARGET_COUNT; i++) {
    if (reading.targets[i].active) count++;
  }
  return count;
}

uint32_t targetRangeMm(const RadarTarget &target) {
  return static_cast<uint32_t>(sqrtf(
      static_cast<float>(target.xMm) * target.xMm +
      static_cast<float>(target.yMm) * target.yMm));
}

int targetAngleDegrees(const RadarTarget &target) {
  const float degrees = atan2f(static_cast<float>(target.xMm),
                               static_cast<float>(target.yMm)) * 57.2957795f;
  return static_cast<int>(degrees >= 0.0f ? degrees + 0.5f : degrees - 0.5f);
}

bool sameTarget(const RadarTarget &a, const RadarTarget &b) {
  return a.active == b.active && (!a.active ||
      (a.xMm == b.xMm && a.yMm == b.yMm &&
       a.velocityCmPerSec == b.velocityCmPerSec));
}

TargetMarker makeTargetMarker(const RadarTarget &target, bool online, uint8_t index) {
  TargetMarker marker;
  const uint32_t rangeMm = targetRangeMm(target);
  if (!online || !target.active || target.yMm <= 0 || rangeMm == 0 ||
      rangeMm > MAX_RANGE_MM) return marker;
  marker.visible = true;
  marker.x = RADAR_CX + static_cast<int32_t>(target.xMm) * RADAR_RADIUS / MAX_RANGE_MM;
  marker.y = RADAR_CY - static_cast<int32_t>(target.yMm) * RADAR_RADIUS / MAX_RANGE_MM;
  marker.color = TARGET_COLORS[index];
  return marker;
}

bool sameMarker(const TargetMarker &a, const TargetMarker &b) {
  return a.visible == b.visible && (!a.visible || (a.x == b.x && a.y == b.y));
}

void drawRadarGrid() {
  const int ringDegrees[] = {210, 240, 270, 300, 330};
  for (uint8_t ring = 1; ring <= 3; ring++) {
    const int radius = RADAR_RADIUS * ring / 3;
    for (int degree = 210; degree <= 330; degree += 2) {
      const float radians = degree * 0.0174532925f;
      tft.drawPixel(RADAR_CX + static_cast<int>(radius * cosf(radians)),
                    RADAR_CY + static_cast<int>(radius * sinf(radians)), COLOR_GRID);
    }
    char label[8];
    snprintf(label, sizeof(label), "%um", ring * 2);
    tft.setTextColor(COLOR_ACCENT, COLOR_BG);
    const float labelRadians = 300.0f * 0.0174532925f;
    tft.setCursor(RADAR_CX + static_cast<int>(radius * cosf(labelRadians)) + 3,
                  RADAR_CY + static_cast<int>(radius * sinf(labelRadians)) - 5);
    tft.print(label);
  }

  const char *angleLabels[] = {"-60", "-30", "0", "+30", "+60"};
  const int labelX[] = {11, 58, 153, 238, 280};
  const int labelY[] = {228, 175, 123, 175, 228};
  for (uint8_t i = 0; i < 5; i++) {
    const float radians = ringDegrees[i] * 0.0174532925f;
    const int x = RADAR_CX + static_cast<int>(RADAR_RADIUS * cosf(radians));
    const int y = RADAR_CY + static_cast<int>(RADAR_RADIUS * sinf(radians));
    tft.drawLine(RADAR_CX, RADAR_CY, x, y, COLOR_GRID);
    tft.setTextColor(COLOR_MUTED, COLOR_BG);
    tft.setCursor(labelX[i], labelY[i]);
    tft.printf("%s deg", angleLabels[i]);
  }
  tft.fillCircle(RADAR_CX, RADAR_CY, 4, COLOR_ACCENT);
}

void drawStatusBar() {
  tft.fillRect(0, 0, SCREEN_WIDTH, STATUS_BAR_HEIGHT, COLOR_PANEL);
  tft.drawRoundRect(MENU_BUTTON_X, MENU_BUTTON_Y, MENU_BUTTON_W, MENU_BUTTON_H, 3, COLOR_GRID);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_ACCENT, COLOR_PANEL);
  tft.setCursor(MENU_BUTTON_X + 9, 4);
  tft.print(menuOpen ? "BACK" : "MENU");

  tft.setTextColor(COLOR_ACCENT, COLOR_PANEL);
  tft.setCursor(BATTERY_STATUS_X + 2, 4);
  if (batteryVoltsEstimate < 0.1f) tft.print("BAT --%");
  else tft.printf("BAT %u%%", batterySocEstimate);
}

void drawStaticUi() {
  tft.fillScreen(COLOR_BG);
  drawStatusBar();
  tft.setTextWrap(false);
  tft.fillRect(0, STATUS_BAR_HEIGHT, SCREEN_WIDTH, 54, COLOR_PANEL);
  tft.setTextColor(COLOR_ACCENT, COLOR_PANEL);
  tft.setTextSize(2);
  tft.setCursor(14, 28);
  tft.print("LD2450 TARGET RADAR");
  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED, COLOR_PANEL);
  tft.setCursor(14, 54);
  tft.print("ESP32 UART2  |  24 GHz  |  UP TO 3 TRACKS");

  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  tft.setCursor(14, 78);
  tft.print("RANGE (m)  /  ANGLE (deg)");
  tft.setTextColor(COLOR_MUTED, COLOR_BG);
  tft.setCursor(14, 91);
  tft.print("Targets plotted from module X/Y coordinates");
  drawRadarGrid();
  tft.setTextColor(COLOR_MUTED, COLOR_BG);
  tft.setCursor(138, 322);
  tft.print("SENSOR");

  tft.drawFastHLine(12, 342, 296, COLOR_GRID);
  tft.fillRoundRect(12, 354, 296, 38, 5, COLOR_PANEL);
  tft.setTextColor(COLOR_MUTED, COLOR_PANEL);
  tft.setCursor(22, 358);
  tft.print("TARGETS");
  tft.setCursor(170, 358);
  tft.print("RADAR LINK");

  tft.drawFastHLine(12, 401, 296, COLOR_GRID);
  for (uint8_t i = 0; i < TARGET_COUNT; i++) {
    const int x = 18 + i * 98;
    tft.setTextColor(TARGET_COLORS[i], COLOR_BG);
    tft.setCursor(x, 406);
    tft.printf("TARGET %u", i + 1);
    tft.setTextColor(COLOR_MUTED, COLOR_BG);
    tft.setCursor(x, 423); tft.print("RANGE");
    tft.setCursor(x, 441); tft.print("ANGLE");
    tft.setCursor(x, 459); tft.print("SPEED");
  }
}

void drawMenuUi() {
  tft.fillScreen(COLOR_BG);
  drawStatusBar();
  tft.setTextColor(COLOR_ACCENT, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(18, 30);
  tft.print("MENU");
  tft.setTextSize(1);
  tft.drawRoundRect(MENU_ITEM_X, MENU_ITEM_Y, MENU_ITEM_W, MENU_ITEM_H, 6, COLOR_ACCENT);
  tft.fillRoundRect(MENU_ITEM_X + 1, MENU_ITEM_Y + 1,
                    MENU_ITEM_W - 2, MENU_ITEM_H - 2, 5, COLOR_PANEL);
  tft.setTextColor(COLOR_ACCENT, COLOR_PANEL);
  tft.setTextSize(2);
  tft.setCursor(MENU_ITEM_X + 42, MENU_ITEM_Y + 19);
  tft.print("Shut Down");
  tft.setTextSize(1);
}

void loadOrCreateTouchCalibration() {
  preferences.begin("radar-p", false);
  const bool saved = preferences.getBytesLength("touch") == sizeof(touchCalibration);
  if (saved) {
    preferences.getBytes("touch", touchCalibration, sizeof(touchCalibration));
    tft.setTouch(touchCalibration);
  } else {
    tft.fillScreen(COLOR_BG);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.setTextSize(2);
    tft.setCursor(22, 28);
    tft.print("Touch calibration");
    tft.setTextSize(1);
    tft.setCursor(22, 58);
    tft.print("Tap each target as it appears.");
    tft.calibrateTouch(touchCalibration, TFT_GREEN, COLOR_BG, 15);
    tft.setTouch(touchCalibration);
    preferences.putBytes("touch", touchCalibration, sizeof(touchCalibration));
  }
  preferences.end();
}

void recalibrateTouch() {
  tft.fillScreen(COLOR_BG);
  tft.calibrateTouch(touchCalibration, TFT_GREEN, COLOR_BG, 15);
  tft.setTouch(touchCalibration);
  preferences.begin("radar-p", false);
  preferences.putBytes("touch", touchCalibration, sizeof(touchCalibration));
  preferences.end();
  for (uint8_t i = 0; i < TARGET_COUNT; i++) lastRenderedMarkers[i] = TargetMarker{};
  menuOpen = false;
  uiInitialized = false;
  drawStaticUi();
}

void drawLineInsidePatch(int x0, int y0, int x1, int y1,
                         int left, int top, int right, int bottom) {
  const int dx = abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  while (true) {
    if (x0 >= left && x0 <= right && y0 >= top && y0 <= bottom) {
      tft.drawPixel(x0, y0, COLOR_GRID);
    }
    if (x0 == x1 && y0 == y1) break;
    const int twiceError = 2 * error;
    if (twiceError >= dy) { error += dy; x0 += sx; }
    if (twiceError <= dx) { error += dx; y0 += sy; }
  }
}

void drawMarkerInsidePatch(const TargetMarker &marker,
                           int left, int top, int right, int bottom) {
  if (!marker.visible) return;
  constexpr int outerRadius = 8;
  constexpr int fillRadiusSquared = 25;
  constexpr int outerRadiusSquared = outerRadius * outerRadius;
  for (int y = max(top, static_cast<int>(marker.y) - outerRadius);
       y <= min(bottom, static_cast<int>(marker.y) + outerRadius); y++) {
    for (int x = max(left, static_cast<int>(marker.x) - outerRadius);
         x <= min(right, static_cast<int>(marker.x) + outerRadius); x++) {
      const int dx = x - marker.x;
      const int dy = y - marker.y;
      const int d2 = dx * dx + dy * dy;
      if (d2 <= fillRadiusSquared) tft.drawPixel(x, y, marker.color);
      else if (d2 <= outerRadiusSquared) tft.drawPixel(x, y, COLOR_TEXT);
    }
  }
}

void redrawRadarPatch(int markerX, int markerY,
                      const TargetMarker markers[TARGET_COUNT]) {
  constexpr int padding = 9;
  const int left = max(0, markerX - padding);
  const int right = min(SCREEN_WIDTH - 1, markerX + padding);
  const int top = max(99, markerY - padding);
  const int bottom = min(RADAR_CY - 1, markerY + padding);
  tft.fillRect(left, top, right - left + 1, bottom - top + 1, COLOR_BG);

  const int ringDegrees[] = {210, 240, 270, 300, 330};
  for (uint8_t ring = 1; ring <= 3; ring++) {
    const int radius = RADAR_RADIUS * ring / 3;
    for (int degree = 210; degree <= 330; degree += 2) {
      const float radians = degree * 0.0174532925f;
      const int x = RADAR_CX + static_cast<int>(radius * cosf(radians));
      const int y = RADAR_CY + static_cast<int>(radius * sinf(radians));
      if (x >= left && x <= right && y >= top && y <= bottom) {
        tft.drawPixel(x, y, COLOR_GRID);
      }
    }
  }
  for (uint8_t i = 0; i < 5; i++) {
    const float radians = ringDegrees[i] * 0.0174532925f;
    drawLineInsidePatch(RADAR_CX, RADAR_CY,
        RADAR_CX + static_cast<int>(RADAR_RADIUS * cosf(radians)),
        RADAR_CY + static_cast<int>(RADAR_RADIUS * sinf(radians)),
        left, top, right, bottom);
  }
  for (int y = max(top, RADAR_CY - 4); y <= min(bottom, RADAR_CY + 4); y++) {
    for (int x = max(left, RADAR_CX - 4); x <= min(right, RADAR_CX + 4); x++) {
      const int dx = x - RADAR_CX;
      const int dy = y - RADAR_CY;
      if (dx * dx + dy * dy <= 16) tft.drawPixel(x, y, COLOR_ACCENT);
    }
  }
  for (uint8_t i = 0; i < TARGET_COUNT; i++) {
    drawMarkerInsidePatch(markers[i], left, top, right, bottom);
  }
}

void drawValue(int x, int y, const char *value, uint16_t color) {
  tft.fillRect(x, y, 54, 10, COLOR_BG);
  tft.setTextColor(color, COLOR_BG);
  tft.setCursor(x, y);
  tft.print(value);
}

void drawTargetValue(const RadarTarget &target, uint8_t index, bool online) {
  const int x = 18 + index * 98;
  char value[16];
  if (online && target.active) {
    const uint32_t rangeDm = (targetRangeMm(target) + 50) / 100;
    snprintf(value, sizeof(value), "%lu.%lum",
        static_cast<unsigned long>(rangeDm / 10),
        static_cast<unsigned long>(rangeDm % 10));
  } else snprintf(value, sizeof(value), "--");
  drawValue(x + 42, 423, value, TARGET_COLORS[index]);

  if (online && target.active) {
    snprintf(value, sizeof(value), "%+d deg", targetAngleDegrees(target));
  } else snprintf(value, sizeof(value), "--");
  drawValue(x + 42, 441, value, TARGET_COLORS[index]);

  if (online && target.active) {
    snprintf(value, sizeof(value), "%+dcm/s", target.velocityCmPerSec);
  } else snprintf(value, sizeof(value), "--");
  drawValue(x + 42, 459, value, TARGET_COLORS[index]);
}

void renderRadar(const RadarReading &reading, uint32_t nowMs) {
  const bool online = reading.valid && nowMs - reading.observedAtMs <= FRAME_TIMEOUT_MS;
  TargetMarker markers[TARGET_COUNT];
  for (uint8_t i = 0; i < TARGET_COUNT; i++) {
    markers[i] = makeTargetMarker(reading.targets[i], online, i);
    if (!uiInitialized || online != lastRadarOnline ||
        !sameTarget(reading.targets[i], lastRenderedReading.targets[i])) {
      drawTargetValue(reading.targets[i], i, online);
    }
  }

  for (uint8_t i = 0; i < TARGET_COUNT; i++) {
    if (!sameMarker(lastRenderedMarkers[i], markers[i])) {
      if (lastRenderedMarkers[i].visible) {
        redrawRadarPatch(lastRenderedMarkers[i].x, lastRenderedMarkers[i].y, markers);
      }
      if (markers[i].visible) redrawRadarPatch(markers[i].x, markers[i].y, markers);
    }
  }
  const uint16_t count = online ? activeTargetCount(reading) : 0;
  static int lastCount = -1;
  if (!uiInitialized || static_cast<int>(count) != lastCount ||
      online != lastRadarOnline) {
    tft.fillRect(78, 371, 62, 14, COLOR_PANEL);
    tft.setTextColor(count ? COLOR_ACCENT : COLOR_MUTED, COLOR_PANEL);
    tft.setCursor(80, 372);
    tft.printf("%u", count);
    tft.fillRect(230, 371, 68, 14, COLOR_PANEL);
    tft.setTextColor(online ? COLOR_ACCENT : COLOR_MUTED, COLOR_PANEL);
    tft.setCursor(232, 372);
    tft.print(online ? "LIVE" : "NO DATA");
    lastCount = count;
  }
  for (uint8_t i = 0; i < TARGET_COUNT; i++) lastRenderedMarkers[i] = markers[i];
  lastRenderedReading = reading;
  lastRadarOnline = online;
  uiInitialized = true;
}

uint8_t estimateSocPercent(float volts) {
  float fraction = (volts - 3.3f) / (4.2f - 3.3f);
  if (fraction < 0.0f) fraction = 0.0f;
  if (fraction > 1.0f) fraction = 1.0f;
  return static_cast<uint8_t>(fraction * 100.0f + 0.5f);
}

void sampleBattery(uint32_t nowMs) {
  uint32_t accumulatedMilliVolts = 0;
  constexpr uint8_t SAMPLE_COUNT = 64;
  for (uint8_t i = 0; i < SAMPLE_COUNT; i++) {
    accumulatedMilliVolts += analogReadMilliVolts(BATTERY_ADC_PIN);
    delayMicroseconds(300);
  }
  batteryPinMilliVolts = accumulatedMilliVolts / SAMPLE_COUNT;
  batteryVoltsEstimate = (batteryPinMilliVolts / 1000.0f) *
                         BATTERY_DIVIDER_RATIO * BATTERY_CALIBRATION;
  batterySocEstimate = estimateSocPercent(batteryVoltsEstimate);
  if (lastDrawnBatterySoc != batterySocEstimate) {
    tft.fillRect(BATTERY_STATUS_X, 0, BATTERY_STATUS_W, STATUS_BAR_HEIGHT, COLOR_PANEL);
    tft.setTextSize(1);
    tft.setTextColor(COLOR_ACCENT, COLOR_PANEL);
    tft.setCursor(BATTERY_STATUS_X + 2, 4);
    tft.printf("BAT %u%%", batterySocEstimate);
    lastDrawnBatterySoc = batterySocEstimate;
  }

  Serial.printf("battery_adc_mv=%u battery_v_est=%.3f soc_est=%u%%\n",
                batteryPinMilliVolts, batteryVoltsEstimate, batterySocEstimate);
  (void)nowMs;
}

void enterDeepSleep() {
  Serial.println("sleep: stopping radar UART");
  RadarSerial.end();
  pinMode(LD2450_TX_PIN, INPUT); // Prevent ESP32 TX from back-powering the radar.
  pinMode(LD2450_RX_PIN, INPUT);
  delay(20);

  Serial.println("sleep: disabling PowerBoost EN on GPIO25");
  digitalWrite(BOOST_EN_PIN, LOW);
  rtc_gpio_init(GPIO_NUM_25);
  rtc_gpio_set_direction(GPIO_NUM_25, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level(GPIO_NUM_25, 0);
  rtc_gpio_hold_en(GPIO_NUM_25);
  delay(50);

  Serial.println("sleep: putting LCD and ESP32 to sleep; press RESET to restart");
  tft.writecommand(0x28); // Display off
  delay(20);
  tft.writecommand(0x10); // Sleep in
  delay(120);
  digitalWrite(LCD_BACKLIGHT_PIN, LOW);
  esp_deep_sleep_start();
}

void handleTouch(uint32_t nowMs) {
  uint16_t x = 0;
  uint16_t y = 0;
  const bool down = tft.getTouch(&x, &y, 600);
  if (down && !touchWasDown) {
    Serial.printf("touch x=%u y=%u\n", x, y);
    if (x >= MENU_TOUCH_X && x < MENU_TOUCH_X + MENU_TOUCH_W &&
        y >= MENU_TOUCH_Y && y < MENU_TOUCH_Y + MENU_TOUCH_H) {
      menuOpen = !menuOpen;
      if (menuOpen) drawMenuUi();
      else {
        for (uint8_t i = 0; i < TARGET_COUNT; i++) lastRenderedMarkers[i] = TargetMarker{};
        uiInitialized = false;
        drawStaticUi();
      }
    } else if (menuOpen && x >= MENU_ITEM_X && x < MENU_ITEM_X + MENU_ITEM_W &&
               y >= MENU_ITEM_Y && y < MENU_ITEM_Y + MENU_ITEM_H) {
      sleepRequested = true;
    }
  }
  touchWasDown = down;
  (void)nowMs;
}

void initializeBoostAndDisplay() {
  // Deep-sleep RTC hold may still be active after the RESET-button reboot.
  rtc_gpio_hold_dis(GPIO_NUM_25);
  rtc_gpio_deinit(GPIO_NUM_25);
  pinMode(BOOST_EN_PIN, OUTPUT);
  digitalWrite(BOOST_EN_PIN, HIGH);
  delay(100);

  pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(LCD_BACKLIGHT_PIN, HIGH);
  tft.init();
  tft.setRotation(0);
  digitalWrite(LCD_BACKLIGHT_PIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.printf("wall-hack firmware v%s | E32R40T LD2450 booting\n", FIRMWARE_VERSION);

  initializeBoostAndDisplay();
  RadarSerial.begin(LD2450_BAUD, SERIAL_8N1, LD2450_RX_PIN, LD2450_TX_PIN);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);

  tft.fillScreen(COLOR_BG);
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  tft.setTextSize(2);
  tft.setCursor(16, 14);
  tft.print("Starting dashboard...");
  loadOrCreateTouchCalibration();

  drawStaticUi();
  sampleBattery(millis());
  uiInitialized = false;
  Serial.println("Ready. Send 'c' over USB serial to recalibrate touch.");
}

void loop() {
  RadarReading frame;
  if (parser.readFrame(RadarSerial, frame)) {
    latestReading = frame;
  }

  if (Serial.available() && Serial.read() == 'c') recalibrateTouch();

  const uint32_t nowMs = millis();
  if (nowMs - lastUiCheckMs >= UI_CHECK_INTERVAL_MS) {
    lastUiCheckMs = nowMs;
    handleTouch(nowMs);
    if (sleepRequested) enterDeepSleep();
    if (!menuOpen) renderRadar(latestReading, nowMs);
  }

  if (nowMs - lastBatterySampleMs >= BATTERY_SAMPLE_INTERVAL_MS) {
    lastBatterySampleMs = nowMs;
    sampleBattery(nowMs);
  }
}
