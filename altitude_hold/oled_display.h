/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║   oled_display.h  --  AltiOS UAV  v1.3                   ║
 * ║   Hardware: OLED M091 12832  (SSD1306 128x32 I2C)        ║
 * ║   Style   : DJI Fly App OSD compact                      ║
 * ╚══════════════════════════════════════════════════════════╝
 *
 * ── Wiring (4 wires only) ──────────────────────────────────
 *
 * OLED  GND  →  Arduino  GND
 * OLED  VCC  →  Arduino  3.3V   ← DO NOT connect to 5V!
 * OLED  SCK  →  Arduino  A5     ← I2C SCL
 * OLED  SDA  →  Arduino  A4     ← I2C SDA
 *
 * ── Library (Tools > Manage Libraries) ─────────────────────
 * Adafruit SSD1306  by Adafruit
 * Adafruit GFX      by Adafruit
 *
 * ── I2C Address ─────────────────────────────────────────────
 * Normally 0x3C  |  If screen is blank, change to 0x3D
 *
 * ── Layout 128x32 (DJI Fly compact) ────────────────────────
 *
 * ┌──────────────────────────────────────────────────────────┐
 * │ [P-SIM]  =HOLD   H: 87.3m            |||||    row 0-10  │
 * ├──────────────────────────────────────────────────────────┤
 * │ V:+0.2m/s          TGT: 100.0m       row 12-20          │
 * ├──────────────────────────────────────────────────────────┤
 * │ E:+12.7m  [──┼──]   67%   +31        row 22-31          │
 * └──────────────────────────────────────────────────────────┘
 *
 * DJI Fly fields mirrored:
 * H:  = Altitude AGL (from ultrasonic)
 * V:  = Vertical speed (calculated from delta alt)
 * TGT = Target / RTH altitude (setpoint)
 * E:  = Error deviation + bar graph
 * mode tag  = P-SIM (test) / ARMED
 * status    = HOLD / UP / RISE / DOWN / DROP
 * signal    = 5-bar icon
 * %         = Throttle % (proxy battery)
 */

#pragma once
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// ─── Config ──────────────────────────────────────────────────
#define OLED_WIDTH 128
#define OLED_HEIGHT 32 // M091 = 32px
#define OLED_ADDR 0x3C // Change to 0x3D if screen is blank

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
bool oledReady = false;

// ─── Vertical speed state ────────────────────────────────────
static double _oled_altPrev = 0.0;
static double _oled_vSpeed = 0.0;
static unsigned long _oled_vsTime = 0;

// ════════════════════════════════════════════════════════════
//  oledInit()  —  Call in setup() after Wire.begin()
// ════════════════════════════════════════════════════════════
void oledInit() {
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(">> OLED not found!  addr=0x" + String(OLED_ADDR, HEX));
    Serial.println("   Try: #define OLED_ADDR 0x3D in oled_display.h");
    oledReady = false;
    return;
  }
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oledReady = true;
  Serial.println(">> OLED OK  128x32  I2C  M091");
}

// ════════════════════════════════════════════════════════════
//  oledBootSplash()  —  Display during LED matrix boot animation
// ════════════════════════════════════════════════════════════
void oledBootSplash() {
  if (!oledReady)
    return;
  oled.clearDisplay();

  oled.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(10, 2);
  oled.print("AltiOS UAV  v1.3");
  oled.setTextColor(SSD1306_WHITE);

  oled.setCursor(4, 14);
  oled.print("Initializing sensors...");
  oled.setCursor(4, 24);
  oled.print("SCK=A5  SDA=A4  0x3C");

  oled.display();
}

// ─── Helper: 5-bar signal icon ───────────────────────────────
static void _drawSignal(int x, int y, int bars) {
  for (int i = 0; i < 5; i++) {
    int h = 2 + i;
    int bx = x + i * 4;
    int by = y + (6 - h);
    if (i < bars)
      oled.fillRect(bx, by, 3, h, SSD1306_WHITE);
    else
      oled.drawRect(bx, by, 3, h, SSD1306_WHITE);
  }
}

// ════════════════════════════════════════════════════════════
//  oledUpdate()  —  Call 10 Hz from loop()
//
//  alt      cm   Current altitude (from ultrasonic)
//  target   cm   Target altitude
//  pidOut        PID output value
//  throttle µs   1100–1700
//  kp,ki,kd      PID gains
//  armed         true = MODE_ARMED
//  sigBars   0–5 (default 5)
// ════════════════════════════════════════════════════════════
void oledUpdate(double alt, double target, double pidOut, int throttle,
                double kp, double ki, double kd, bool armed, int sigBars = 5) {

  if (!oledReady)
    return;

  // Derived
  double altM = alt / 100.0;
  double tgtM = target / 100.0;
  double errCm = target - alt;
  double errM = errCm / 100.0;

  // Vertical speed (EWA smoothed)
  unsigned long now = millis();
  double dt = (now - _oled_vsTime) / 1000.0;
  if (dt >= 0.1 && alt > 0.0) {
    double raw = ((alt - _oled_altPrev) / dt) / 100.0;
    _oled_vSpeed = _oled_vSpeed * 0.6 + raw * 0.4;
    _oled_altPrev = alt;
    _oled_vsTime = now;
  }

  int thrPct = constrain((int)((throttle - 1100) * 100.0 / 600.0), 0, 100);
  bool isHold = (errCm >= -10.0 && errCm <= 10.0);

  oled.clearDisplay();

  // ═══════════════════════════════════════════════════════
  //  ROW 0–10  Top status bar
  // ═══════════════════════════════════════════════════════

  oled.setTextSize(1);

  // Mode tag
  if (armed) {
    oled.fillRect(0, 0, 34, 10, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setCursor(2, 1);
    oled.print("ARMED");
    oled.setTextColor(SSD1306_WHITE);
  } else {
    oled.drawRect(0, 0, 34, 10, SSD1306_WHITE);
    oled.setCursor(2, 1);
    oled.print("P-SIM");
  }

  // Flight status
  if (isHold) {
    oled.fillRect(37, 0, 34, 10, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setCursor(39, 1);
    oled.print("=HOLD");
    oled.setTextColor(SSD1306_WHITE);
  } else if (errCm > 40) {
    oled.setCursor(37, 1);
    oled.print("^RISE");
  } else if (errCm > 10) {
    oled.setCursor(37, 1);
    oled.print("^  UP");
  } else if (errCm < -40) {
    oled.setCursor(37, 1);
    oled.print("vDROP");
  } else {
    oled.setCursor(37, 1);
    oled.print("vDOWN");
  }

  // Altitude  H: xx.xm
  oled.setCursor(75, 1);
  oled.print("H:");
  if (alt <= 0.0) {
    oled.print("--.--");
  } else {
    int intP = (int)altM;
    int decP = abs((int)((altM - intP) * 10));
    if (intP < 10)
      oled.print(" ");
    oled.print(intP);
    oled.print(".");
    oled.print(decP);
    oled.print("m");
  }

  // Signal bars
  _drawSignal(105, 2, sigBars);

  oled.drawLine(0, 11, 127, 11, SSD1306_WHITE);

  // ═══════════════════════════════════════════════════════
  //  ROW 12–20  V-speed  +  Target
  // ═══════════════════════════════════════════════════════

  oled.setCursor(0, 13);
  oled.print("V:");
  if (_oled_vSpeed >= 0)
    oled.print("+");
  oled.print(_oled_vSpeed, 1);
  oled.print("m/s");

  oled.setCursor(66, 13);
  oled.print("TGT:");
  oled.print(tgtM, 1);
  oled.print("m");

  oled.drawLine(0, 21, 127, 21, SSD1306_WHITE);

  // ═══════════════════════════════════════════════════════
  //  ROW 22–31  Error bar  +  Throttle  +  PID
  // ═══════════════════════════════════════════════════════

  // Error value
  oled.setCursor(0, 23);
  oled.print("E:");
  if (errM >= 0)
    oled.print("+");
  oled.print(errM, 1);

  // Center-fill deviation bar
  int bx = 42, bw = 36, bmid = bx + bw / 2;
  oled.drawRect(bx, 23, bw, 7, SSD1306_WHITE);
  int fp = constrain((int)(errCm / 40.0 * ((bw / 2) - 1)), -((bw / 2) - 1),
                     (bw / 2) - 1);
  if (fp > 0)
    oled.fillRect(bmid + 1, 24, fp, 5, SSD1306_WHITE);
  if (fp < 0)
    oled.fillRect(bmid + fp, 24, -fp, 5, SSD1306_WHITE);
  oled.drawLine(bmid, 23, bmid, 29, SSD1306_WHITE);

  // Throttle %
  oled.setCursor(82, 23);
  if (thrPct < 100)
    oled.print(" ");
  oled.print(thrPct);
  oled.print("%");

  // PID output
  oled.setCursor(104, 23);
  if (pidOut >= 0 && (int)pidOut < 100)
    oled.print("+");
  oled.print((int)pidOut);

  oled.display();
}