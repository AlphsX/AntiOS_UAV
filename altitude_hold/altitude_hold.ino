/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║   altitude_hold.ino  --  AltiOS UAV  v1.3                ║
 * ║   Board  : Arduino UNO R4 WiFi                           ║
 * ║   Sensor : HY-SRF05 Ultrasonic                           ║
 * ║   Display: LED Matrix 12x8  +  OLED M091 12832 128x32    ║
 * ╚══════════════════════════════════════════════════════════╝
 *
 * ── Files in the same folder ────────────────────────────────
 * altitude_hold.ino   ← This file
 * animation.h         ← Boot animation for LED matrix
 * oled_display.h      ← OLED HUD (DJI Fly style)
 *
 * ── Required Libraries (Tools > Manage Libraries) ───────────
 * Adafruit SSD1306  by Adafruit
 * Adafruit GFX      by Adafruit
 *
 * ── Wiring ──────────────────────────────────────────────────
 *
 * HY-SRF05 Ultrasonic:
 * VCC  → 5V
 * GND  → GND
 * TRIG → D9
 * ECHO → D10
 *
 * OLED M091 12832 (4-pin I2C):
 * GND  → GND
 * VCC  → 3.3V   ← DO NOT connect to 5V!
 * SCK  → A5     ← I2C SCL
 * SDA  → A4     ← I2C SDA
 *
 * ESC x4 (uncomment [ESC] when motors are connected):
 * ESC1 → D3  |  ESC2 → D5
 * ESC3 → D6  |  ESC4 → D11
 *
 * ── Serial Commands ─────────────────────────────────────────
 * t100    Set target altitude 100 cm
 * p2.5    Set Kp
 * i0.05   Set Ki
 * d1.2    Set Kd
 * stop    Emergency stop ESC
 * status  Print telemetry snapshot
 *
 * ── LED Matrix ──────────────────────────────────────────────
 * ↑↑ fast blink  = go up a lot   (error > 40 cm)
 * ↑  slow blink  = go up a bit   (error 10–40 cm)
 * ■  steady      = HOLD          (error < ±10 cm)
 * ↓  slow blink  = go down a bit (error -10 to -40 cm)
 * ↓↓ fast blink  = go down a lot (error < -40 cm)
 */

// ─── Libraries ──────────────────────────────────────────────
#include "Arduino_LED_Matrix.h"
#include "animation.h"
#include "oled_display.h"
// [ESC] #include <Servo.h>

// ─── Pins ────────────────────────────────────────────────────
#define TRIG_PIN 9
#define ECHO_PIN 10
#define ESC1_PIN 3
#define ESC2_PIN 5
#define ESC3_PIN 6
#define ESC4_PIN 11

// ─── Flight Config ───────────────────────────────────────────
#define TARGET_ALT_CM 100.0 // cm
#define THROTTLE_BASE 1350  // hover throttle µs
#define THROTTLE_MIN 1100
#define THROTTLE_MAX 1700
#define THROTTLE_ARM 1000
#define HOLD_ZONE_CM 10.0

// ─── PID Gains ───────────────────────────────────────────────
double Kp = 2.5;
double Ki = 0.05;
double Kd = 1.2;

// ─── PID State ───────────────────────────────────────────────
double setpoint = TARGET_ALT_CM;
double altCurrent = 0.0;
double pidOutput = 0.0;
double errorPrev = 0.0;
double integral = 0.0;
const double INTEGRAL_MAX = 200.0;

// ─── Timing ──────────────────────────────────────────────────
unsigned long lastPIDTime = 0;
unsigned long lastDisplayTime = 0;
unsigned long lastOLEDTime = 0;
#define PID_INTERVAL_MS 50     // 20 Hz
#define DISPLAY_INTERVAL_MS 80 // 12 Hz  LED matrix
#define OLED_INTERVAL_MS 100   // 10 Hz  OLED

// ─── Altitude Filter ─────────────────────────────────────────
#define FILTER_SIZE 5
double altBuffer[FILTER_SIZE] = {0};
int bufIndex = 0;

// ─── LED Matrix ──────────────────────────────────────────────
ArduinoLEDMatrix matrix;
uint8_t frame[8][12] = {0};

// [ESC] Servo esc1, esc2, esc3, esc4;

// ─── System Mode ─────────────────────────────────────────────
enum SystemMode { MODE_TEST, MODE_ARMED };
SystemMode currentMode = MODE_TEST;

// ─── Throttle cache (shared with OLED) ───────────────────────
int lastThrottle = THROTTLE_BASE;

// ─── LED Sprites 12×8 ────────────────────────────────────────

const uint8_t ARROW_UP[8][12] = {
    {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

const uint8_t ARROW_UP_BIG[8][12] = {
    {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0}, {0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0},
    {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0}, {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0},
    {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0}, {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

const uint8_t ARROW_DOWN[8][12] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0}};

const uint8_t ARROW_DOWN_BIG[8][12] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0}, {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0},
    {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0}, {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0},
    {0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0}, {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0}};

const uint8_t ICON_HOLD[8][12] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0}, {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0}, {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

// ─── Blink ───────────────────────────────────────────────────
bool blinkState = false;
unsigned long lastBlink = 0;

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000)
    ;

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // OLED init (before animation to see splash during boot)
  Wire.begin();
  oledInit();
  oledBootSplash();

  // LED Matrix boot animation (~2.2 seconds)
  matrix.loadSequence(frames);
  matrix.begin();
  matrix.play(false);
  delay(2200);

  // Pre-fill altitude filter
  for (int i = 0; i < FILTER_SIZE; i++) {
    altBuffer[i] = readDistanceCM();
    delay(60);
  }

  // ESC arm sequence (uncomment when real motors are connected)
  // [ESC] esc1.attach(ESC1_PIN, 1000, 2000);
  // [ESC] esc2.attach(ESC2_PIN, 1000, 2000);
  // [ESC] esc3.attach(ESC3_PIN, 1000, 2000);
  // [ESC] esc4.attach(ESC4_PIN, 1000, 2000);
  // [ESC] setAllESC(THROTTLE_ARM);
  // [ESC] delay(3000);
  // [ESC] setAllESC(THROTTLE_BASE);
  // [ESC] delay(500);
  // [ESC] currentMode = MODE_ARMED;

  printBanner();
  lastPIDTime = millis();
}

// ════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── PID 20 Hz ────────────────────────────────────────────
  if (now - lastPIDTime >= PID_INTERVAL_MS) {
    double dt = (now - lastPIDTime) / 1000.0;
    lastPIDTime = now;

    altCurrent = readAltitudeFiltered();

    if (altCurrent > 0 && altCurrent < 400) {
      computePID(dt);
    }

    lastThrottle =
        constrain((int)(THROTTLE_BASE + pidOutput), THROTTLE_MIN, THROTTLE_MAX);

    if (currentMode == MODE_ARMED) {
      setAllESC(lastThrottle);
    }

    printTelemetry(lastThrottle);
  }

  // ── LED Matrix 12 Hz ─────────────────────────────────────
  if (now - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
    lastDisplayTime = now;
    updateLEDMatrix();
  }

  // ── OLED 10 Hz ───────────────────────────────────────────
  if (now - lastOLEDTime >= OLED_INTERVAL_MS) {
    lastOLEDTime = now;
    oledUpdate(altCurrent, setpoint, pidOutput, lastThrottle, Kp, Ki, Kd,
               currentMode == MODE_ARMED);
  }

  handleSerialCommand();
}

// ════════════════════════════════════════════════════════════
//  PID
// ════════════════════════════════════════════════════════════
void computePID(double dt) {
  double error = setpoint - altCurrent;

  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);

  double derivative = (error - errorPrev) / dt;
  errorPrev = error;

  pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
}

// ════════════════════════════════════════════════════════════
//  LED Matrix
// ════════════════════════════════════════════════════════════
void updateLEDMatrix() {
  double error = setpoint - altCurrent;
  unsigned long now = millis();

  const uint8_t(*icon)[12] = ICON_HOLD;
  int blinkRate = 0;

  if (error > 40) {
    icon = ARROW_UP_BIG;
    blinkRate = 150;
  } else if (error > HOLD_ZONE_CM) {
    icon = ARROW_UP;
    blinkRate = 400;
  } else if (error < -40) {
    icon = ARROW_DOWN_BIG;
    blinkRate = 150;
  } else if (error < -HOLD_ZONE_CM) {
    icon = ARROW_DOWN;
    blinkRate = 400;
  } else {
    icon = ICON_HOLD;
    blinkRate = 0;
  }

  if (blinkRate > 0) {
    if (now - lastBlink >= (unsigned long)blinkRate) {
      blinkState = !blinkState;
      lastBlink = now;
    }
  } else {
    blinkState = true;
  }

  if (blinkState)
    memcpy(frame, icon, sizeof(frame));
  else
    memset(frame, 0, sizeof(frame));

  matrix.renderBitmap(frame, 8, 12);
}

// ════════════════════════════════════════════════════════════
//  Sensor
// ════════════════════════════════════════════════════════════
double readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0)
    return -1.0;
  return (dur * 0.0343) / 2.0;
}

double readAltitudeFiltered() {
  double raw = readDistanceCM();
  if (raw > 0 && raw < 400) {
    altBuffer[bufIndex] = raw;
    bufIndex = (bufIndex + 1) % FILTER_SIZE;
  }
  double sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++)
    sum += altBuffer[i];
  return sum / FILTER_SIZE;
}

// ════════════════════════════════════════════════════════════
//  ESC
// ════════════════════════════════════════════════════════════
void setAllESC(int v) {
  // [ESC] esc1.writeMicroseconds(v);
  // [ESC] esc2.writeMicroseconds(v);
  // [ESC] esc3.writeMicroseconds(v);
  // [ESC] esc4.writeMicroseconds(v);
}

// ════════════════════════════════════════════════════════════
//  Serial
// ════════════════════════════════════════════════════════════
void printTelemetry(int throttle) {
  Serial.print("Altitude:");
  Serial.print(altCurrent, 1);
  Serial.print(",Setpoint:");
  Serial.print(setpoint, 1);
  Serial.print(",Error:");
  Serial.print(setpoint - altCurrent, 1);
  Serial.print(",PID:");
  Serial.print(pidOutput, 1);
  Serial.print(",Throttle:");
  Serial.println(throttle);
}

void handleSerialCommand() {
  if (!Serial.available())
    return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("p")) {
    Kp = cmd.substring(1).toDouble();
    Serial.println(">> Kp=" + String(Kp));
  } else if (cmd.startsWith("i")) {
    Ki = cmd.substring(1).toDouble();
    Serial.println(">> Ki=" + String(Ki));
  } else if (cmd.startsWith("d")) {
    Kd = cmd.substring(1).toDouble();
    Serial.println(">> Kd=" + String(Kd));
  } else if (cmd.startsWith("t")) {
    setpoint = cmd.substring(1).toDouble();
    integral = 0;
    Serial.println(">> Target=" + String(setpoint) + "cm");
  } else if (cmd == "stop") {
    setAllESC(THROTTLE_ARM);
    currentMode = MODE_TEST;
    Serial.println(">> EMERGENCY STOP");
  } else if (cmd == "status") {
    Serial.println("────────────────────────────────");
    Serial.println("Mode    : " +
                   String(currentMode == MODE_ARMED ? "ARMED" : "TEST"));
    Serial.println("Alt     : " + String(altCurrent, 1) + " cm");
    Serial.println("Target  : " + String(setpoint, 1) + " cm");
    Serial.println("Error   : " + String(setpoint - altCurrent, 1) + " cm");
    Serial.println("PID out : " + String(pidOutput, 1));
    Serial.println("Throttle: " + String(lastThrottle) + " µs");
    Serial.println("Kp/Ki/Kd: " + String(Kp) + " / " + String(Ki) + " / " +
                   String(Kd));
    Serial.println("────────────────────────────────");
  }
}

void printBanner() {
  Serial.println();
  Serial.println("+==========================================+");
  Serial.println("|   AltiOS UAV  --  Altitude Hold  v1.3    |");
  Serial.println("+------------------------------------------+");
  Serial.println("|  Sensor TRIG=D9  ECHO=D10 (points down)  |");
  Serial.println("|  OLED   SCK=A5   SDA=A4    VCC=3.3V      |");
  Serial.println("|  Target: " + String(TARGET_ALT_CM, 0) +
                 " cm                          |");
  Serial.println("+------------------------------------------+");
  Serial.println("|  CMD: t100  p2.5  i0.05  d1.2  status    |");
  Serial.println("+==========================================+");
  Serial.println();
}