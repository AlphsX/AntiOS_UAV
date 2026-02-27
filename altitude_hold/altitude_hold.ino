/*
 * ╔══════════════════════════════════════════════════════╗
 * ║   Drone Altitude Hold -- Arduino UNO R4 WiFi         ║
 * ║   Sensor  : HY-SRF05 Ultrasonic                      ║
 * ║   Display : LED Matrix 12x8 -- AltiOS UAV            ║
 * ╚══════════════════════════════════════════════════════╝
 *
 * Files required in the same folder:
 *   altitude_hold.ino   <- This file
 *   animation.h         <- Boot animation frames
 *
 * How it works:
 *   1. Power on -> Boot animation plays (~1 second)
 *   2. Animation ends -> PID loop starts, arrows/square displayed
 *
 * PIN MAP:
 *   TRIG -> D9  |  ECHO -> D10
 *   ESC1 -> D3  |  ESC2 -> D5  (uncomment [ESC] when motors connected)
 *   ESC3 -> D6  |  ESC4 -> D11
 *
 * Serial Commands:
 *   t100  -> Set target altitude to 100cm
 *   p2.5  -> Set Kp
 *   i0.05 -> Set Ki
 *   d1.2  -> Set Kd
 *   stop  -> Stop ESC (emergency)
 *   status -> Print all current values
 *
 * LED Matrix behavior:
 *   double-up  fast blink = go up a lot     (error > 40cm)
 *   single-up  slow blink = go up a little  (error 10-40cm)
 *   square     steady on  = HOLD            (error < 10cm)
 *   single-dn  slow blink = go down little  (error -10 to -40cm)
 *   double-dn  fast blink = go down a lot   (error < -40cm)
 */

// ─── Libraries ─────────────────────────────────────────
#include "Arduino_LED_Matrix.h"
#include "animation.h"
// [ESC] #include <Servo.h>

// ─── Pins ──────────────────────────────────────────────
#define TRIG_PIN   9
#define ECHO_PIN   10
#define ESC1_PIN   3
#define ESC2_PIN   5
#define ESC3_PIN   6
#define ESC4_PIN   11

// ─── Flight Config ─────────────────────────────────────
#define TARGET_ALT_CM   100.0   // Target altitude in cm
#define THROTTLE_BASE   1350    // Hover throttle in microseconds
#define THROTTLE_MIN    1100    // Minimum throttle
#define THROTTLE_MAX    1700    // Maximum throttle
#define THROTTLE_ARM    1000    // ESC arm signal
#define HOLD_ZONE_CM    10.0    // +/- cm considered "holding"

// ─── PID Gains (tune these for your drone) ─────────────
double Kp = 2.5;
double Ki = 0.05;
double Kd = 1.2;

// ─── PID State ─────────────────────────────────────────
double setpoint   = TARGET_ALT_CM;
double altCurrent = 0.0;
double pidOutput  = 0.0;
double errorPrev  = 0.0;
double integral   = 0.0;
const double INTEGRAL_MAX = 200.0;  // Anti-windup limit

// ─── Loop Timing ───────────────────────────────────────
unsigned long lastPIDTime     = 0;
unsigned long lastDisplayTime = 0;
#define PID_INTERVAL_MS     50   // 20Hz PID loop
#define DISPLAY_INTERVAL_MS 80   // 12Hz display refresh

// ─── Altitude Moving Average Filter ────────────────────
#define FILTER_SIZE 5
double altBuffer[FILTER_SIZE] = {0};
int    bufIndex = 0;

// ─── LED Matrix ────────────────────────────────────────
ArduinoLEDMatrix matrix;
uint8_t frame[8][12] = {0};

// [ESC] Servo esc1, esc2, esc3, esc4;

// ─── System Mode ───────────────────────────────────────
enum SystemMode { MODE_TEST, MODE_ARMED };
SystemMode currentMode = MODE_TEST;

// ─── Sprites (12 cols x 8 rows) ────────────────────────

// Single up arrow
const uint8_t ARROW_UP[8][12] = {
  {0,0,0,0,0,1,0,0,0,0,0,0},
  {0,0,0,0,1,1,1,0,0,0,0,0},
  {0,0,0,1,1,1,1,1,0,0,0,0},
  {0,0,0,0,0,1,0,0,0,0,0,0},
  {0,0,0,0,0,1,0,0,0,0,0,0},
  {0,0,0,0,0,1,0,0,0,0,0,0},
  {0,0,0,0,0,1,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0}
};

// Double up arrows
const uint8_t ARROW_UP_BIG[8][12] = {
  {0,0,0,1,0,0,0,1,0,0,0,0},
  {0,0,1,1,1,0,1,1,1,0,0,0},
  {0,0,0,1,0,0,0,1,0,0,0,0},
  {0,0,0,1,0,0,0,1,0,0,0,0},
  {0,0,0,1,0,0,0,1,0,0,0,0},
  {0,0,0,1,0,0,0,1,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0}
};

// Single down arrow
const uint8_t ARROW_DOWN[8][12] = {
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,1,0,0,0,0,0,0},
  {0,0,0,0,0,1,0,0,0,0,0,0},
  {0,0,0,0,0,1,0,0,0,0,0,0},
  {0,0,0,0,0,1,0,0,0,0,0,0},
  {0,0,0,1,1,1,1,1,0,0,0,0},
  {0,0,0,0,1,1,1,0,0,0,0,0},
  {0,0,0,0,0,1,0,0,0,0,0,0}
};

// Double down arrows
const uint8_t ARROW_DOWN_BIG[8][12] = {
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,1,0,0,0,1,0,0,0,0},
  {0,0,0,1,0,0,0,1,0,0,0,0},
  {0,0,0,1,0,0,0,1,0,0,0,0},
  {0,0,0,1,0,0,0,1,0,0,0,0},
  {0,0,1,1,1,0,1,1,1,0,0,0},
  {0,0,0,1,0,0,0,1,0,0,0,0}
};

// Square frame (HOLD)
const uint8_t ICON_HOLD[8][12] = {
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,1,1,1,1,1,1,1,0,0,0},
  {0,0,1,1,1,1,1,1,1,0,0,0},
  {0,0,1,1,1,1,1,1,1,0,0,0},
  {0,0,1,1,1,1,1,1,1,0,0,0},
  {0,0,1,1,1,1,1,1,1,0,0,0},
  {0,0,1,1,1,1,1,1,1,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0}
};

// ─── Blink State ───────────────────────────────────────
bool blinkState = false;
unsigned long lastBlink = 0;

//  SETUP
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // ── Boot Animation ─────────────────────────────────
  // Same pattern as Arduino heart animation example:
  //   loadSequence -> begin -> play(false) -> delay
  matrix.loadSequence(frames);
  matrix.begin();
  matrix.play(false);
  delay(2200);  // radar scan ~2.2 seconds
  //    80ms x 7 rows + 200ms all-on + 80ms x 4 flash = 1040ms
  //    using 1200ms to be safe
  // ───────────────────────────────────────────────────

  // Pre-fill altitude buffer with real readings
  for (int i = 0; i < FILTER_SIZE; i++) {
    altBuffer[i] = readDistanceCM();
    delay(60);
  }

  // ── ESC Arm Sequence (uncomment when motors connected) ──
  // [ESC] esc1.attach(ESC1_PIN, 1000, 2000);
  // [ESC] esc2.attach(ESC2_PIN, 1000, 2000);
  // [ESC] esc3.attach(ESC3_PIN, 1000, 2000);
  // [ESC] esc4.attach(ESC4_PIN, 1000, 2000);
  // [ESC] setAllESC(THROTTLE_ARM);
  // [ESC] delay(3000);                  // wait for ESC to arm
  // [ESC] setAllESC(THROTTLE_BASE);
  // [ESC] delay(500);
  // [ESC] currentMode = MODE_ARMED;

  printBanner();
  lastPIDTime = millis();
}

//  LOOP
void loop() {
  unsigned long now = millis();

  // ── PID Loop 20Hz ──────────────────────────────────
  if (now - lastPIDTime >= PID_INTERVAL_MS) {
    double dt = (now - lastPIDTime) / 1000.0;
    lastPIDTime = now;

    altCurrent = readAltitudeFiltered();

    if (altCurrent > 0 && altCurrent < 400) {
      computePID(dt);
    }

    int throttle = constrain(
      (int)(THROTTLE_BASE + pidOutput),
      THROTTLE_MIN,
      THROTTLE_MAX
    );

    if (currentMode == MODE_ARMED) {
      setAllESC(throttle);
    }

    printTelemetry(throttle);
  }

  // ── Display Loop 12Hz ──────────────────────────────
  if (now - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
    lastDisplayTime = now;
    updateDisplay();
  }

  handleSerialCommand();
}

//  PID Controller
void computePID(double dt) {
  double error = setpoint - altCurrent;

  // Integral with anti-windup
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);

  // Derivative
  double derivative = (error - errorPrev) / dt;
  errorPrev = error;

  pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
}

//  LED Matrix Display
void updateDisplay() {
  double error = setpoint - altCurrent;
  unsigned long now = millis();

  const uint8_t (*icon)[12] = ICON_HOLD;
  int blinkRate = 0;

  if      (error >  40)           { icon = ARROW_UP_BIG;   blinkRate = 150; }
  else if (error >  HOLD_ZONE_CM) { icon = ARROW_UP;       blinkRate = 400; }
  else if (error < -40)           { icon = ARROW_DOWN_BIG; blinkRate = 150; }
  else if (error < -HOLD_ZONE_CM) { icon = ARROW_DOWN;     blinkRate = 400; }
  else                            { icon = ICON_HOLD;      blinkRate = 0;   }

  if (blinkRate > 0) {
    if (now - lastBlink >= (unsigned long)blinkRate) {
      blinkState = !blinkState;
      lastBlink  = now;
    }
  } else {
    blinkState = true;  // HOLD = steady on, no blink
  }

  if (blinkState) memcpy(frame, icon, sizeof(frame));
  else            memset(frame, 0,    sizeof(frame));

  matrix.renderBitmap(frame, 8, 12);
}

//  Sensor
double readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return -1.0;
  return (dur * 0.0343) / 2.0;
}

double readAltitudeFiltered() {
  double raw = readDistanceCM();
  if (raw > 0 && raw < 400) {
    altBuffer[bufIndex] = raw;
    bufIndex = (bufIndex + 1) % FILTER_SIZE;
  }
  double sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) sum += altBuffer[i];
  return sum / FILTER_SIZE;
}

//  ESC
void setAllESC(int v) {
  // [ESC] esc1.writeMicroseconds(v);
  // [ESC] esc2.writeMicroseconds(v);
  // [ESC] esc3.writeMicroseconds(v);
  // [ESC] esc4.writeMicroseconds(v);
}

//  Serial Output
void printTelemetry(int throttle) {
  Serial.print("Altitude:");  Serial.print(altCurrent, 1);
  Serial.print(",Setpoint:"); Serial.print(setpoint, 1);
  Serial.print(",Error:");    Serial.print(setpoint - altCurrent, 1);
  Serial.print(",PID:");      Serial.print(pidOutput, 1);
  Serial.print(",Throttle:"); Serial.println(throttle);
}

void handleSerialCommand() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if      (cmd.startsWith("p")) {
    Kp = cmd.substring(1).toDouble();
    Serial.println(">> Kp=" + String(Kp));
  }
  else if (cmd.startsWith("i")) {
    Ki = cmd.substring(1).toDouble();
    Serial.println(">> Ki=" + String(Ki));
  }
  else if (cmd.startsWith("d")) {
    Kd = cmd.substring(1).toDouble();
    Serial.println(">> Kd=" + String(Kd));
  }
  else if (cmd.startsWith("t")) {
    setpoint = cmd.substring(1).toDouble();
    integral = 0;  // reset integral on target change
    Serial.println(">> Target=" + String(setpoint) + "cm");
  }
  else if (cmd == "stop") {
    setAllESC(THROTTLE_ARM);
    currentMode = MODE_TEST;
    Serial.println(">> EMERGENCY STOP");
  }
  else if (cmd == "status") {
    Serial.println("────────────────────────────────");
    Serial.println("Mode   : " + String(currentMode == MODE_ARMED ? "ARMED" : "TEST"));
    Serial.println("Alt    : " + String(altCurrent, 1) + " cm");
    Serial.println("Target : " + String(setpoint, 1)  + " cm");
    Serial.println("Error  : " + String(setpoint - altCurrent, 1) + " cm");
    Serial.println("PID    : " + String(pidOutput, 1));
    Serial.println("Kp/Ki/Kd: " + String(Kp) + " / " + String(Ki) + " / " + String(Kd));
    Serial.println("────────────────────────────────");
  }
}

//  Startup Banner
void printBanner() {
  Serial.println();
  Serial.println("+========================================+");
  Serial.println("|   AltiOS UAV  --  Altitude Hold        |");
  Serial.println("+----------------------------------------+");
  Serial.println("|  Sensor head pointing DOWN always      |");
  Serial.println("|  Target: " + String(TARGET_ALT_CM, 0) + " cm                        |");
  Serial.println("+----------------------------------------+");
  Serial.println("|  LED:  up-up  fast = go up a lot       |");
  Serial.println("|        up     slow = go up a little    |");
  Serial.println("|        square      = HOLD              |");
  Serial.println("|        down   slow = go down a little  |");
  Serial.println("|        dn-dn  fast = go down a lot     |");
  Serial.println("+----------------------------------------+");
  Serial.println("|  CMD:  t100 p2.5 i0.05 d1.2 status     |");
  Serial.println("+========================================+");
  Serial.println();
}
