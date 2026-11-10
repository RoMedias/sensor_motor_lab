/*
  Component test sketch — Arduino Uno
  ------------------------------------
  Serial Monitor: 115200 baud (any line-ending setting works).
  Send a single letter command (h = help).

  Pin map
    A0  Potentiometer wiper           D5  L298 E1-2 (PWM)
    A1  Flex sensor (INPUT_PULLUP)    D6  EasyDriver STEP
    A2  Encoder ch A (pull-up)        D7  EasyDriver DIR
    A3  Encoder ch B (pull-up)        D8  L298 I1
    A4  VL53L1X SDA                   D9  Servo signal
    A5  VL53L1X SCL                   D11 L298 I2
    D4  Omron EE-SX1042 collector     D12 Mode pushbutton
    D0/D1 reserved for USB serial     D2/D3 free

  Library needed: "VL53L1X" by Pololu (Arduino IDE > Library Manager).
*/

#include <Servo.h>
#include <Wire.h>
#include <VL53L1X.h>

// ---------------- Pin map ----------------
const uint8_t PIN_POT      = A0;
const uint8_t PIN_FLEX     = A1;
const uint8_t PIN_ENC_A    = A2;   // PC2 / PCINT10
const uint8_t PIN_ENC_B    = A3;   // PC3 / PCINT11
const uint8_t PIN_SLOT     = 4;
const uint8_t PIN_L298_EN  = 5;
const uint8_t PIN_STEP     = 6;
const uint8_t PIN_DIR      = 7;
const uint8_t PIN_L298_IN1 = 8;
const uint8_t PIN_SERVO    = 9;
const uint8_t PIN_L298_IN2 = 11;
const uint8_t PIN_BUTTON   = 12;

// ---------------- Startup command ----------------
// Command to run automatically at power-up / reset (same letters as the menu).
// e.g. 'p' = live sensor stream, 'm' = DC motor, 's' = stepper, 'v' = servo,
//      't' = ToF, 'i' = interactive, 'r' = read once. Use 0 for none.
const char DEFAULT_COMMAND = 'p';

// ---------------- Test settings ----------------
const int          STEPS_PER_TEST = 1600; // EasyDriver default = 1/8 microstep -> 1600 = 1 rev of a 200-step motor
const unsigned int STEP_DELAY_US  = 600;  // larger = slower stepper
const int          MOTOR_MAX_PWM  = 200;  // 0-255
const uint8_t      TOF_ADDR       = 0x29; // VL53L1X default I2C address

Servo servo;
volatile long    encCount = 0;
volatile uint8_t encPrev  = 0;
bool streaming = false;
unsigned long lastPrint = 0;

// ================= VL53L1X ToF (hardware I2C: SDA=A4, SCL=A5) =================
VL53L1X tof;
bool tofReady = false;
int  tofLastMm = -1;

void tofInit() {
  Wire.begin();
  Wire.setClock(400000);
  tof.setTimeout(500);
  tofReady = tof.init();
  if (!tofReady) {
    Serial.println(F("[ToF] init FAILED - check VIN/GND/SDA(A4)/SCL(A5)."));
    return;
  }
  tof.setDistanceMode(VL53L1X::Long);       // up to ~4 m
  tof.setMeasurementTimingBudget(50000);    // 50 ms per reading
  tof.startContinuous(50);
  Serial.println(F("[ToF] VL53L1X ready."));
}

// Non-blocking: returns the newest reading in mm (-1 = not ready/failed).
int tofReadMm() {
  if (!tofReady) return -1;
  if (tof.dataReady()) {
    tofLastMm = tof.read(false);
    if (tof.ranging_data.range_status != VL53L1X::RangeValid) tofLastMm = -2; // out of range / no target
  }
  return tofLastMm;
}

// ================= Encoder (pin-change interrupt on A2/A3) =================
ISR(PCINT1_vect) {
  static const int8_t QDEC[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
  uint8_t curr = (PINC >> 2) & 0x03;          // bit0 = A2, bit1 = A3
  encCount += QDEC[(encPrev << 2) | curr];
  encPrev = curr;
}

long readEnc() {
  noInterrupts();
  long c = encCount;
  interrupts();
  return c;
}

void resetEnc() {
  noInterrupts();
  encCount = 0;
  interrupts();
}

// ================= Helpers =================
void motorDrive(int speed) {                  // -255..255
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { digitalWrite(PIN_L298_IN1, HIGH); digitalWrite(PIN_L298_IN2, LOW);  }
  else if (speed < 0) { digitalWrite(PIN_L298_IN1, LOW);  digitalWrite(PIN_L298_IN2, HIGH); }
  else                { digitalWrite(PIN_L298_IN1, LOW);  digitalWrite(PIN_L298_IN2, LOW);  }
  analogWrite(PIN_L298_EN, abs(speed));
}

void motorStop() { motorDrive(0); }

void flushInput() {
  delay(20);
  while (Serial.available()) Serial.read();
}

// Any key or a button press aborts a running test (and stops the DC motor).
bool abortRequested() {
  if (Serial.available()) {
    motorStop();
    flushInput();
    return true;
  }
  if (digitalRead(PIN_BUTTON) == LOW) {
    motorStop();
    delay(30);
    while (digitalRead(PIN_BUTTON) == LOW) {}
    delay(30);
    return true;
  }
  return false;
}

bool waitOrAbort(unsigned long ms) {
  unsigned long t0 = millis();
  do {
    if (abortRequested()) return true;
  } while (millis() - t0 < ms);
  return false;
}

bool buttonPressedEdge() {
  static bool stable = HIGH, lastRead = HIGH;
  static unsigned long tChange = 0;
  bool r = digitalRead(PIN_BUTTON);
  if (r != lastRead) { lastRead = r; tChange = millis(); }
  if (millis() - tChange > 30 && r != stable) {
    stable = r;
    if (stable == LOW) return true;
  }
  return false;
}

// ================= Sensor readout =================
void printSensors() {
  Serial.print(F("pot="));      Serial.print(analogRead(PIN_POT));
  Serial.print(F("  flex="));   Serial.print(analogRead(PIN_FLEX));
  Serial.print(F("  slot="));   Serial.print(digitalRead(PIN_SLOT) ? F("BLOCKED") : F("clear"));
  Serial.print(F("  button=")); Serial.print(digitalRead(PIN_BUTTON) ? F("up") : F("PRESSED"));
  Serial.print(F("  enc="));    Serial.print(readEnc());
  Serial.print(F("  AB="));     Serial.print(digitalRead(PIN_ENC_A)); Serial.print(digitalRead(PIN_ENC_B));
  Serial.print(F("  tof_mm=")); Serial.println(tofReadMm());
}

// ================= DC motor + encoder =================
bool motorPhase(int dir) {
  for (int s = 0; s <= MOTOR_MAX_PWM; s += 10) { motorDrive(dir * s); if (waitOrAbort(40)) return false; }
  if (waitOrAbort(1500)) return false;
  for (int s = MOTOR_MAX_PWM; s >= 0; s -= 10) { motorDrive(dir * s); if (waitOrAbort(40)) return false; }
  return !waitOrAbort(500);
}

void testDcMotor() {
  flushInput();
  Serial.println(F("\n[DC motor] Ramp forward, then reverse. Any key/button aborts."));
  for (int dir = 1; dir >= -1; dir -= 2) {
    long c0 = readEnc();
    if (!motorPhase(dir)) { Serial.println(F("  Aborted.")); return; }
    Serial.print(dir > 0 ? F("  forward: ") : F("  reverse: "));
    Serial.print(readEnc() - c0);
    Serial.println(F(" encoder counts"));
  }
  Serial.println(F("  Expect large counts with opposite signs. 0 = check encoder power/wiring."));
}

// ================= Stepper =================
void testStepper() {
  flushInput();
  Serial.println(F("\n[Stepper] 1 rev one way, 1 rev back. Any key/button aborts."));
  for (uint8_t d = 0; d < 2; d++) {
    digitalWrite(PIN_DIR, d == 0 ? LOW : HIGH);
    delay(5);
    for (int i = 0; i < STEPS_PER_TEST; i++) {
      digitalWrite(PIN_STEP, HIGH); delayMicroseconds(5);
      digitalWrite(PIN_STEP, LOW);  delayMicroseconds(STEP_DELAY_US);
      if ((i & 0x3F) == 0 && abortRequested()) { Serial.println(F("  Aborted.")); return; }
    }
    Serial.println(d == 0 ? F("  direction 1 done") : F("  direction 2 done"));
    if (waitOrAbort(300)) { Serial.println(F("  Aborted.")); return; }
  }
  Serial.println(F("  Buzzing/jitter without smooth rotation = coil pairs wrong."));
}

// ================= Servo =================
void testServo() {
  flushInput();
  Serial.println(F("\n[Servo] 90 -> 0 -> 180 -> 90. Any key/button aborts."));
  const uint8_t pts[] = {90, 0, 180, 90};
  int pos = servo.read();
  for (uint8_t k = 0; k < 4; k++) {
    int target = pts[k];
    while (pos != target) {
      pos += (target > pos) ? 1 : -1;
      servo.write(pos);
      if (waitOrAbort(10)) { Serial.println(F("  Aborted.")); return; }
    }
    Serial.print(F("  at ")); Serial.println(pos);
    if (waitOrAbort(500)) { Serial.println(F("  Aborted.")); return; }
  }
}

// ================= ToF test =================
void testTof() {
  flushInput();
  Serial.println(F("\n[ToF] I2C scan (A4/A5)..."));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  device at 0x")); Serial.print(addr, HEX);
      Serial.println(addr == TOF_ADDR ? F("  <- VL53L1X") : F(""));
      found++;
    }
  }
  if (!found) {
    Serial.println(F("  Nothing found. Check VIN, GND, SDA->A4, SCL->A5. XSHUT must not be held LOW."));
    return;
  }

  if (!tofReady) tofInit();
  if (!tofReady) return;

  Serial.println(F("  20 readings (-2 = no valid target). Any key/button aborts."));
  for (uint8_t i = 0; i < 20; i++) {
    unsigned long t0 = millis();
    while (!tof.dataReady() && millis() - t0 < 200) {}
    Serial.print(F("  distance_mm = ")); Serial.println(tofReadMm());
    if (waitOrAbort(100)) return;
  }
}

// ================= Interactive: pot drives servo + DC motor =================
void interactive() {
  flushInput();
  Serial.println(F("\n[Interactive] Pot -> servo angle and DC motor speed. Any key/button exits."));
  unsigned long tPrint = 0;
  while (true) {
    int pot = analogRead(PIN_POT);
    int angle = map(pot, 0, 1023, 0, 180);
    int speed = map(pot, 0, 1023, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
    if (abs(speed) < 25) speed = 0;                          // dead band at center
    servo.write(angle);
    motorDrive(speed);
    if (millis() - tPrint > 250) {
      tPrint = millis();
      Serial.print(F("  pot=")); Serial.print(pot);
      Serial.print(F("  servo=")); Serial.print(angle);
      Serial.print(F("  motor=")); Serial.print(speed);
      Serial.print(F("  enc=")); Serial.println(readEnc());
    }
    if (waitOrAbort(20)) break;
  }
  motorStop();
  Serial.println(F("  Exited."));
}

// ================= Menu =================
void printHelp() {
  Serial.println(F("\n=== Component test ==="));
  Serial.println(F(" r  read all sensors once"));
  Serial.println(F(" p  toggle live sensor stream (button also toggles)"));
  Serial.println(F(" e  reset encoder count"));
  Serial.println(F(" m  DC motor + encoder test"));
  Serial.println(F(" s  stepper test"));
  Serial.println(F(" v  servo sweep"));
  Serial.println(F(" t  ToF: I2C scan + 20 distance readings"));
  Serial.println(F(" i  interactive: pot drives servo + DC motor"));
  Serial.println(F(" x  stop DC motor"));
  Serial.println(F(" h  help"));
}

void handleCommand(char c) {
  switch (tolower(c)) {
    case 'h': printHelp(); break;
    case 'r': printSensors(); break;
    case 'p': streaming = !streaming;
              Serial.println(streaming ? F("Stream ON") : F("Stream OFF")); break;
    case 'e': resetEnc(); Serial.println(F("Encoder reset.")); break;
    case 'm': testDcMotor(); break;
    case 's': testStepper(); break;
    case 'v': testServo(); break;
    case 't': testTof(); break;
    case 'i': interactive(); break;
    case 'x': motorStop(); Serial.println(F("Motor stopped.")); break;
    case '\r': case '\n': case ' ': break;
    default:  Serial.println(F("Unknown command (h = help)"));
  }
}

// ================= Setup / loop =================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_FLEX,   INPUT_PULLUP);
  pinMode(PIN_ENC_A,  INPUT_PULLUP);
  pinMode(PIN_ENC_B,  INPUT_PULLUP);
  pinMode(PIN_SLOT,   INPUT_PULLUP);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  pinMode(PIN_L298_EN,  OUTPUT);
  pinMode(PIN_L298_IN1, OUTPUT);
  pinMode(PIN_L298_IN2, OUTPUT);
  motorStop();

  servo.attach(PIN_SERVO);
  servo.write(90);

  // Pin-change interrupts on A2 (PCINT10) and A3 (PCINT11)
  encPrev = (PINC >> 2) & 0x03;
  PCMSK1 |= _BV(PCINT10) | _BV(PCINT11);
  PCIFR  |= _BV(PCIF1);
  PCICR  |= _BV(PCIE1);

  tofInit();
  printHelp();

  if (DEFAULT_COMMAND) {
    Serial.print(F("\nRunning default command: "));
    Serial.println(DEFAULT_COMMAND);
    handleCommand(DEFAULT_COMMAND);
  }
}

void loop() {
  if (Serial.available()) handleCommand(Serial.read());

  if (buttonPressedEdge()) {
    streaming = !streaming;
    Serial.println(streaming ? F("[button] Stream ON") : F("[button] Stream OFF"));
  }

  if (streaming && millis() - lastPrint >= 200) {
    lastPrint = millis();
    printSensors();
  }
}
