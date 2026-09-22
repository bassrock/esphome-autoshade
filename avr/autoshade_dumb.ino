/*
 * AUTOSHADE dumb-board firmware  v3.0
 * Board 100108 Rev C  (ATmega328P)
 *
 * This firmware contains NO POLICY. It knows nothing about shades, feet,
 * schedules, what "down" means, what a button should do, or what belongs on
 * the screen. All of that lives in ESPHome on an ESP32-S3, which talks to this
 * board over its existing USB-B port through the onboard ATmega16U2.
 *
 * The AVR keeps exactly one job that physically cannot move to the ESP: the
 * real-time step pulse train. Everything else is raw I/O passed through.
 *
 *   owns here          : step timing, step counters, coil phase, raw I2C
 *   owns on the ESP    : positions, travel, speed, hold policy, buttons,
 *                        screen contents, schedule, homing, persistence
 *
 * No EEPROM is used. The ESP holds the authoritative position and pushes it
 * back with X after an AVR reset (opening the USB port pulses DTR, which
 * resets this chip — that is expected and handled).
 *
 * The board itself is not modified. Flashing MyShadesV325.ino back over the
 * same USB port restores the original standalone behaviour.
 *
 * ---------------------------------------------------------------------------
 * Protocol: 115200 8N1, one command per line, '\n' terminated. n = motor 1..6.
 *
 *   V                          -> V AUTOSHADE-DUMB 3.0 N=6
 *   Q                          -> Q P=p1,..,p6 M=<moving> K=<buttons>
 *                                   U=<uptime_s> A=<adc0> B=<adc1>
 *   T n target sps acc endmask -> OK   run to absolute step target, then
 *                                      apply endmask to the coils
 *   S n                        -> OK   decelerating stop (n=0 = all)
 *   X n pos                    -> OK   set the step counter, no motion
 *   C n mask                   -> OK   write a raw coil mask (0..15)
 *   L row text                 -> OK   literal text to LCD row 0 or 1
 *   G v                        -> OK   backlight bits 0..7
 *
 * K= is a sticky button mask, cleared on every Q, so a short press between
 * polls is never lost. Bits follow the Adafruit RGB LCD shield:
 *   bit0 SELECT  bit1 RIGHT  bit2 DOWN  bit3 UP  bit4 LEFT
 *
 * Coil mask bits, per motor: bit0 A_IN2, bit1 A_IN1, bit2 B_IN1, bit3 B_IN2.
 *   0x00 = coast (both inputs low, outputs Hi-Z)
 *   0x0F = brake (both inputs high, windings shorted, zero current draw)
 * ---------------------------------------------------------------------------
 */

#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

#define FW_NAME "AUTOSHADE-DUMB"
#define FW_VER "3.0"
#define N_MOTORS 6

/* ---------------- PCA9685, driven directly ----------------
 *
 * The board is an Adafruit Motor Shield v2 clone, so the channel map is
 * Adafruit's:  port 1 -> M1/M2 = channels 9,10,11,12
 *              port 2 -> M3/M4 = channels 3,4,5,6
 * Both are four contiguous channels, so one step is a single 16-byte
 * auto-incremented write. The shield's PWM enable channels are not connected
 * to anything here — a DRV8871 has only IN1/IN2, no enable — which is why the
 * original firmware could only ever use full-step DOUBLE mode.
 *
 * We do not use Adafruit_MotorShield: its setPin()/setPWM() are private, so
 * brake mode is unreachable through it.
 */

const uint8_t PCA_ADDR[3] = {0x7C, 0x7D, 0x7E};  // motors 1-2, 3-4, 5-6
const uint8_t PORT_BASE[2] = {9, 3};             // first channel of each port

#define PCA_MODE1 0x00
#define PCA_MODE2 0x01
#define PCA_LED0_ON_L 0x06
#define MODE1_AI 0x20      // auto-increment
#define MODE2_OUTDRV 0x04  // totem-pole outputs

/* Two-phase-on full step, the same sequence the original firmware produced
   via onestep(dir, DOUBLE). */
const uint8_t STEP_TABLE[4] = {0x06, 0x05, 0x09, 0x0A};

uint8_t phase[N_MOTORS];
uint8_t endMask[N_MOTORS];

void pcaInit(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(PCA_MODE1);
  Wire.write(MODE1_AI);  // clear SLEEP, enable auto-increment
  Wire.endTransmission();
  delayMicroseconds(500);  // oscillator settle
  Wire.beginTransmission(addr);
  Wire.write(PCA_MODE2);
  Wire.write(MODE2_OUTDRV);
  Wire.endTransmission();
}

/* Full ON = ON_H bit 4 set. Full OFF = OFF_H bit 4 set. 17 bytes total, which
   fits the AVR's 32-byte Wire buffer. */
void writeCoils(uint8_t m, uint8_t mask) {
  Wire.beginTransmission(PCA_ADDR[m / 2]);
  Wire.write(PCA_LED0_ON_L + 4 * PORT_BASE[m % 2]);
  for (uint8_t i = 0; i < 4; i++) {
    bool on = (mask >> i) & 0x01;
    Wire.write((uint8_t)0x00);
    Wire.write(on ? (uint8_t)0x10 : (uint8_t)0x00);
    Wire.write((uint8_t)0x00);
    Wire.write(on ? (uint8_t)0x00 : (uint8_t)0x10);
  }
  Wire.endTransmission();
}

/* Phase is tracked separately from the step counter on purpose: X can move the
   counter without moving the rotor, and deriving phase from position would
   make the next step jump. */
void advance(uint8_t m, int8_t dir) {
  phase[m] = (uint8_t)((phase[m] + dir) & 0x03);
  writeCoils(m, STEP_TABLE[phase[m]]);
}

void f0() { advance(0, 1); }
void b0() { advance(0, -1); }
void f1() { advance(1, 1); }
void b1() { advance(1, -1); }
void f2() { advance(2, 1); }
void b2() { advance(2, -1); }
void f3() { advance(3, 1); }
void b3() { advance(3, -1); }
void f4() { advance(4, 1); }
void b4() { advance(4, -1); }
void f5() { advance(5, 1); }
void b5() { advance(5, -1); }

AccelStepper stp[N_MOTORS] = {
    AccelStepper(f0, b0), AccelStepper(f1, b1), AccelStepper(f2, b2),
    AccelStepper(f3, b3), AccelStepper(f4, b4), AccelStepper(f5, b5),
};

/* ---------------- panel (raw I/O only) ---------------- */

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

uint8_t btnSticky = 0;
unsigned long lastBtn = 0;

/* ---------------- state ---------------- */

int8_t active = -1;  // only one motor steps at a time
char line[56];
uint8_t lineLen = 0;
unsigned long lastSec = 0;
uint32_t uptime = 0;
int adc0 = 0, adc1 = 0;

uint8_t movingMask() {
  uint8_t mask = 0;
  for (uint8_t m = 0; m < N_MOTORS; m++)
    if (stp[m].distanceToGo() != 0) mask |= (1 << m);
  return mask;
}

/* ---------------- serial ---------------- */

void sendVersion() {
  Serial.print(F("V " FW_NAME " " FW_VER " N="));
  Serial.println(N_MOTORS);
}

void sendStatus() {
  Serial.print(F("Q P="));
  for (uint8_t m = 0; m < N_MOTORS; m++) {
    Serial.print(stp[m].currentPosition());
    if (m < N_MOTORS - 1) Serial.print(',');
  }
  Serial.print(F(" M="));
  Serial.print(movingMask());
  Serial.print(F(" K="));
  Serial.print(btnSticky);
  Serial.print(F(" U="));
  Serial.print(uptime);
  Serial.print(F(" A="));
  Serial.print(adc0);
  Serial.print(F(" B="));
  Serial.println(adc1);
  btnSticky = 0;  // sticky mask is consumed by the read
}

/* Returns 0..5, -1 for "all" (0), -2 out of range. */
int8_t parseMotor(const char *s) {
  long n = atol(s);
  if (n == 0) return -1;
  if (n < 1 || n > N_MOTORS) return -2;
  return (int8_t)(n - 1);
}

/* Splits up to 5 space-separated arguments after the command letter. */
uint8_t split(char *s, char *argv[], uint8_t maxArgs) {
  uint8_t n = 0;
  while (*s && n < maxArgs) {
    while (*s == ' ') s++;
    if (!*s) break;
    argv[n++] = s;
    while (*s && *s != ' ') s++;
    if (*s) *s++ = 0;
  }
  return n;
}

void handleLine(char *s) {
  while (*s == ' ') s++;
  char cmd = *s;
  if (cmd >= 'a' && cmd <= 'z') cmd -= 32;
  if (!cmd) return;
  char *rest = s + 1;

  /* L takes free text, so it is handled before generic tokenising. */
  if (cmd == 'L') {
    while (*rest == ' ') rest++;
    if (!*rest) { Serial.println(F("ERR args")); return; }
    uint8_t row = (*rest == '1') ? 1 : 0;
    rest++;
    if (*rest == ' ') rest++;
    lcd.setCursor(0, row);
    uint8_t i = 0;
    for (; rest[i] && i < 16; i++) lcd.write(rest[i]);
    for (; i < 16; i++) lcd.write(' ');
    Serial.println(F("OK"));
    return;
  }

  char *a[5];
  uint8_t n = split(rest, a, 5);

  switch (cmd) {
    case 'V':
      sendVersion();
      return;

    case 'Q':
      sendStatus();
      return;

    case 'T': {  // T n target sps acc endmask
      if (n < 5) { Serial.println(F("ERR args")); return; }
      int8_t m = parseMotor(a[0]);
      if (m < 0) { Serial.println(F("ERR motor")); return; }
      long sps = atol(a[2]);
      long acc = atol(a[3]);
      if (sps < 1) sps = 1;
      if (acc < 1) acc = 1;
      stp[m].setMaxSpeed((float)sps);
      stp[m].setAcceleration((float)acc);
      endMask[m] = (uint8_t)(atol(a[4]) & 0x0F);
      stp[m].moveTo(atol(a[1]));
      Serial.println(F("OK"));
      return;
    }

    case 'S': {
      if (n < 1) { Serial.println(F("ERR args")); return; }
      int8_t m = parseMotor(a[0]);
      if (m == -2) { Serial.println(F("ERR motor")); return; }
      if (m == -1) {
        for (uint8_t i = 0; i < N_MOTORS; i++) stp[i].stop();
      } else {
        stp[m].stop();  // decelerates; the run loop finishes and applies endMask
      }
      Serial.println(F("OK"));
      return;
    }

    case 'X': {
      if (n < 2) { Serial.println(F("ERR args")); return; }
      int8_t m = parseMotor(a[0]);
      if (m < 0) { Serial.println(F("ERR motor")); return; }
      long p = atol(a[1]);
      stp[m].setCurrentPosition(p);  // also zeroes speed
      Serial.println(F("OK"));
      return;
    }

    case 'C': {
      if (n < 2) { Serial.println(F("ERR args")); return; }
      int8_t m = parseMotor(a[0]);
      if (m < 0) { Serial.println(F("ERR motor")); return; }
      writeCoils(m, (uint8_t)(atol(a[1]) & 0x0F));
      Serial.println(F("OK"));
      return;
    }

    case 'G': {
      if (n < 1) { Serial.println(F("ERR args")); return; }
      lcd.setBacklight((uint8_t)(atol(a[0]) & 0x07));
      Serial.println(F("OK"));
      return;
    }

    default:
      Serial.println(F("ERR cmd"));
      return;
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[lineLen] = 0;
      if (lineLen) handleLine(line);
      lineLen = 0;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineLen = 0;  // overlong: drop rather than wrap
    }
  }
}

/* ---------------- setup / loop ---------------- */

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000L);  // 797 steps/s ceiling; 265 at the 100 kHz default

  for (uint8_t i = 0; i < 3; i++) pcaInit(PCA_ADDR[i]);

  for (uint8_t m = 0; m < N_MOTORS; m++) {
    phase[m] = 0;
    endMask[m] = 0x00;  // coast
    writeCoils(m, 0x00);
    stp[m].setMaxSpeed(400);
    stp[m].setAcceleration(400);
  }

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setBacklight(0x7);
  lcd.print(F(FW_NAME));
  lcd.setCursor(0, 1);
  lcd.print(F("v" FW_VER " waiting"));

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  sendVersion();  // banner, in case the host is already listening
}

void loop() {
  pollSerial();

  unsigned long now = millis();

  if (now - lastSec >= 1000) {
    lastSec = now;
    uptime++;
    adc0 = analogRead(A0);
    adc1 = analogRead(A1);
  }

  /* Buttons are sampled even while a motor runs. One read is a handful of
     bytes at 400 kHz; at 20 Hz that is well under 1% of the step budget, and
     a stop button that goes dead during a move is not acceptable. */
  if (now - lastBtn >= 50) {
    lastBtn = now;
    btnSticky |= lcd.readButtons();
  }

  /* One motor at a time: the 400 kHz bus tops out near 797 steps/s total. */
  if (active < 0) {
    for (uint8_t m = 0; m < N_MOTORS; m++) {
      if (stp[m].distanceToGo() != 0) {
        active = (int8_t)m;
        break;
      }
    }
  }

  if (active >= 0) {
    uint8_t m = (uint8_t)active;
    if (stp[m].distanceToGo() != 0) {
      stp[m].run();
    } else {
      writeCoils(m, endMask[m]);
      active = -1;
    }
  }
}
