/*
 * AUTOSHADE dumb-board firmware  v3.3
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
 * v3.3: HALF-STEP DRIVE. Full steps at 250+ steps/s stall these motors and
 * grind; half steps at ~90 full steps/s run cleanly and quietly. The wire
 * protocol stays in FULL steps (T/X targets, speeds, accels and Q positions);
 * the board doubles them internally. Also:
 *   - the I2C clock is set to 400 kHz again after lcd.begin(), which had
 *     silently reset it to 100 kHz (each coil write ate most of a step).
 *   - buttons are sampled with one GPIOA read instead of readButtons()' five.
 *   - EEPROM records are now in half steps with a 0..7 phase, under a new
 *     magic; a v3.2 record reads as blank (E=0) and the ESP pushes its copy.
 *
 * v3.2: THIS BOARD IS NOW THE SOURCE OF TRUTH FOR POSITION. Step counters,
 * coil phase and the at-rest coil mask are written to EEPROM whenever a motor
 * comes to rest, and on every X. They are read back at boot, so a reset —
 * including the one the USB host causes by asserting DTR — no longer loses
 * anything. The ESP reads positions on connect instead of pushing them.
 *   - two A/B slots with a sequence number and CRC: a write torn by a power
 *     cut leaves the other slot intact.
 *   - stored at 0x200, clear of the stock firmware's data at the bottom of
 *     EEPROM, which R can still read out.
 *   - EEPROM.put only rewrites bytes that changed. ~10 moves a day is decades
 *     of the 100k-cycle rating.
 * Also new in v3.2:
 *   - 12 V interlock: if the motor supply drops while a motor is stepping, all
 *     motion is cancelled on the spot. Without VM the DRV8871s are off, so
 *     every further "step" would be counted but never happen and the saved
 *     position would be a lie. Reported as F=1 until the next T.
 *   - R addr len: raw EEPROM read, so the stock firmware's settings can be
 *     dumped from ESPHome without a laptop.
 *
 * v3.1: the ESP can also connect through the ICSP header J3 with three jumper
 * wires. That path is compiled out by default (LINK_J3 0) because, with
 * nothing plugged into J3, the floating MOSI pin would feed noise into the
 * command parser. Set LINK_J3 to 1 only if you actually wire J3.
 *
 *   J3-1 MISO (PB4, D12)  -> ESP RX   open-drain: pulled LOW or left floating,
 *                                     never driven to 5 V. The ESP's own
 *                                     pull-up to 3.3 V makes the high level,
 *                                     so no level shifter is needed.
 *   J3-4 MOSI (PB3, D11)  <- ESP TX   3.3 V clears the AVR's 3.0 V threshold.
 *   J3-6 GND              -- ESP GND
 *   J3-2 VCC, J3-3 SCK, J3-5 RESET: leave unconnected.
 *
 *   38400 8N1. The link is half-duplex by protocol: the ESP sends one command
 *   and waits for its one-line reply before sending the next.
 *
 * The board itself is not modified. Flashing MyShadesV325.ino back over the
 * same USB port restores the original standalone behaviour.
 *
 * ---------------------------------------------------------------------------
 * Protocol: 115200 8N1, one command per line, '\n' terminated. n = motor 1..6.
 * All positions, speeds and accelerations on the wire are FULL steps (200/rev);
 * the board half-steps internally (see HALF below).
 *
 *   V                          -> V AUTOSHADE-DUMB 3.3 N=6
 *   Q                          -> Q P=p1,..,p6 M=<moving> K=<buttons>
 *                                   U=<uptime_s> A=<adc0> B=<adc1>
 *                                   E=<0|1> F=<0|1>
 *        E=1: P= is real — restored from EEPROM at boot, or set by X since.
 *        E=0: blank EEPROM, counters are just zeros. Push positions with X.
 *        F=1: the last move was cancelled because 12 V dropped out.
 *   T n target sps acc endmask -> OK   run to absolute step target, then
 *                                      apply endmask to the coils
 *   S n                        -> OK   decelerating stop (n=0 = all)
 *   X n pos                    -> OK   set the step counter, no motion
 *   C n mask                   -> OK   write a raw coil mask (0..15)
 *   L row text                 -> OK   literal text to LCD row 0 or 1
 *   G v                        -> OK   backlight bits 0..7
 *   R addr len                 -> R addr hh hh ..  raw EEPROM bytes (len<=16)
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
#include <EEPROM.h>

#define N_MOTORS 6

/* EEPROM record (defined up here so Arduino's auto-prototypes can see it). */
struct Saved {
  uint16_t magic;
  uint16_t seq;
  int32_t pos[N_MOTORS];
  uint8_t phase[N_MOTORS];
  uint8_t endMask[N_MOTORS];
  uint8_t crc;
};


#ifndef LINK_J3
#define LINK_J3 0  // 1 = also accept commands on the ICSP header J3 (see top)
#endif

#if LINK_J3
#include <SoftwareSerial.h>
#include <util/delay.h>
#endif

/* ---------------- ICSP-header link (J3) ----------------
 *
 * RX uses SoftwareSerial on D11 (PB3/MOSI). Its TX pin argument is a dummy:
 * D13 (SCK) only feeds the heartbeat-LED buffer on this board. We must NOT let
 * SoftwareSerial own D12, because it would drive the line to 5 V and the
 * ESP32 is not 5 V tolerant.
 *
 * TX is our own open-drain bit-banger on D12 (PB4/MISO): a 0 bit pulls the line
 * low, a 1 bit releases it and the ESP's pull-up lifts it to 3.3 V. One byte is
 * sent per loop() pass so the step train is never held off for more than one
 * byte time (~260 us at 38400).
 */
#if LINK_J3
#define LINK_BAUD 38400
#define LINK_BIT_US (1000000.0 / LINK_BAUD)
SoftwareSerial linkRx(11, 13);

#define OD_PIN_BIT 4  // PB4 = D12 = MISO = J3 pin 1
static inline void odLow() { DDRB |= _BV(OD_PIN_BIT); }
static inline void odRelease() { DDRB &= ~_BV(OD_PIN_BIT); }

uint8_t odBuf[192];
uint8_t odHead = 0, odTail = 0;

void odSendByte(uint8_t b) {
  uint8_t sreg = SREG;
  cli();
  odLow();
  _delay_us(LINK_BIT_US);  // start bit
  for (uint8_t i = 0; i < 8; i++) {
    if (b & 0x01) odRelease(); else odLow();
    b >>= 1;
    _delay_us(LINK_BIT_US);
  }
  odRelease();
  _delay_us(LINK_BIT_US);  // stop bit
  SREG = sreg;
}

/* Everything the firmware prints goes to both the USB serial port and the
   ICSP link, so either can be used — or both, e.g. a laptop watching. */
class MirrorOut : public Print {
 public:
  size_t write(uint8_t c) override {
    Serial.write(c);
    uint8_t next = (uint8_t)((odHead + 1) % sizeof(odBuf));
    if (next != odTail) {  // drop on overflow rather than block
      odBuf[odHead] = c;
      odHead = next;
    }
    return 1;
  }
};
MirrorOut out;
#else
Print &out = Serial;
#endif

#define FW_NAME "AUTOSHADE-DUMB"
#define FW_VER "3.3"

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

/* Half-step drive: two-phase-on full steps {0x06, 0x05, 0x09, 0x0A} with a
   single-coil state between each pair. Even entries are exactly those full
   steps, so full-step positions line up. The order keeps this firmware's
   v3.2 direction; if a shade jogs the wrong way, reverse it
   ({0x05, 0x04, 0x06, 0x02, 0x0A, 0x08, 0x09, 0x01} is the other way). */
const uint8_t HALF_TABLE[8] = {0x06, 0x04, 0x05, 0x01, 0x09, 0x08, 0x0A, 0x02};
#define HALF 2  // AccelStepper runs in half steps; the protocol stays in full steps

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
  phase[m] = (uint8_t)((phase[m] + dir) & 0x07);
  writeCoils(m, HALF_TABLE[phase[m]]);
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

/* One GPIOA read instead of readButtons()' five register reads, so sampling
   the panel mid-move costs ~0.1 ms rather than several. The buttons are on
   GPA0-GPA4, active low, in readButtons()' bit order. */
uint8_t readButtonsFast() {
  Wire.beginTransmission(0x20);
  Wire.write((uint8_t)0x12);  // MCP23017 GPIOA (IOCON.BANK = 0)
  if (Wire.endTransmission() != 0) return 0;
  if (Wire.requestFrom((uint8_t)0x20, (uint8_t)1) != 1) return 0;
  return (uint8_t)(~Wire.read()) & 0x1F;
}

/* ---------------- state ---------------- */

int8_t active = -1;  // only one motor steps at a time
char line[56];
uint8_t lineLen = 0;
unsigned long lastSec = 0;
uint32_t uptime = 0;
int adc0 = 0, adc1 = 0;

/* ---------------- persistence ---------------- */

#define EE_BASE 0x200
#define EE_SLOT 0x40
#define EE_MAGIC 0x5A54  // v3.3: half steps. v3.2's 0x5A53 reads as blank

uint16_t eeSeq = 0;
bool posValid = false;  // E= in the status line

uint8_t crc8(const uint8_t *p, uint8_t n) {
  uint8_t c = 0;
  while (n--) {
    c ^= *p++;
    for (uint8_t i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

bool readSlot(uint8_t i, Saved &s) {
  EEPROM.get(EE_BASE + i * EE_SLOT, s);
  return s.magic == EE_MAGIC && s.crc == crc8((const uint8_t *)&s, sizeof(Saved) - 1);
}

/* Called only with every motor at rest, so the counters are exact. Alternates
   slots, so the slot being overwritten is never the only good copy. */
void saveState() {
  Saved s;
  s.magic = EE_MAGIC;
  s.seq = ++eeSeq;
  for (uint8_t m = 0; m < N_MOTORS; m++) {
    s.pos[m] = stp[m].currentPosition();
    s.phase[m] = phase[m];
    s.endMask[m] = endMask[m];
  }
  s.crc = crc8((const uint8_t *)&s, sizeof(Saved) - 1);
  EEPROM.put(EE_BASE + (s.seq & 1) * EE_SLOT, s);
}

bool loadState() {
  Saved a, b;
  bool va = readSlot(0, a), vb = readSlot(1, b);
  if (!va && !vb) return false;
  Saved &s = (va && vb) ? (((int16_t)(a.seq - b.seq) > 0) ? a : b) : (va ? a : b);
  eeSeq = s.seq;
  for (uint8_t m = 0; m < N_MOTORS; m++) {
    stp[m].setCurrentPosition(s.pos[m]);
    phase[m] = s.phase[m] & 0x07;
    endMask[m] = s.endMask[m] & 0x0F;
  }
  return true;
}

/* ---------------- 12 V interlock ----------------
   A0 reads the 12 V rail through a divider, ~0.01238 V per count. Below this
   the DRV8871s are in undervoltage lockout (or close to it) and the rotor is
   not following the step train. 650 counts ~= 8.0 V. */
#define VM_MIN_COUNTS 650
bool vmFault = false;
unsigned long lastVm = 0;

bool vmOk() {
  adc0 = analogRead(A0);
  return adc0 >= VM_MIN_COUNTS;
}

/* Freeze every motor where it is. setCurrentPosition() also sets the target to
   the current count, so the run loop sees each move as finished. */
void cancelAll() {
  for (uint8_t m = 0; m < N_MOTORS; m++)
    if (stp[m].distanceToGo() != 0) stp[m].setCurrentPosition(stp[m].currentPosition());
  vmFault = true;
}

uint8_t movingMask() {
  uint8_t mask = 0;
  for (uint8_t m = 0; m < N_MOTORS; m++)
    if (stp[m].distanceToGo() != 0) mask |= (1 << m);
  return mask;
}

/* ---------------- serial ---------------- */

void sendVersion() {
  out.print(F("V " FW_NAME " " FW_VER " N="));
  out.println(N_MOTORS);
}

void sendStatus() {
  out.print(F("Q P="));
  for (uint8_t m = 0; m < N_MOTORS; m++) {
    out.print(stp[m].currentPosition() / HALF);
    if (m < N_MOTORS - 1) out.print(',');
  }
  out.print(F(" M="));
  out.print(movingMask());
  out.print(F(" K="));
  out.print(btnSticky);
  out.print(F(" U="));
  out.print(uptime);
  out.print(F(" A="));
  out.print(adc0);
  out.print(F(" B="));
  out.print(adc1);
  out.print(F(" E="));
  out.print(posValid ? 1 : 0);
  out.print(F(" F="));
  out.println(vmFault ? 1 : 0);
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
    if (!*rest) { out.println(F("ERR args")); return; }
    uint8_t row = (*rest == '1') ? 1 : 0;
    rest++;
    if (*rest == ' ') rest++;
    lcd.setCursor(0, row);
    uint8_t i = 0;
    for (; rest[i] && i < 16; i++) lcd.write(rest[i]);
    for (; i < 16; i++) lcd.write(' ');
    out.println(F("OK"));
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
      if (n < 5) { out.println(F("ERR args")); return; }
      int8_t m = parseMotor(a[0]);
      if (m < 0) { out.println(F("ERR motor")); return; }
      long sps = atol(a[2]);
      long acc = atol(a[3]);
      if (sps < 1) sps = 1;
      if (acc < 1) acc = 1;
      stp[m].setMaxSpeed((float)sps * HALF);
      stp[m].setAcceleration((float)acc * HALF);
      endMask[m] = (uint8_t)(atol(a[4]) & 0x0F);
      vmFault = false;
      stp[m].moveTo(atol(a[1]) * HALF);
      out.println(F("OK"));
      return;
    }

    case 'S': {
      if (n < 1) { out.println(F("ERR args")); return; }
      int8_t m = parseMotor(a[0]);
      if (m == -2) { out.println(F("ERR motor")); return; }
      if (m == -1) {
        for (uint8_t i = 0; i < N_MOTORS; i++) stp[i].stop();
      } else {
        stp[m].stop();  // decelerates; the run loop finishes and applies endMask
      }
      out.println(F("OK"));
      return;
    }

    case 'X': {
      if (n < 2) { out.println(F("ERR args")); return; }
      int8_t m = parseMotor(a[0]);
      if (m < 0) { out.println(F("ERR motor")); return; }
      long p = atol(a[1]);
      stp[m].setCurrentPosition(p * HALF);  // also zeroes speed
      posValid = true;
      if (active < 0) saveState();  // mid-move, the save at rest covers it
      out.println(F("OK"));
      return;
    }

    case 'C': {
      if (n < 2) { out.println(F("ERR args")); return; }
      int8_t m = parseMotor(a[0]);
      if (m < 0) { out.println(F("ERR motor")); return; }
      writeCoils(m, (uint8_t)(atol(a[1]) & 0x0F));
      out.println(F("OK"));
      return;
    }

    case 'G': {
      if (n < 1) { out.println(F("ERR args")); return; }
      lcd.setBacklight((uint8_t)(atol(a[0]) & 0x07));
      out.println(F("OK"));
      return;
    }

    case 'R': {  // R addr len  ->  R addr hh hh ...
      if (n < 2) { out.println(F("ERR args")); return; }
      long addr = atol(a[0]);
      long len = atol(a[1]);
      if (addr < 0 || len < 1 || len > 16 || addr + len > (long)EEPROM.length()) {
        out.println(F("ERR range"));
        return;
      }
      out.print(F("R "));
      out.print(addr);
      for (long i = 0; i < len; i++) {
        uint8_t b = EEPROM.read((int)(addr + i));
        out.print(' ');
        if (b < 0x10) out.print('0');
        out.print(b, HEX);
      }
      out.println();
      return;
    }

    default:
      out.println(F("ERR cmd"));
      return;
  }
}

void feedLine(char c, char *buf, uint8_t &len, uint8_t cap) {
  if (c == '\r') return;
  if (c == '\n') {
    buf[len] = 0;
    if (len) handleLine(buf);
    len = 0;
  } else if (len < cap - 1) {
    buf[len++] = c;
  } else {
    len = 0;  // overlong: drop rather than wrap
  }
}

#if LINK_J3
char linkLine[56];
uint8_t linkLen = 0;
#endif

void pollSerial() {
  while (Serial.available()) feedLine(Serial.read(), line, lineLen, sizeof(line));
#if LINK_J3
  while (linkRx.available()) feedLine(linkRx.read(), linkLine, linkLen, sizeof(linkLine));
#endif
}

/* ---------------- setup / loop ---------------- */

void setup() {
  Serial.begin(115200);

#if LINK_J3
  PORTB &= ~_BV(OD_PIN_BIT);  // output latch low, so "output" means "pull low"
  odRelease();                // idle = released = high via the ESP pull-up
  linkRx.begin(LINK_BAUD);
#endif

  Wire.begin();
  Wire.setClock(400000L);  // 797 steps/s ceiling; 265 at the 100 kHz default

  for (uint8_t i = 0; i < 3; i++) pcaInit(PCA_ADDR[i]);

  for (uint8_t m = 0; m < N_MOTORS; m++) {
    phase[m] = 0;
    endMask[m] = 0x00;  // coast
    stp[m].setMaxSpeed(90 * HALF);
    stp[m].setAcceleration(100 * HALF);
  }
  posValid = loadState();  // positions, phase and hold mask from last time
  for (uint8_t m = 0; m < N_MOTORS; m++) writeCoils(m, endMask[m]);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setBacklight(0x7);
  lcd.print(F(FW_NAME));
  lcd.setCursor(0, 1);
  lcd.print(posValid ? F("v" FW_VER " pos restored") : F("v" FW_VER " no saved pos"));

  /* lcd.begin() -> Adafruit_MCP23017::begin() -> Wire.begin(), and on AVR
     Wire.begin() resets the TWI to 100 kHz. Without this the whole bus runs
     at a quarter speed and every step's coil write takes most of a step. */
  Wire.setClock(400000L);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  sendVersion();  // banner, in case the host is already listening
}

void loop() {
  pollSerial();

#if LINK_J3
  if (odTail != odHead) {
    odSendByte(odBuf[odTail]);
    odTail = (uint8_t)((odTail + 1) % sizeof(odBuf));
  }
#endif

  unsigned long now = millis();

  if (now - lastSec >= 1000) {
    lastSec = now;
    uptime++;
    adc0 = analogRead(A0);
    adc1 = analogRead(A1);
  }

  /* Buttons are sampled even while a motor runs. One read is a handful of
     bytes at 400 kHz (readButtonsFast); at 20 Hz that is well under 1% of the step budget, and
     a stop button that goes dead during a move is not acceptable. */
  if (now - lastBtn >= 50) {
    lastBtn = now;
    btnSticky |= readButtonsFast();
  }

  /* One motor at a time: the 400 kHz bus tops out near 797 steps/s total. */
  if (active < 0) {
    for (uint8_t m = 0; m < N_MOTORS; m++) {
      if (stp[m].distanceToGo() != 0) {
        if (!vmOk()) {  // never start a move with no motor supply
          cancelAll();
          break;
        }
        active = (int8_t)m;
        lastVm = now;
        break;
      }
    }
  }

  if (active >= 0) {
    uint8_t m = (uint8_t)active;
    /* 20 ms between checks: at 400 half steps/s at most 8 steps can be counted
       after VM collapses. One analogRead is ~110 us. */
    if (now - lastVm >= 20) {
      lastVm = now;
      if (!vmOk()) cancelAll();
    }
    if (stp[m].distanceToGo() != 0) {
      stp[m].run();
    } else {
      writeCoils(m, endMask[m]);
      active = -1;
      saveState();  // at rest: the counter is exact, so persist it now
    }
  }
}
