/*
  Vehicle Blind Spot Detection System
  Board: Arduino Uno

  Two HC-SR04 ultrasonic sensors (left / right side of the car) are each
  mounted on a servo that sweeps them back and forth to cover a wide arc.
  Every sweep position stores a distance reading, and the nearest object
  on each side is shown on a 20x4 I2C LCD with its distance and direction.
  A potentiometer adjusts the sweep speed, and one LED per side lights up
  when something is detected (blinking when it is closer than CLOSE_CM).

  Wiring
    Left  sonar  TRIG -> D11  ECHO -> D10
    Right sonar  TRIG -> D4   ECHO -> D3
    Left  servo  PWM  -> D12
    Right servo  PWM  -> D2
    Left  LED    5V -> LED -> 220R -> D8   (active-low)
    Right LED    5V -> LED -> 220R -> D6   (active-low)
    Speed pot    SIG  -> A0
    LCD (I2C)    SDA  -> A4   SCL  -> A5   (address 0x27)
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// Pins
const uint8_t L_TRIG = 11, L_ECHO = 10;
const uint8_t R_TRIG = 4, R_ECHO = 3;
const uint8_t L_SERVO_PIN = 12, R_SERVO_PIN = 2;
const uint8_t SPEED_POT_PIN = A0;
// LEDs are wired active-low (anode to 5V, cathode through a resistor to the
// pin), so writing LOW turns them on.
const uint8_t L_LED_PIN = 8, R_LED_PIN = 6;

// Sweep
const int SWEEP_MIN = 30; // servo angle limits (degrees)
const int SWEEP_MAX = 150;
const int SWEEP_STEP = 5; // degrees moved per step
const int SLOTS = (SWEEP_MAX - SWEEP_MIN) / SWEEP_STEP + 1;

// Time per step, chosen by the potentiometer (5 deg per step).
// 60 ms (~1.4 s per pass) is the minimum: it leaves time for the servo to
// settle, for both sonars to time out (~25 ms each), and matches the
// HC-SR04 datasheet's minimum interval between pings.
const unsigned long STEP_MS_FAST = 60;
const unsigned long STEP_MS_SLOW = 300; // ~7.2 s per pass

// ---------------- Detection settings ----------------
const uint16_t DETECT_RANGE_CM = 300;          // ignore anything further away
const uint16_t MIN_RANGE_CM = 2;               // HC-SR04 minimum reliable range
const unsigned long ECHO_TIMEOUT_US = 25000UL; // ~4 m round trip
const uint16_t NO_OBJECT = 0;

// Readings within this many cm of the nearest count as equally near; the
// most recent of them is shown so the angle follows the sweep.
const uint16_t TIE_TOLERANCE_CM = 2;

// The side's LED blinks when the nearest object is closer than this
const uint16_t CLOSE_CM = 100;
const unsigned long BLINK_MS = 125;

// Relative angle (degrees from straight out sideways) used for labels.
// Negative = toward the rear of the car, positive = toward the front.
const int SIDE_ZONE_DEG = 15;

const unsigned long LCD_REFRESH_MS = 200;

// ---------------- LCD ----------------
const uint8_t LCD_COLS = 20;
const uint8_t LCD_ROWS = 4;

// ---------------- Types ----------------
struct Side
{
  char id; // 'L' or 'R' shown on the LCD
  uint8_t trigPin;
  uint8_t echoPin;
  uint8_t servoPin;
  uint8_t ledPin;
  bool mirrored; // right servo turns the opposite way to the left one
  Servo servo;
  uint16_t distMap[SLOTS]; // last distance seen at each sweep position
  uint16_t seenAt[SLOTS];  // step count when each position was measured
};

Side sides[2] = {
    {'L', L_TRIG, L_ECHO, L_SERVO_PIN, L_LED_PIN, false, Servo(), {}, {}},
    {'R', R_TRIG, R_ECHO, R_SERVO_PIN, R_LED_PIN, true, Servo(), {}, {}},
};

LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);

int slot = 0;           // current sweep position index
int sweepDir = 1;       // +1 or -1
uint16_t stepCount = 0; // total steps taken (wraps around safely)
unsigned long stepInterval = STEP_MS_SLOW;
unsigned long lastStepMs = 0;
unsigned long lastLcdMs = 0;

// ---------------- Helpers ----------------
int slotToServoAngle(int s)
{
  return SWEEP_MIN + s * SWEEP_STEP;
}

// The servos are mounted as mirror images: the left one faces left and the
// right one faces right (front of the car = top of the diagram). The right
// servo is driven to (180 - angle) so both sonars point the same way
// relative to the car (both toward the rear, or both toward the front) at
// the same time.
int servoAngleFor(const Side &side, int s)
{
  int angle = slotToServoAngle(s);
  return side.mirrored ? 180 - angle : angle;
}

// Angle from straight out sideways, the same for both sides.
// Negative = toward the rear of the car, positive = toward the front.
int relativeAngle(int s)
{
  return slotToServoAngle(s) - 90;
}

const char *directionLabel(int rel)
{
  if (rel < -SIDE_ZONE_DEG)
    return "REAR";
  if (rel > SIDE_ZONE_DEG)
    return "FRONT";
  return "SIDE";
}

uint16_t measureCm(uint8_t trigPin, uint8_t echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long us = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (us == 0)
    return NO_OBJECT; // no echo within timeout

  uint16_t cm = us / 58; // speed of sound: ~58 us per cm (round trip)
  if (cm < MIN_RANGE_CM || cm > DETECT_RANGE_CM)
    return NO_OBJECT;
  return cm;
}

// Finds the nearest object in the side's map. Returns its slot, or -1.
// Among readings within TIE_TOLERANCE_CM of the nearest, the most recently
// measured one wins, so a wide object (e.g. a car alongside) is tracked
// as the sweep moves across it instead of locking to one angle.
int nearestSlot(const Side &side)
{
  uint16_t minDist = NO_OBJECT;
  for (int s = 0; s < SLOTS; s++)
  {
    uint16_t d = side.distMap[s];
    if (d != NO_OBJECT && (minDist == NO_OBJECT || d < minDist))
      minDist = d;
  }
  if (minDist == NO_OBJECT)
    return -1;

  int best = -1;
  uint16_t bestAge = 0;
  for (int s = 0; s < SLOTS; s++)
  {
    uint16_t d = side.distMap[s];
    if (d == NO_OBJECT || d > minDist + TIE_TOLERANCE_CM)
      continue;
    uint16_t age = stepCount - side.seenAt[s];
    if (best < 0 || age < bestAge)
    {
      best = s;
      bestAge = age;
    }
  }
  return best;
}

unsigned long readStepInterval()
{
  int raw = analogRead(SPEED_POT_PIN);
  // Turning the pot clockwise makes the sweep faster
  return map(raw, 0, 1023, STEP_MS_SLOW, STEP_MS_FAST);
}

void printLine(uint8_t row, const char *text)
{
  char line[LCD_COLS + 1];
  snprintf(line, sizeof(line), "%-20s", text);
  lcd.setCursor(0, row);
  lcd.print(line);
}

// Fills one side's direction, distance and angle text (max 10 chars each)
void sideText(const Side &side, char dir[11], char dist[11], char angle[11])
{
  int s = nearestSlot(side);
  if (s < 0)
  {
    strcpy(dir, "CLEAR");
    strcpy(dist, "---");
    strcpy(angle, "---");
    return;
  }
  int rel = relativeAngle(s);
  strcpy(dir, directionLabel(rel));
  snprintf(dist, 11, "%u cm", side.distMap[s]);
  snprintf(angle, 11, "%+d\xDF", rel); // 0xDF = LCD degree symbol
}

// Screen split into two 10-char columns, left side on the left:
//   "LEFT           RIGHT"
//   "REAR           CLEAR"
//   "85 cm            ---"
//   "-45°             ---"
void showSides()
{
  char left[3][11], right[3][11];
  sideText(sides[0], left[0], left[1], left[2]);
  sideText(sides[1], right[0], right[1], right[2]);

  printLine(0, "LEFT           RIGHT");
  char buf[LCD_COLS + 1];
  for (uint8_t i = 0; i < 3; i++)
  {
    snprintf(buf, sizeof(buf), "%-10s%10s", left[i], right[i]);
    printLine(i + 1, buf);
  }
}

void moveServos()
{
  for (Side &side : sides)
    side.servo.write(servoAngleFor(side, slot));
}

// Off = clear, on = object detected, blinking = closer than CLOSE_CM
void updateLed(const Side &side, unsigned long now)
{
  int s = nearestSlot(side);
  bool on = false;
  if (s >= 0)
    on = side.distMap[s] >= CLOSE_CM || (now / BLINK_MS) % 2 == 0;
  digitalWrite(side.ledPin, on ? LOW : HIGH); // active-low
}

// ---------------- Main ----------------
void setup()
{
  Serial.begin(115200);

  for (Side &side : sides)
  {
    pinMode(side.trigPin, OUTPUT);
    pinMode(side.echoPin, INPUT);
    digitalWrite(side.trigPin, LOW);
    pinMode(side.ledPin, OUTPUT);
    digitalWrite(side.ledPin, HIGH); // off
    side.servo.attach(side.servoPin);
    for (int s = 0; s < SLOTS; s++)
      side.distMap[s] = NO_OBJECT;
  }

  lcd.init();
  lcd.backlight();
  printLine(0, "Blind Spot Det.");
  printLine(1, "Starting sweep..");

  moveServos();
  delay(1000); // let servos reach the start position
  lcd.clear();

  Serial.println(F("angle,left_cm,right_cm,step_ms"));
}

void loop()
{
  unsigned long now = millis();

  if (now - lastStepMs >= stepInterval)
  {
    lastStepMs = now;
    stepInterval = readStepInterval();

    // Servos were moved one step ago and have settled: measure here.
    // Sonars fire one after the other so their echoes can't interfere.
    for (Side &side : sides)
    {
      side.distMap[slot] = measureCm(side.trigPin, side.echoPin);
      side.seenAt[slot] = stepCount;
    }
    stepCount++;

    Serial.print(slotToServoAngle(slot));
    Serial.print(',');
    Serial.print(sides[0].distMap[slot]);
    Serial.print(',');
    Serial.print(sides[1].distMap[slot]);
    Serial.print(',');
    Serial.println(stepInterval);

    // Advance and bounce at the ends of the sweep
    slot += sweepDir;
    if (slot <= 0 || slot >= SLOTS - 1)
      sweepDir = -sweepDir;
    moveServos();
  }

  if (now - lastLcdMs >= LCD_REFRESH_MS)
  {
    lastLcdMs = now;
    showSides();
  }

  for (const Side &side : sides)
    updateLed(side, now);
}
