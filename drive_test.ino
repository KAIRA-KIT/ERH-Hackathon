// 1) Simple test: drive 3 s forward, then 3 s backward, then stop.
// Board: Seeed XIAO ESP32-C3 + MX1508 dual motor driver.
// Left motors (in parallel) on MX1508 channel A, right motors on channel B.
// Press the RESET button on the XIAO to run it again.
//
// Wiring:
//   XIAO D3 (GPIO5)  -> MX1508 IN1   (left)
//   XIAO D4 (GPIO6)  -> MX1508 IN2   (left)
//   XIAO D5 (GPIO7)  -> MX1508 IN3   (right)
//   XIAO D10 (GPIO10)-> MX1508 IN4   (right)
//   XIAO GND         -> MX1508 GND  (shared ground!)
//   Battery + / -    -> MX1508 + / -  (2-10 V)
// If one side turns the wrong way, swap that motor's two wires (or swap its two pins below).

const int L_IN1 = 5;   // D3
const int L_IN2 = 6;   // D4
const int R_IN1 = 7;   // D5
const int R_IN2 = 10;  // D10

const int SPEED = 180; // 0-255

// s: -255 (full backward) .. 0 (stop) .. 255 (full forward)
void setMotor(int pinA, int pinB, int s) {
  if (s > 0)      { analogWrite(pinA, s);  analogWrite(pinB, 0); }
  else if (s < 0) { analogWrite(pinA, 0);  analogWrite(pinB, -s); }
  else            { analogWrite(pinA, 0);  analogWrite(pinB, 0); }
}

void drive(int s) {
  setMotor(L_IN1, L_IN2, s);
  setMotor(R_IN1, R_IN2, s);
}

void setup() {
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  drive(0);
  delay(2000);          // time to put the car down

  drive(SPEED);         // forward
  delay(3000);
  drive(0);
  delay(500);           // short stop before reversing (protects motors/driver)
  drive(-SPEED);        // backward
  delay(3000);
  drive(0);             // stop
}

void loop() {
}
