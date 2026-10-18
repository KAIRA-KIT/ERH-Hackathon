// 2) Keyboard control over the USB cable (Serial).
// Board: Seeed XIAO ESP32-C3 + MX1508. Same wiring as drive_test.ino.
// Tools > USB CDC On Boot > Enabled
//
// Commands (single characters, 115200 baud):
//   w = forward    s = backward    x or space = stop
//   + = faster     - = slower
// Either type them in the Serial Monitor (letter + Enter),
// or run drive_serial.py on the laptop to drive with key presses (no Enter needed).

const int L_IN1 = 5;   // D3
const int L_IN2 = 6;   // D4
const int R_IN1 = 7;   // D5
const int R_IN2 = 10;  // D10

int speed = 180;       // 0-255
int direction = 0;     // 1 forward, -1 backward, 0 stop

void setMotor(int pinA, int pinB, int s) {
  if (s > 0)      { analogWrite(pinA, s);  analogWrite(pinB, 0); }
  else if (s < 0) { analogWrite(pinA, 0);  analogWrite(pinB, -s); }
  else            { analogWrite(pinA, 0);  analogWrite(pinB, 0); }
}

void drive(int s) {
  setMotor(L_IN1, L_IN2, s);
  setMotor(R_IN1, R_IN2, s);
}

void apply() {
  drive(direction * speed);
  Serial.printf("%s, speed %d\n",
                direction > 0 ? "FORWARD" : direction < 0 ? "BACKWARD" : "STOP", speed);
}

void handleCommand(char c) {
  switch (c) {
    case 'w': case 'W': direction = 1;  break;
    case 's': case 'S': direction = -1; break;
    case 'x': case 'X': case ' ': direction = 0; break;
    case '+': speed = min(speed + 25, 255); break;
    case '-': speed = max(speed - 25, 60);  break;
    default: return;   // ignore Enter and other characters
  }
  apply();
}

void setup() {
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  drive(0);
  Serial.begin(115200);
  delay(1000);
  Serial.println("Ready: w = forward, s = backward, x/space = stop, +/- = speed");
}

void loop() {
  while (Serial.available()) handleCommand(Serial.read());
}
