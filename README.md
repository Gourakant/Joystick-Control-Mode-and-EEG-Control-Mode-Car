# Joystick-Control-Mode-and-EEG-Control-Mode-Car
# Code for slave
#include <SoftwareSerial.h>

// ====== Bluetooth (EEG Mode) ======
SoftwareSerial btSerial(10, 11); // RX, TX

// ====== Motor Driver ======
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9
#define ENA 5
#define ENB 3

// ====== Joystick ======
int joyXPin = A0;
int joyYPin = A1;
int buttonPin = 2;

// ====== Ultrasonic ======
int trigPin = 12;
int echoPin = 13;

// ====== Mode Switch ======
int modePin = 4;  // HIGH = EEG, LOW = Joystick

// ====== Variables ======
String incoming = "";
int attention = 0;
const int threshold = 30;

void setup() {
  Serial.begin(115200);
  btSerial.begin(38400); // HC-05 baud rate

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(modePin, INPUT_PULLUP);

  stopMotors();
  Serial.println("Slave Ready. Waiting for data...");
}

// =========================
// ===== MAIN LOOP =========
// =========================
void loop() {
  if (digitalRead(modePin) == HIGH) {
    runEEGControl();
  } else {
    runJoystickObstacleControl();
  }
}

// =========================
// ===== EEG MODE ==========
// =========================
void runEEGControl() {
  while (btSerial.available()) {
    char c = btSerial.read();
    if (c == '\n') {
      if (incoming.startsWith("ATT:")) {
        attention = incoming.substring(4).toInt();
        Serial.print("EEG Attention: ");
        Serial.println(attention);

        long distance = getDistance();
        Serial.print("Distance: ");
        Serial.println(distance);

        if (attention > threshold && distance > 15) {
          int speed = map(attention, threshold + 1, 100, 100, 255);
          moveForward(speed);
        } else {
          stopMotors();
        }
      }
      incoming = "";
    } else {
      incoming += c;
    }
  }
}


// =========================
// ===== JOYSTICK MODE =====
// =========================
void runJoystickObstacleControl() {
  int xVal = analogRead(joyXPin);
  int yVal = analogRead(joyYPin);
  int buttonState = digitalRead(buttonPin);

  long distance = getDistance();

  Serial.print("Joystick X: "); Serial.print(xVal);
  Serial.print(" | Y: "); Serial.print(yVal);
  Serial.print(" | Distance: "); Serial.print(distance); Serial.println(" cm");

  if (yVal < 400 && distance > 10) {
    moveForward(200);
  } else if (yVal > 600) {
    moveBackward(200);
  } else if (xVal > 600) {
    moveLeft(180);
  } else if (xVal < 400) {
    moveRight(180);
  } else {
    stopMotors();
  }
}

// =========================
// ===== Motor Control =====
// =========================
void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void moveBackward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void moveLeft(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void moveRight(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// =========================
// ===== Ultrasonic ========
// =========================
long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000); // 20ms timeout
  if (duration == 0) return 999; // no echo
  long distance = duration * 0.034 / 2;
  return distance;
}



# Code for Master
code for master: #include <Wire.h>
#include <LiquidCrystal_I2C.h>

// TGAM EEG on Serial1 (RX1=19, TX1=18)
// HC-05 Master on Serial2 (RX2=17, TX2=16)
LiquidCrystal_I2C lcd(0x27, 16, 2);
int attention = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);   // TGAM EEG
  Serial2.begin(38400);  // HC-05 Master

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("EEG Transmitter");
}

void loop() {
  static byte lastByte = 0;
  static byte payload[32];
  static int payloadIndex = 0;
  static int payloadLength = 0;
  static bool syncFound = false;

  while (Serial1.available()) {
    byte currentByte = Serial1.read();

    if (!syncFound) {
      if (lastByte == 0xAA && currentByte == 0xAA) {
        syncFound = true;
        payloadIndex = 0;
      }
      lastByte = currentByte;
      continue;
    }

    if (payloadIndex == 0) {
      payloadLength = currentByte;
      if (payloadLength > sizeof(payload)) {
        syncFound = false;
        lastByte = 0;
        continue;
      }
      payloadIndex++;
      continue;
    }

    if (payloadIndex <= payloadLength) {
      payload[payloadIndex - 1] = currentByte;
      payloadIndex++;
      continue;
    }

    byte checksum = currentByte;
    byte sum = 0;
    for (int i = 0; i < payloadLength; i++) sum += payload[i];
    sum = 255 - sum;

    if (sum == checksum) {
      for (int i = 0; i < payloadLength - 1; i++) {
        if (payload[i] == 0x04) {
          attention = payload[i + 1];
          Serial.print("Attention: ");
          Serial.println(attention);

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Attention:");
          lcd.setCursor(0, 1);
          lcd.print(attention);

          // Send via Bluetooth
          Serial2.print("ATT:");
          Serial2.println(attention);
          break;
        }
      }
    }

    syncFound = false;
    lastByte = 0;
  }
}
