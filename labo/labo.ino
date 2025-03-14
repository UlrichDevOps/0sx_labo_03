#include <LCD_I2C.h>
#include "Adafruit_LiquidCrystal.h"
#include <Wire.h>

#define BTN_PIN 2

bool displayPage = true;  // true -> temp * false -> mvt
unsigned long currentTime = 0, previousMillis = 0;
int mappedValueX = 0, mappedValueY = 0;
int thermistorPin = A0, Vo;
float R1 = 10000, logR2, R2, T, Tc;
const float c1 = 1.129148e-03, c2 = 2.34125e-04, c3 = 8.76741e-08;
const int ledPin = 8;
int ledState = LOW;
const int maxTemperature = 25, minTemperature = 24;
const int minPWM = 0, maxPWM = 1023, middlePWM = 512;
const int minSpeed = -25, maxSpeed = 120;
const int minAngle = -90, maxAngle = 90;
int speed, angle, valueX, valueY;
String mouvement, direction;
String DA = "2372368";
LCD_I2C lcd(0x27, 16, 2);

// Caractères personnalisés
byte rightArrow[8] = {
  0b00000, 0b00100, 0b11110, 0b11111,
  0b11110, 0b00100, 0b00000, 0b00000
};

byte sixtyHeight[8] = {
  0b11100, 0b10000, 0b11100, 0b10111,
  0b11101, 0b00111, 0b00101, 0b00111
};

byte degree[8] = {
  0b00111, 0b00101, 0b00111, 0b00000,
  0b00000, 0b00000, 0b00000, 0b00000
};

void setup() {
  Serial.begin(115200);
  lcd.backlight();
  lcd.begin(16, 2);
  lcd.createChar(0, rightArrow);
  lcd.createChar(1, sixtyHeight);
  lcd.createChar(2, degree);
  pinMode(ledPin, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  displayStartupMessage();
}

void loop() {
  currentTime = millis();

  if (clickButton(currentTime)) {
    displayPage = !displayPage;
    lcd.clear();
  }

  readTemperature();
  readJoystick();

  if (displayPage) {
    controlAirConditioning();
  } else {
    displayMovement();
  }

  serialData(currentTime);
}

void displayStartupMessage() {
  unsigned long startMillis = millis();
  while (millis() - startMillis < 3000) {
    lcd.setCursor(0, 0);
    lcd.print("OMAM");
    lcd.setCursor(0, 1);
    lcd.write(byte(0));
    lcd.setCursor(15, 1);
    lcd.write(byte(1));
  }
  lcd.clear();
}

void readTemperature() {
  Vo = analogRead(thermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  Tc = T - 273.15;
}

void controlAirConditioning() {
  String AirConditioning = "OFF";
  if (Tc > maxTemperature) {
    ledState = HIGH;
    AirConditioning = "ON";
  } else if (Tc < minTemperature) {
    ledState = LOW;
  }
  digitalWrite(ledPin, ledState);
  lcd.setCursor(0, 0);
  lcd.print("Temp: " + String(Tc));
  lcd.write(byte(2));
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("AC: " + AirConditioning);
}

void readJoystick() {
  valueX = analogRead(A6);
  valueY = analogRead(A7);

  if (valueY < middlePWM) {
    mouvement = "Recule";
    speed = map(valueY, middlePWM, minPWM, 0, minSpeed);  // 511, 0, 0, -25
  } else {
    mouvement = "Avance";
    speed = map(valueY, middlePWM, maxPWM, 0, maxSpeed);  // 511, 1023, 0, 120
  }

  if (valueX < middlePWM) {
    direction = "G  ";
    angle = map(valueX, middlePWM, minPWM, 0, minAngle);  // 511, 0, 0, -90
  } else {
    direction = "D  ";
    angle = map(valueX, middlePWM, maxPWM, 0, maxAngle);  // 511, 1023, 0, 90
  }
}

void displayMovement() {
  lcd.setCursor(0, 0);
  lcd.print(mouvement + " " + String(speed) + " km/h  ");
  lcd.setCursor(0, 1);
  lcd.print(angle);
  lcd.write(byte(2));
  lcd.print(" ");
  lcd.print(direction);
}

bool clickButton(unsigned long ct) {
  static int lastState = HIGH;
  static int state = HIGH;
  const int rate = 50;
  static unsigned long previousTime = 0;

  int currentState = digitalRead(BTN_PIN);
  if (currentState != lastState) {
    previousTime = ct;
  }

  if ((ct - previousTime) > rate) {
    if (currentState != state) {
      state = currentState;
      if (state == LOW) {
        lastState = currentState;
        return true;
      }
    }
  }
  lastState = currentState;
  return false;
}

void serialData(unsigned long ct) {
  unsigned long dataMillis = 0;
  if (ct - dataMillis > 100) {
    Serial.print("etd:" + DA);
    Serial.print(",x:" + String(valueX));
    Serial.print(",y:" + String(valueY));
    Serial.println("sys:" + String(ledState) + ".");

    dataMillis = ct;
  }
}
