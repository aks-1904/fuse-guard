#include <LiquidCrystal_I2C.h>

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pin Definitions
const int sensorPin = A0;
const int relayPin = 8;
const int greenLedPin = 9;
const int redLedPin = 10;
const int buzzerPin = 11;

// Overload threshold in Amps (adjust based on ACS712 type and need)
const float currentThreshold = 5.0;

void setup() {
  Serial.begin(9600);
  lcd.begin();
  lcd.backlight();

  pinMode(relayPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  digitalWrite(relayPin, HIGH); // Start with circuit connected
  lcd.print("FuseGuard Ready");
  delay(2000);
}

void loop() {
  int sensorValue = analogRead(sensorPin);
  float voltage = sensorValue * (5.0 / 1023.0);
  float current = (voltage - 2.5) / 0.185; // For ACS712-5A; adjust for 20A or 30A

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Current: ");
  lcd.print(current, 2);
  lcd.print(" A");

  if (abs(current) > currentThreshold) {
    // Overload detected
    digitalWrite(relayPin, LOW);      // Break circuit
    digitalWrite(redLedPin, HIGH);
    digitalWrite(greenLedPin, LOW);
    digitalWrite(buzzerPin, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Overload! CUT OFF");
  } else {
    digitalWrite(relayPin, HIGH);     // Keep circuit on
    digitalWrite(redLedPin, LOW);
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(buzzerPin, LOW);
    lcd.setCursor(0, 1);
    lcd.print("Status: Normal");
  }

  delay(1000);
}