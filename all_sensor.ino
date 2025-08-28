const int flameSensorPin = 4; // Signal pin of the flame sensor
const int mq2SensorPin = A0; // Analog pin for MQ2 gas sensor
const int tempSensorPin = A1; // Analog pin for Temperature sensor
const int dht11Pin = 2;      // Digital pin for DHT11 humidity sensor
const int ledPin = 13;       // Built-in LED on Arduino
const int relayPin = 8;      // Pin connected to the relay module
const int buzzerPin = 7;     // Pin connected to the piezo buzzer

// Thresholds
const int gasThreshold = 120;     // Threshold for MQ2 gas sensor
const int tempThreshold = 30;     // Threshold for Temperature in Celsius (adjust as needed)
const int humidityThreshold = 70; // Threshold for Humidity in percentage

#include "DHT.h"
#define DHTTYPE DHT11
DHT dht(dht11Pin, DHTTYPE);

void setup() {
  pinMode(flameSensorPin, INPUT); // Flame sensor as input
  pinMode(mq2SensorPin, INPUT);   // MQ2 gas sensor as input
  pinMode(tempSensorPin, INPUT);  // Temperature sensor as input
  pinMode(ledPin, OUTPUT);        // LED as output
  pinMode(relayPin, OUTPUT);      // Relay as output
  pinMode(buzzerPin, OUTPUT);     // Buzzer as output

  digitalWrite(relayPin, LOW);    // Ensure the relay is off initially
  digitalWrite(buzzerPin, LOW);   // Ensure the buzzer is off initially

  Serial.begin(9600);             // Initialize Serial Monitor
  dht.begin();                    // Initialize DHT sensor
}

void loop() {
  // Read the flame sensor
  int flameSensorValue = digitalRead(flameSensorPin);
  // Read the MQ2 gas sensor (analog value)
  int gasSensorValue = analogRead(mq2SensorPin);
  // Read the temperature sensor (analog value converted to Celsius)
  int tempSensorValue = analogRead(tempSensorPin);
  float voltage = (tempSensorValue / 1024.0) * 5.0; // Convert to voltage
  float temperature = (voltage - 0.5) * 100;        // Convert to Celsius
  // Read humidity from DHT11 sensor
  float humidity = dht.readHumidity();

  // Log sensor values to the Serial Monitor
  Serial.print("Flame Sensor: ");
  Serial.print(flameSensorValue);
  Serial.print(" | Gas Sensor: ");
  Serial.print(gasSensorValue);
  Serial.print(" | Temperature: ");
  Serial.print(temperature);
  Serial.print(" | Humidity: ");
  Serial.println(humidity);

  // Handle flame sensor input
  if (flameSensorValue == LOW) { // Flame detected
    Serial.println("Flame detected!");
    digitalWrite(ledPin, HIGH);   // Turn on LED
    digitalWrite(relayPin, LOW);  // Turn on pump
    digitalWrite(buzzerPin, HIGH); // Turn on buzzer
  } else {
    Serial.println("No flame detected.");
    digitalWrite(ledPin, LOW);    // Turn off LED
    digitalWrite(relayPin, HIGH); // Turn off pump
    digitalWrite(buzzerPin, LOW); // Turn on buzzer
  }

  // Handle MQ2 gas sensor input
  if (gasSensorValue > gasThreshold) { // Gas concentration exceeds threshold
    Serial.println("High gas concentration detected!");
    digitalWrite(buzzerPin, HIGH); // Turn on buzzer
  }

  // Handle temperature sensor input
  if (temperature > tempThreshold) { // Temperature exceeds threshold
    Serial.println("High temperature detected!");
    digitalWrite(buzzerPin, HIGH); // Turn on buzzer
  }

  // Handle humidity sensor input
  if (humidity > humidityThreshold) { // Humidity exceeds threshold
    Serial.println("High humidity detected!");
    digitalWrite(buzzerPin, HIGH); // Turn on buzzer
  }

  // Turn off buzzer if no alarms
  if (flameSensorValue != LOW && gasSensorValue <= gasThreshold && temperature <= tempThreshold && humidity <= humidityThreshold) {
    digitalWrite(buzzerPin, LOW);  // Turn off buzzer
  }

  delay(500); // Wait for half a second
}
