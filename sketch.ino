#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <Servo.h> // Biblioteca para controle do servo motor

// Define pins for ultrasonic sensor
#define TRIG_PIN 9
#define ECHO_PIN 8

// Define pins for LEDs
#define LED_GREEN 4
#define LED_RED 5
#define LED_YELLOW 6

// Define pin for servo motor
#define SERVO_PIN 3

// Data wire is connected to pin 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

// Create an RTC object
RTC_DS1307 rtc;

// Create a Servo object
Servo servo;

// EEPROM configuration
const int maxRecords = 100;
const int recordSize = 8; // Tamanho de cada registro em bytes
int startAddress = 0;
int endAddress = maxRecords * recordSize;
int currentAddress = 0;

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);

  sensors.begin();
  servo.attach(SERVO_PIN);
  servo.write(0); // Inicializa o servo na posição 0 graus

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, LOW);

  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  EEPROM.begin();
}

void loop() {
  DateTime now = rtc.now();

  // Measure distance with ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;

  // Request temperature measurements from DS18B20 sensor
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);

  // Check if reading is valid
  if (temperatureC == DEVICE_DISCONNECTED_C) {
    Serial.println("Failed to read from DS18B20 sensor!");
    return;
  }

  // Print temperature to Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" *C");

  // Turn on green LED if temperature is within a normal range
  if (temperatureC < 35) {
    digitalWrite(LED_GREEN, HIGH);
  } else {
    digitalWrite(LED_GREEN, LOW);
  }

  bool lixoDetectado = false;

  // If distance is less than 10 cm, assume waste is detected
  if (distance < 10) {
    digitalWrite(LED_RED, HIGH); // Turn on red LED to indicate detection
    delay(1000); // Wait for 1 second

    digitalWrite(LED_YELLOW, HIGH); // Simulate collection with yellow LED
    servo.write(90); // Move servo to 90 degrees to simulate collection
    delay(1000); // Wait for 1 second to simulate collection time
    servo.write(0); // Move servo back to 0 degrees
    digitalWrite(LED_YELLOW, LOW); // Turn off yellow LED
    digitalWrite(LED_RED, LOW); // Turn off red LED

    lixoDetectado = true;
  }

  // Check for irregularities
  if (lixoDetectado || temperatureC >= 35) {
    // Save irregularity to EEPROM
    saveIrregularity(now, temperatureC, lixoDetectado);
  }

  printStatus(now, temperatureC, lixoDetectado);

  delay(5000); // Delay between readings
}

void saveIrregularity(DateTime timestamp, float temperatureC, bool lixoDetectado) {
  Serial.println("Saving irregularity to EEPROM");

  // Converter valores para int para armazenamento
  int tempInt = (int)(temperatureC * 100);
  byte lixoStatus = lixoDetectado ? 1 : 0;

  // Escrever dados na EEPROM
  EEPROM.put(currentAddress, timestamp.unixtime());
  EEPROM.put(currentAddress + 4, tempInt);
  EEPROM.put(currentAddress + 6, lixoStatus);

  // Atualiza o endereço para o próximo registro
  getNextAddress();
}

void getNextAddress() {
  currentAddress += recordSize;
  if (currentAddress >= endAddress) {
    currentAddress = 0; // Volta para o começo se atingir o limite
  }
}

void printStatus(DateTime timestamp, float temperatureC, bool lixoDetectado) {
  Serial.print(timestamp.year());
  Serial.print("-");
  Serial.print(timestamp.month() < 10 ? "0" : "");
  Serial.print(timestamp.month());
  Serial.print("-");
  Serial.print(timestamp.day() < 10 ? "0" : "");
  Serial.print(timestamp.day());
  Serial.print(" ");
  Serial.print(timestamp.hour() < 10 ? "0" : "");
  Serial.print(timestamp.hour());
  Serial.print(":");
  Serial.print(timestamp.minute() < 10 ? "0" : "");
  Serial.print(timestamp.minute());
  Serial.print(":");
  Serial.print(timestamp.second() < 10 ? "0" : "");
  Serial.print(timestamp.second());
  Serial.print("       ");
  Serial.print(temperatureC);
  Serial.print(" graus    Lixo: ");
  Serial.println(lixoDetectado ? "SIM" : "NAO");
}
