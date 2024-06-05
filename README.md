# SeaGuard

SeaGuard é um projeto de monitoramento e coleta de lixo no mar utilizando sensores e um servo motor para detecção e remoção de detritos. O sistema mede a temperatura da água e detecta a presença de lixo, registrando irregularidades em uma memória EEPROM.

## Autores
  Lucas Ludovico Cabral - RM554589
  Weslley Oliveira Cardoso - RM557927

## Componentes

- Arduino
- Sensor de temperatura DS18B20
- Sensor ultrassônico HC-SR04
- LEDs (verde, vermelho e amarelo)
- Servo motor
- Módulo RTC DS1307
- Módulo EEPROM
- Cabos jumper e protoboard

## Instalação

1. Clone este repositório.
2. Abra o arquivo `SeaGuard.ino` no Arduino IDE.
3. Carregue o código no seu Arduino.
4. Conecte os componentes conforme o esquema abaixo.

## Esquema de Conexões

- DS18B20: Pino de dados no pino 2 do Arduino.
- HC-SR04:
  - Trig no pino 9 do Arduino.
  - Echo no pino 8 do Arduino.
- LEDs:
  - Verde no pino 4 do Arduino.
  - Vermelho no pino 5 do Arduino.
  - Amarelo no pino 6 do Arduino.
- Servo motor: Pino de controle no pino 3 do Arduino.
- RTC DS1307: Conectado aos pinos SDA e SCL do Arduino.
- EEPROM: Conectada aos pinos SDA e SCL do Arduino.

## Funcionamento

### Sensores

- **DS18B20**: Mede a temperatura da água.
- **HC-SR04**: Detecta a presença de lixo a uma distância inferior a 10 cm.

### LEDs

- **Verde**: Indica temperatura dentro da faixa normal (abaixo de 35°C).
- **Vermelho**: Acende quando lixo é detectado.
- **Amarelo**: Indica operação do servo motor para coleta do lixo.

### Servo Motor

- Movimenta-se para 90 graus para coletar o lixo e retorna a 0 graus.

### Memória EEPROM

Registra irregularidades (temperatura acima de 35°C ou detecção de lixo) com timestamp, temperatura e status de detecção de lixo.

### RTC DS1307

Mantém o registro de tempo real para timestamp das leituras e irregularidades.

## Código

```cpp
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <Servo.h>

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
const int recordSize = 8;
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
  servo.write(0);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, LOW);

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

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;

  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);

  if (temperatureC == DEVICE_DISCONNECTED_C) {
    Serial.println("Failed to read from DS18B20 sensor!");
    return;
  }

  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" *C");

  if (temperatureC < 35) {
    digitalWrite(LED_GREEN, HIGH);
  } else {
    digitalWrite(LED_GREEN, LOW);
  }

  bool lixoDetectado = false;

  if (distance < 10) {
    digitalWrite(LED_RED, HIGH);
    delay(1000);
    digitalWrite(LED_YELLOW, HIGH);
    servo.write(90);
    delay(1000);
    servo.write(0);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, LOW);

    lixoDetectado = true;
  }

  if (lixoDetectado || temperatureC >= 35) {
    saveIrregularity(now, temperatureC, lixoDetectado);
  }

  printStatus(now, temperatureC, lixoDetectado);

  delay(5000);
}

void saveIrregularity(DateTime timestamp, float temperatureC, bool lixoDetectado) {
  Serial.println("Saving irregularity to EEPROM");

  int tempInt = (int)(temperatureC * 100);
  byte lixoStatus = lixoDetectado ? 1 : 0;

  EEPROM.put(currentAddress, timestamp.unixtime());
  EEPROM.put(currentAddress + 4, tempInt);
  EEPROM.put(currentAddress + 6, lixoStatus);

  getNextAddress();
}

void getNextAddress() {
  currentAddress += recordSize;
  if (currentAddress >= endAddress) {
    currentAddress = 0;
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
```

Licença
Todos os direitos reservados. 1ESPI FIAP Global Solution 2024, first semester.

Contato
Lucas Ludovico Cabral - lucasludovicocabraal@gmail.com

Projeto Link: https://github.com/LucasLCabral/SeaGuard-GlobalSolution
