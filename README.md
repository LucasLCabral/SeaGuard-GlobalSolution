# SeaGuard

SeaGuard é um projeto de monitoramento e coleta de lixo no mar utilizando sensores e um servo motor para detecção e remoção de detritos. O sistema mede a temperatura da água e detecta a presença de lixo, registrando irregularidades em uma memória EEPROM.

<p align="center">
  <img src="https://github.com/LucasLCabral/SeaGuard-GlobalSolution/assets/162235385/83aa8d30-88fc-43db-8aa7-1efce6718d1e" alt="SeaGuard" width="400"/>
</p>

## Solução Proposta

O objetivo do projeto SeaGuard é monitorar e coletar lixo no mar, utilizando a plataforma Arduino em conjunto com diversos sensores e atuadores. A solução busca detectar a presença de detritos na água e registrar irregularidades, como temperaturas anormais, para melhorar a saúde do ecossistema marinho.
## Resultados Esperados e Impacto

Expectativas de Resultados
Monitoramento Contínuo: Espera-se que o sistema SeaGuard forneça monitoramento contínuo da temperatura da água e presença de detritos, permitindo a detecção precoce de problemas ambientais.
Registro de Dados: Com a capacidade de registrar dados de irregularidades, o sistema fornecerá informações valiosas para análise posterior e tomada de decisão.
Resposta Rápida: A detecção e coleta automáticas de lixo contribuirão para a limpeza do ambiente marinho, reduzindo o impacto negativo dos detritos no ecossistema.
Impacto Positivo
Melhoria da Qualidade da Água: Ao monitorar e coletar lixo, o projeto ajudará a manter a água mais limpa, beneficiando a vida marinha e a saúde do ecossistema.
Prevenção de Poluição: A detecção precoce de anomalias térmicas e a coleta de lixo reduzirão a poluição, prevenindo danos a longo prazo.
Educação e Conscientização: O projeto pode servir como uma ferramenta educativa para aumentar a conscientização sobre a importância da conservação marinha e o impacto do lixo nos oceanos.
Dados para Pesquisa: Os dados coletados podem ser utilizados por pesquisadores e organizações ambientais para estudar padrões de poluição e desenvolver estratégias de mitigação.

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

Contribuição
Faça um fork deste repositório.
Crie uma branch para sua feature (git checkout -b feature/AmazingFeature).
Commit suas mudanças (git commit -m 'Add some AmazingFeature').
Dê um push na sua branch (git push origin feature/AmazingFeature).
Abra um Pull Request.

Contato
Lucas Ludovico Cabral - lucasludovicocabraal@gmail.com

Projeto Link: https://github.com/LucasLCabral/SeaGuard
