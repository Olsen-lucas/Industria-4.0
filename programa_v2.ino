#include <Wire.h>
#include <MPU6050_tockn.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h> //Nova biblioteca
#include <PubSubClient.h> //Nova biblioteca

#define TEMP_PIN 2

#define PWM D1
#define POT_PIN A0

const char* ssid = "OLSEN"; //Substitua com o nome da sua rede
const char* password = "13021964"; //Substitua com a senha da sua rede

const char* mqttServer = "broker.hivemq.com";
const int mqttPort = 1883; //Porta MQTT SSL

WiFiClient espClient;
PubSubClient client(espClient);

// ========================================================================================================
// --- Declaração de Objetos e Variáveis ---
MPU6050 mpu6050(Wire);
OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress DS18B20 = {0x28, 0xD8, 0xA3, 0x16, 0xA8, 0x01, 0x3C, 0x3D};
float tempC = 0;
float magnitude = 0;

// ========================================================================================================
// --- Protótipo das Funções ---
void readTemperature(DeviceAddress deviceAddress);
void readVibration();
void printData();
void controlMotor();
void setup_wifi();
void reconnect();

// ========================================================================================================
// --- Configurações Iniciais ---
void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  sensors.begin();
  sensors.setResolution(DS18B20, 10); //configura para resolução de 10 bits
  
  pinMode(PWM, OUTPUT);

  setup_wifi();
  client.setServer(mqttServer, mqttPort);
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  readTemperature(DS18B20);
  readVibration();
  controlMotor();

  String tempString = String(tempC);
  String magnitudeString = String(magnitude);
  
  client.publish("olsen_esp8266_temperature", tempString.c_str());
  client.publish("olsen_esp8266_vibration", magnitudeString.c_str());

  delay(1000);
}

// ========================================================================================================
// --- Desenvolvimento das Funções ---
void readTemperature(DeviceAddress deviceAddress)
{
  sensors.requestTemperatures();
  tempC = sensors.getTempC(deviceAddress);
  if (tempC == -127.00) 
  {
    Serial.print("Erro de leitura");
    delay(5000);
  }  
}

void readVibration() 
{
  mpu6050.update();

  float ax = mpu6050.getAccX();
  float ay = mpu6050.getAccY();
  float az = mpu6050.getAccZ();

  magnitude = sqrt(ax*ax + ay*ay + az*az);
}

void controlMotor()
{
  int potValue = analogRead(POT_PIN);
  int motorSpeed = map(potValue, 0, 1023, 0, 255);
  analogWrite(MOTOR_PIN, motorSpeed);
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP8266Client_OLSEN")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
