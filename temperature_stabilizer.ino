#define BLYNK_TEMPLATE_ID "TMPLYcy-HC5Y"
#define BLYNK_DEVICE_NAME "Temperature Stabilizer"
#define BLYNK_AUTH_TOKEN "V7MSkFU9IoT6oztNMxYdkQayQ2Le5KPO"

#define DHTPIN 14
#define DHTTYPE DHT11

#define BLYNK_GREEN "#23C48E"
#define BLYNK_RED   "#D3435C"

#define relay 12

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <WiFiManager.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;

WidgetLED led1(V1);
bool ledStatus = false;

double setTemperature = 50.0;
BLYNK_WRITE(V7) {
  setTemperature = param.asDouble();
}

void setup() {
  Serial.begin(115200);
  
  pinMode(relay, OUTPUT);
  
  digitalWrite(relay, HIGH);

  if (wifiManager()) {
    Serial.println("WiFi Connected");
  } else {
    Serial.println("Failed to connect.");
    ESP.restart();
  }
  
  blynkBegin();
  
  dht.begin();
  led1.on();
  timer.setInterval(1000L, sendSensor);
}

void loop() {
  Blynk.run();
  timer.run();
}

bool wifiManager() {
  WiFi.mode(WIFI_STA);

  WiFiManager manager;

  // enable for testing necessary
  manager.resetSettings();

  bool res;
  res = manager.autoConnect("Temperature Stabilizer", "temp12345");

  return res;
}

void sendSensor() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if(isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT!");
    return;
  }

  Blynk.virtualWrite(V5, t);
  Blynk.virtualWrite(V6, h);

  if (led1.getValue()) {
    led1.off();
  } else {
    led1.on();
  }
  
  if(t >= setTemperature) {
    led1.setColor(BLYNK_RED);
    digitalWrite(relay, LOW);
  } else {
    led1.setColor(BLYNK_GREEN);
    digitalWrite(relay, HIGH);
  }
}

void blynkBegin() {
  String __ssid = WiFi.SSID();
  String __pass = WiFi.psk();

  char ssid[__ssid.length() + 1];
  char pass[__pass.length() + 1];

  strcpy(ssid, __ssid.c_str());
  strcpy(pass, __pass.c_str());

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}
