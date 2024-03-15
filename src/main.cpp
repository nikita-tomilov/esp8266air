#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CScd4x.h>
#include <U8g2lib.h>

#define SEALEVELPRESSURE_HPA (1013.25)
unsigned long delayTime;

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "wificreds.h"

#define temperature_topic "esp8266air/temperature"
#define humidity_topic "esp8266air/humidity"
#define co2_topic "esp8266air/co2"
SensirionI2CScd4x scd4x;
int x = 6;

uint16_t co2ppm = 0;
float temp = 0.0f;
float humidity = 0.0f;
#define temp_offset -2.0f;
#define hum_offset 20.0f;
bool valAcquired = false;

long lastValuesUpdate = 0;
long lastMqttUpdate = 0;
long lastButtonPress = 0;

WiFiClient espClient;
PubSubClient client(espClient);

void printLog(String log)
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.setCursor(6, 12);
  u8g2.print(log);
  u8g2.sendBuffer();
}

void setupWifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  printLog("Waiting for WiFi");

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, mqtt_port);
}

void setupPeripherals()
{
  // OLED setup
  u8g2.begin();
  u8g2.setFlipMode(2);
  Serial.println("OLED done");
  printLog("Setup started");
}

void setupScd()
{

  uint16_t error;
  char errorMessage[256];

  scd4x.begin(Wire);

  // stop potentially previously started measurement
  error = scd4x.stopPeriodicMeasurement();
  if (error)
  {
    Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  uint16_t serial0;
  uint16_t serial1;
  uint16_t serial2;
  error = scd4x.getSerialNumber(serial0, serial1, serial2);
  if (error)
  {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else
  {
    Serial.print("SCD OK, serial: ");
    Serial.print(serial0);
    Serial.print(serial1);
    Serial.println(serial2);
  }

  error = scd4x.startPeriodicMeasurement();
  if (error)
  {
    Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  Serial.println("Waiting for first measurement... (5 sec)");
}

void setup()
{
  Wire.begin();
  printLog("BOOT");

  delay(2000);

  Serial.begin(115200);
  while (!Serial)
    ;

  setupPeripherals();
  setupWifi();

  pinMode(0, INPUT);
  lastButtonPress = millis();

  Wire.begin();
  setupScd();
}

void updateVals()
{
  char errorMessage[256];
  uint16_t lco2 = 0;
  float lt = 0.0f;
  float lh = 0.0f;
  bool isDataReady = false;
  int error = scd4x.getDataReadyFlag(isDataReady);
  if (error)
  {
    Serial.print("Error trying to execute getDataReadyFlag(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    return;
  }
  if (!isDataReady)
  {
    return;
  }
  Serial.println("Data ready");
  error = scd4x.readMeasurement(lco2, lt, lh);
  if (error)
  {
    Serial.print("Error trying to execute readMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else if (lco2 == 0)
  {
    Serial.println("Invalid sample detected, skipping.");
  }
  else
  {
    Serial.println("Valid sample detected");
    temp = lt + temp_offset;
    co2ppm = lco2;
    humidity = lh + hum_offset;
    valAcquired = true;
    Serial.print(co2ppm);
    Serial.print(" ppm; ");
    Serial.print(temp);
    Serial.print(" C; %");
    Serial.println(humidity);
  }
}

void drawVals(void)
{
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.setCursor(x, 12);
  u8g2.printf("T %.2f C\n", temp);
  u8g2.setCursor(x, 24);
  u8g2.printf("H %.0f %% \n", humidity);

  u8g2.setCursor(x, 36);
  u8g2.printf("CO2 %d ppm\n", co2ppm);

  x += 1;
  if (x > 26)
  {
    x = 6;
  }
}

void sendVals()
{
  if (valAcquired)
  {
    client.publish(temperature_topic, String(temp).c_str(), true);
    client.publish(humidity_topic, String(humidity).c_str(), true);
    client.publish(co2_topic, String(co2ppm).c_str(), true);
  }
  else
  {
    Serial.println("Val not acquired");
  }
}

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      printLog("No MQTT");
      delay(5000);
    }
  }
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastValuesUpdate > 1000)
  {
    lastValuesUpdate = now;
    updateVals();

    u8g2.clearBuffer();
    if (now - lastButtonPress < 5000)
    {
      drawVals();
    }
    u8g2.sendBuffer();
  }

  if (digitalRead(0) == 0)
  {
    lastButtonPress = millis();
  }

  if (now - lastMqttUpdate > 10000)
  {
    lastMqttUpdate = now;
    sendVals();
  }
}