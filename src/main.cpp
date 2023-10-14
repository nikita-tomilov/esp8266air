#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_CCS811.h>
#include "DHTesp.h"
#include <U8g2lib.h>

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP280 bmp;
unsigned long delayTime;

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
Adafruit_CCS811 ccs;
DHTesp dht;

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "wificreds.h"

#define temperature_topic "esp8266air/temperature"
#define pressure_topic "esp8266air/pressure"
#define humidity_topic "esp8266air/humidity"
#define eco2_topic "esp8266air/eco2"
#define tvoc_topic "esp8266air/tvoc"

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
  Serial.println("OLED done");

  // BMP280 setup
  unsigned status = bmp.begin(0x76);
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1)
      delay(10);
  }
  Serial.println("BMP280 done");

  dht.setup(13, DHTesp::DHT11);
  Serial.println("DHT done");

  // AIR TVOC setup
  if (!ccs.begin(0x5A))
  {
    while (1)
    {
      Serial.println("Failed to start sensor! Please check your wiring.");
      delay(500);
    }
  }
  while (!ccs.available())
  {
    printLog("Waiting for CCS811");
    delay(500);
  };
}

void setup()
{

  Wire.begin();
  printLog("BOOT");

  delay(2000);

  Serial.begin(115200);
  while (!Serial);

  setupPeripherals();
  setupWifi();

  pinMode(0, INPUT);
}

int x = 6;

uint16_t eco2ppm = 0;
uint16_t tvoc = 0;

float temp = 0.0f;
float pressHPa = 0.0f;
float humidity = 0.0f;

long lastValuesUpdate = 0;
long lastMqttUpdate = 0;
long lastButtonPress = 0;

void updateVals()
{
  if (ccs.available())
  {
    if (!ccs.readData())
    {
      eco2ppm = ccs.geteCO2();
      tvoc = ccs.getTVOC();
    }
  }

  temp = bmp.readTemperature();
  pressHPa = bmp.readPressure() / 100.0F;
  humidity = dht.getHumidity();
}

void drawVals(void)
{
  float temp = bmp.readTemperature();
  float pressHPa = bmp.readPressure() / 100.0F;
  float humidity = dht.getHumidity();

  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.setCursor(x, 12);
  u8g2.printf("T %.2f C\n", temp);
  u8g2.setCursor(x, 24);
  u8g2.printf("P %.2f hPa\n", pressHPa);
  u8g2.setCursor(x, 36);
  u8g2.printf("H %.0f %% \n", humidity);

  u8g2.setCursor(x, 48);
  u8g2.printf("CO2 %d ppm\n", eco2ppm);
  u8g2.setCursor(x, 60);
  u8g2.printf("TVOC %d ppb\n", tvoc);

  x += 1;
  if (x > 26)
  {
    x = 6;
  }
}

void sendVals()
{
  client.publish(temperature_topic, String(temp).c_str(), true);
  client.publish(pressure_topic, String(pressHPa).c_str(), true);
  client.publish(humidity_topic, String(humidity).c_str(), true);
  client.publish(eco2_topic, String(eco2ppm).c_str(), true);
  client.publish(tvoc_topic, String(tvoc).c_str(), true);

  ccs.setEnvironmentalData(humidity, temp);
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