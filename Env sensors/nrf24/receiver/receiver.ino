/////// NODE SPECIFIC!!!!! ///////
#define GATEWAYNR 1    // Nr of this gateway
#define TEMPSENTYPE 2  // 0 = NONE, 1 = DHT22, 2 = BME280, 3 = BMP280
#define OTHERSENTYPE 0 // 0 = NONE, 6 = CAPACITIVE WATER LEVEL

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#if TEMPSENTYPE == DHT22
#include "DHT.h" // Modified lines 155, 156 and 160 !!!
#endif

#include "wificonfig.h"
// #include <OneWire.h>
// #include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>

// Debug prints
#define DEBUG

#ifdef DEBUG
#define PRINT(x) Serial.print(x)
#define PRINTL(x) Serial.println(x)
#else
#define PRINT(x)
#define PRINTL(x)
#endif

// Pins
// DTH22 PCB layout still uses different NF24 CE PIN, therefore include it in conditional define
#if TEMPSENTYPE == 1
#define NF24_CE_PIN D2
#define NF24_CSN_PIN D8
#define DHTPIN D1 // what digital pin the DHT22 is conected to
#else
#define NF24_CE_PIN D0
#define NF24_CSN_PIN D8
#define DHTPIN D5 // what digital pin the DHT22 is conected to
#endif

#define DHTTYPE DHT22 // there are multiple kinds of DHT sensors

#define SENPWRPIN D4 // Sensor power supply pin

#define DHT22 1
#define BME280 2
#define BMP280 3

#if TEMPSENTYPE == BME280
Adafruit_BME280 bme; // I2C
#else
Adafruit_BMP280 bme; // I2C
#endif
#define SEALEVELPRESSURE_HPA (1013.25)

#if TEMPSENTYPE == DHT22
DHT dht(DHTPIN, DHTTYPE);
#endif

#define INTSENSUPDINTERVAL 600000      // Internal sensor update interval (ms)
RF24 radio(NF24_CE_PIN, NF24_CSN_PIN); // CE, CSN
uint8_t nrf24Addresses[][6] = {
    "1GW01",
    "2GW01",
    "3GW01",
    "4GW01",
    "5GW01",
    "6GW01",
}; // syntax xGWyy x = sensor number yy = gateway (nf24 only supports 6 clients)
char nrf24Address[6];

char *NodeName = "sensorgwyy";
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;
const char *mqttServer = MQTT_SERVER_IP;
const int mqttPort = MQTT_SERVER_PORT;

WiFiClient espClient;
PubSubClient client(espClient);

struct trh
{
  float rh = 0;
  float temp = 0;
  float pressure = 0;
} trh1;

struct sensorpacket
{
  char id[8] = "self"; //"SGW0001";
  /* Payload type
     0 = unknown/error
     1 = temperature (C * 10)
     2 = humidity (RH * 10)
     3 = pressure (hPa)
     4 = voltage (V)
     5 = luminance (lux)
     6 = waterlevel (%)
  */
  int8_t packetnr = 0; // running packet number
  int8_t type = 0;
  int16_t payload = 0;
};

// tmp vars
char tmpstring[50];
char tmpstring2[50];
long prevUpdMillis = -INTSENSUPDINTERVAL;

///////////////////////////////////////////////////
// Setup
///////////////////////////////////////////////////

void setup()
{

#ifdef DEBUG
  Serial.begin(115200); // Debugging only
#endif
  delay(500);

  // Node name
  sprintf(NodeName, "sensorgw%d", GATEWAYNR); // 'sensorgwYY
  
  // nRF24 addresses to listen
  PRINTL(sizeof(nrf24Addresses) / sizeof(nrf24Addresses[0]) - 1);
  for (int i = 0; i <= sizeof(nrf24Addresses) / sizeof(nrf24Addresses[0]) - 1; i++)
  {
    sprintf(nrf24Address, "%dGW0%d", i+1, GATEWAYNR); // syntax xGWyy x = sensor number yy = gateway
    memcpy(nrf24Addresses[i], nrf24Address, sizeof(nrf24Address));
    PRINT("nRF24 sensor address ");
    for (int j = 0; j < sizeof(nrf24Addresses[i]); j++)
    {
      PRINT((char)nrf24Addresses[i][j]);
    }
    PRINTL("");
  }

    // IO
    pinMode(LED_BUILTIN, OUTPUT);
    if (TEMPSENTYPE == DHT22) {
      PRINTL("Configuring DHT sensor pins");
      pinMode(DHTPIN, INPUT);
    }

    pinMode(SENPWRPIN, OUTPUT);

    digitalWrite(LED_BUILTIN, HIGH);

    delay(500);
    PRINT("configured 433 packet size: ");
    PRINTL(sizeof(sensorpacket));

    // WiFi

    // We start by connecting to a WiFi network
    PRINTL();
    PRINT(NodeName);
    PRINT(" connecting to ");
    PRINTL(ssid);

    WiFi.hostname(NodeName);
    WiFi.begin(ssid, password);

    int retrycnt = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      retrycnt += 1;

      if (retrycnt > 50)
      {
        PRINTL("WiFi connection timeout, restart ESP..");
        ESP.restart();
      }
      PRINT(".");
    }

    PRINTL("");
    PRINTL("WiFi connected");
    PRINTL("IP address: ");
    PRINTL(WiFi.localIP());

    client.setServer(mqttServer, mqttPort);
    // client.setCallback(callback);

    

    // NRF24
    radio.begin();
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setPayloadSize(12);
    // Open all 6 pipes for listening
    for (uint8_t i = 0; i <= 5; i++)
    {
      radio.openReadingPipe(i, nrf24Addresses[i]);
    }
    radio.startListening();

    PRINTL("init done");
    blinkLed(5, 250);
    
}

void loop()
{
  // WiFi connection
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  // Internal sensors
  if (abs((long)millis() - prevUpdMillis) > INTSENSUPDINTERVAL)
  {
    internalSensors();
    prevUpdMillis = millis();
  }

  uint8_t buf[sizeof(sensorpacket)];
  uint8_t buflen = sizeof(buf);
  byte pipeNum = 0; // variable to hold which reading pipe sent data
  if (radio.available(&pipeNum))
  {
    /* We can ignore the pipe of received packet as the packet
       contains sensor ID that's used to transfer data
       under correct MQTT topic
    */

    //   PRINTL("received shit");
    radio.read(&buf, sizeof(sensorpacket));

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // Message with a good checksum received, dump it.
    // driver.printBuffer("Got:", buf, buflen);

    struct sensorpacket sensorx;
    memcpy(&sensorx, buf, sizeof(sensorpacket));
    Mqttpub(NodeName, sensorx.id, sensorx.type, sensorx.payload);
  }
}

void internalSensors()
{
  updateValues();

  struct sensorpacket intsens;
  String("self").toCharArray(intsens.id, 8);

  // Temperature
  if (isnan(trh1.temp))
  {
    PRINTL("temp error");
    intsens.payload = -2000;
  }
  else
  {
    intsens.payload = (int)(trh1.temp * 10.0);
  }
  Mqttpub(NodeName, intsens.id, 1, intsens.payload);

  // Humidity
  if (isnan(trh1.rh))
  {
    PRINTL("rh error");
    intsens.payload = -2000;
  }
  else
  {
    intsens.payload = (int)(trh1.rh * 10.0);
  }
  Mqttpub(NodeName, intsens.id, 2, intsens.payload);

  // Pressure
  if (isnan(trh1.pressure))
  {
    PRINTL("pressure error");
    intsens.payload = -2000;
  }
  else
  {
    intsens.payload = (int)(trh1.pressure);
  }

  Mqttpub(NodeName, intsens.id, 3, intsens.payload);
}

void updateValues()
{
  PRINTL("Updating sensor values");
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  // Power up temp/humid/pressure sensor
  // voltage = readVcc();

  switch (TEMPSENTYPE)
  {
  case 0:
    break;
  case 1: // DHT
    readDHT(trh1);
    break;

  case 2: // BME280
    readBME280(trh1);
    break;
  case 3: // BMP280
    readBME280(trh1);
    break;

  default:
    break;
  }

  switch (OTHERSENTYPE)
  {
  case 1:
    break;
  case 6: // CAPACITIVE WATER LEVEL
    // ReadCapWH();
    break;

  default:;
    break;
  }
}

void readDHT(trh &_trh)
{
#if TEMPSENTYPE == DHT22
  digitalWrite(SENPWRPIN, HIGH);
  dht.begin();
  delay(2500); // Need to wait at least 2s for stable readings
  _trh.rh = dht.readHumidity();
  _trh.temp = dht.readTemperature();
  digitalWrite(SENPWRPIN, LOW);
  digitalWrite(DHTPIN, LOW); // disable the internal pullup resistor (enabled by dht.begin) to prevent current leak
  // pinmode(DHTPIN, INPUT); <-- this (input with pull-up resistor disabled) should consume equal amount of power as output set to low

  PRINT("temperature: ");
  PRINTL(_trh.temp);
  PRINT("Pressure: ");
  PRINTL(_trh.pressure);
  PRINT("humidity: ");
  PRINTL(_trh.rh);
#endif
}

bool bmeInitialized;
void readBME280(trh &_trh)
{
  digitalWrite(SENPWRPIN, HIGH);
  delay(200);
  if (!bmeInitialized) {
    if (!bme.begin(0x76))
    {
      PRINTL("Could not find a valid BME280 sensor, check wiring!");
      return;
    }
    PRINTL("Initializing BME280/BMP280");
    // Set forced mode to keep sensor cool and prevent self heating from distorting the temp reading.
    #if TEMPSENTYPE == BME280
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
    #else
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::FILTER_OFF);
    #endif
  }
  bme.takeForcedMeasurement();
  
#if TEMPSENTYPE == BME280
  _trh.rh = bme.readHumidity();
  PRINT("humidity: ");
  PRINTL(_trh.rh);
#endif

  _trh.temp = bme.readTemperature();
  PRINT("temperature: ");
  PRINTL(_trh.temp);

  _trh.pressure = bme.readPressure() / 100.0F;
  PRINT("Pressure: ");
  PRINTL(_trh.pressure);

  digitalWrite(SENPWRPIN, LOW);
}

void Mqttpub(const char *nodeName, char *sensorID, int8_t sensorType, int16_t rawPayload)
{
  char topicstring[50];
  char payload[30];
  GetMqttTopic(topicstring, NodeName, sensorType, sensorID);
  FormatPayload(payload, sensorType, rawPayload);
  PRINT(topicstring);
  PRINT(" ");
  PRINTL(payload);
  client.publish(topicstring, payload);
}

void GetMqttTopic(char *topicbuf, const char *nodeName, int8_t sensorType, char *sensorID)
{

  char sensorTypeStr[20];
  switch (sensorType)
  {
  case 0:
    strcpy(sensorTypeStr, "unknown");
    break;

  case 1:
    strcpy(sensorTypeStr, "temperature");
    break;

  case 2:
    strcpy(sensorTypeStr, "humidity");
    break;

  case 3:
    strcpy(sensorTypeStr, "pressure");
    break;

  case 4:
    strcpy(sensorTypeStr, "voltage");
    break;

  case 5:
    // sensorTypeStr = "luminance";
    strcpy(sensorTypeStr, "luminance");
    break;

  case 6:
    strcpy(sensorTypeStr, "waterlevel/perc");
    break;

  default:
    strcpy(sensorTypeStr, "unknown");
  }
  sprintf(topicbuf, "devices/%s/sensors/%s/%s", nodeName, sensorID, sensorTypeStr);
}

void FormatPayload(char *payloadbuf, int8_t sensorType, int16_t rawPayload)
{
  payloadbuf[0] = '\0';

  if (rawPayload == -1000)
  {
    strcpy(payloadbuf, "davesnothere");
    return;
  }
  if (rawPayload == -2000)
  {
    strcpy(payloadbuf, "error");
    return;
  }

  switch (sensorType)
  {
  case 0: // Unknown
    sprintf(payloadbuf, "%d", rawPayload);
    break;

  case 1: // Temperature
    sprintf(payloadbuf, "%.1f", (float)rawPayload / 10.0);
    break;

  case 2: // Humidity
    sprintf(payloadbuf, "%.1f", (float)rawPayload / 10.0);
    break;

  case 3: // Pressure
    sprintf(payloadbuf, "%d", rawPayload);
    break;

  case 4: // Voltage
    sprintf(payloadbuf, "%.2f", (float)rawPayload / 1000.0);
    break;

  case 5: // Luminance
    sprintf(payloadbuf, "%d", rawPayload);
    break;

  case 6: // Water level percentage
    sprintf(payloadbuf, "%d", rawPayload);
    break;

  default:
    sprintf(payloadbuf, "%d", rawPayload);
  }
}

// void ths(struct sensorpacket &sensor) {
//
//   /*
//    -1000 = sensor not available
//    -2000 = sensor error
//    */
//
//
//   //client.publish(tmpstring, String((float)sensor.temp/10.0).c_str());
//   char valarr[10] = "";
//
//   // Power supply
//   sprintf(tmpstring, "devices/%s/sensors/%s/supplyvoltage", NodeName, sensor.id);
//   if (sensor.pwr > -1000) {
//     sprintf(valarr, "%.2f", (float)sensor.pwr / 1000.0);
//     client.publish(tmpstring, valarr);
//   } else if (sensor.pwr == -2000) {
//     client.publish(tmpstring, "error");
//   }
//
//   // Temperature
//   sprintf(tmpstring, "devices/%s/sensors/%s/temperature", NodeName, sensor.id);
//   if (sensor.temp > -1000) {
//     sprintf(valarr, "%.1f", (float)sensor.temp / 10.0);
//     client.publish(tmpstring, valarr);
//   } else if (sensor.temp == -2000) {
//     client.publish(tmpstring, "error");
//   }
//
//   // Humidity
//   sprintf(tmpstring, "devices/%s/sensors/%s/humidity", NodeName, sensor.id);
//   if (sensor.rh > -1000) {
//     sprintf(valarr, "%.1f", (float)sensor.rh / 10.0);
//     client.publish(tmpstring, valarr);
//   } else if (sensor.rh == -2000) {
//     client.publish(tmpstring, "error");
//   }
//
//   // Pressure
//   sprintf(tmpstring, "devices/%s/sensors/%s/pressure", NodeName, sensor.id);
//   if (sensor.p > -1000) {
//     sprintf(valarr, "%d", sensor.p);
//     client.publish(tmpstring, valarr);
//   } else if (sensor.p == -2000) {
//     client.publish(tmpstring, "error");
//   }
//
//   // Luminosity
//   sprintf(tmpstring, "devices/%s/sensors/%s/luminosity", NodeName, sensor.id);
//   if (sensor.c > -1000) {
//     sprintf(valarr, "%d", sensor.c);
//     client.publish(tmpstring, valarr);
//   } else if (sensor.c == -2000) {
//     client.publish(tmpstring, "error");
//   }
// }

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    PRINT("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(NodeName, MQTT_USR, MQTT_PW))
    {
      PRINTL("connected");
      sprintf(tmpstring, "devices/%s/status", NodeName);

      client.publish(tmpstring, "connected");
    }
    else
    {
      PRINT("failed, rc=");
      PRINT(client.state());
      PRINTL(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void blinkLed(int times, int blinkdelay)
{
  for (int cnt = times; cnt > 0; cnt--)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(blinkdelay);
    digitalWrite(LED_BUILTIN, LOW);
    delay(blinkdelay);
  }
}
