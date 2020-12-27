#define DEBUG

#ifdef DEBUG
#define PRINT(x)  Serial.print (x)
#define PRINTL(x)  Serial.println (x)
#else
#define PRINT(x)
#define PRINTL(x)
#endif

// ask_receiver.pde
// -*- mode: C++ -*-
// Simple example of how to use RadioHead to receive messages
// with a simple ASK transmitter in a very simple way.
// Implements a simplex (one-way) receiver with an Rx-B1 module
// Tested on Arduino Mega, Duemilanova, Uno, Due, Teensy, ESP-12

//#include "sensor.h"
#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif

#include "DHT.h" // Modified lines 155, 156 and 160 !!!
#define DHTPIN D6     // what digital pin the DHT22 is conected to
#define DHTPWRPIN D5     // what digital pin the DHT22 is conected to
#define DHTTYPE DHT22   // there are multiple kinds of DHT sensors
DHT dht(DHTPIN, DHTTYPE);
#define INTSENSUPDINTERVAL 600000 // Internal sensor update interval (ms)

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

struct sensorpacket {
  char id[8] = "SGW0001";
  /* Payload type
     0 = unknown/error
     1 = temperature (C * 10)
     2 = humidity (RH * 10)
     3 = pressure (hPa)
     4 = voltage (V)
     5 = luminance (lux)
  */
  int8_t packetnr = 0; // running packet number
  int8_t type = 0;
  int16_t payload = 0;
} ;


//RH_ASK driver;
RH_ASK driver(100, 4, 5, 0); // 500 bps, receive GPIO4, send, GPIO5
// RH_ASK driver(2000, 2, 4, 5); // ESP8266 or ESP32: do not use pin 11 or 2
// RH_ASK driver(2000, 3, 4, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85),
// RH_ASK driver(2000, PD14, PD13, 0); STM32F4 Discovery: see tx and rx on Orange and Red LEDS


WiFiClient espClient;
PubSubClient client(espClient);
const char* NodeName = "sensorgw1";
const char* ssid = "CNorrisNetworkB_2G";
const char* password =  "rekrop10";
const char* mqttServer = "192.168.1.174";
const int mqttPort = 1883;


// tmp vars
char tmpstring[50];
char tmpstring2[50];
long prevUpdMillis = -INTSENSUPDINTERVAL;

void setup() {

#ifdef DEBUG
  Serial.begin(115200);    // Debugging only
#endif




  // IO
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DHTPWRPIN, OUTPUT);
  pinMode(DHTPIN, INPUT );

  digitalWrite(LED_BUILTIN, HIGH);


  delay(500);
  PRINT("configured 433 packet size: "); PRINTL(sizeof(sensorpacket));

  // WiFi


  // We start by connecting to a WiFi network
  PRINTL();
  PRINT(NodeName);
  PRINT(" connecting to ");
  PRINTL(ssid);

  WiFi.hostname(NodeName);

  WiFi.begin(ssid, password);

  int retrycnt = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    retrycnt += 1;

    if (retrycnt > 50) {
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
  //client.setCallback(callback);

  // RF433
#ifdef RH_HAVE_SERIAL
  //  Serial.begin(9600);	  // Debugging only

  PRINTL("start");
  delay(2000);
#endif
  if (!driver.init())
#ifdef RH_HAVE_SERIAL
    PRINTL("init failed");
#else
    ;
#endif
  PRINTL("init done");
  digitalWrite(LED_BUILTIN, LOW);

  digitalWrite(LED_BUILTIN, LOW);
  blinkLed(5, 250);
}




void loop() {
  // WiFi connection
  if (!client.connected()) {
    reconnect();
  }
  client.loop();


/*
  // Internal sensors
  if (abs(millis() - prevUpdMillis) > 1000) { //INTSENSUPDINTERVAL) {
    internalSensors();
    prevUpdMillis = millis();
  }

*/

  //uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buf[sizeof(sensorpacket)];
  uint8_t buflen = sizeof(buf);

  if (driver.recv(buf, &buflen)) // Non-blocking
  {
    int i;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // Message with a good checksum received, dump it.
    //driver.printBuffer("Got:", buf, buflen);

    struct sensorpacket sensorx;
    memcpy(&sensorx, buf, sizeof(sensorpacket));
    Mqttpub(NodeName, sensorx.id, sensorx.type, sensorx.payload);

  }
}


void internalSensors() {
  struct sensorpacket intsens;
  digitalWrite(DHTPWRPIN, HIGH);
  dht.begin();
  delay(2000); // Need to wait at least 2s for stable readings
  String("self").toCharArray(intsens.id, 8);


  float rh = dht.readHumidity();
  float temp = dht.readTemperature();

  // Temperature
  if (isnan(rh)) {
    PRINTL("rh error");
    intsens.payload = -2000;
  } else {
    intsens.payload = (int)temp * 10;
  }
  Mqttpub(NodeName, intsens.id, 1, intsens.payload);


  // Humidity
  if (isnan(rh)) {
    PRINTL("temp error");
    intsens.payload = -2000;
  } else {
    intsens.payload = (int)rh * 10;
  }
  Mqttpub(NodeName, intsens.id, 2, intsens.payload);

  digitalWrite(DHTPWRPIN, LOW);
  digitalWrite(DHTPIN, LOW);

  // intsens.rh = (int)(dht.readHumidity() * 10);
  // intsens.temp = (int)(dht.readTemperature() * 10);

  //PRINT("Internal sensor "); PRINT((float)intsens.temp/10.0); PRINT("C "); PRINT((float)intsens.rh/10.0); PRINTL(" %rh");


  // ths(intsens);
}


void Mqttpub(const char* nodeName, char* sensorID, int8_t sensorType, int16_t rawPayload) {
  char topicstring[50];
  char payload[30];
  GetMqttTopic(topicstring, NodeName, sensorType, sensorID);
  FormatPayload(payload, sensorType, rawPayload);
  PRINT(topicstring); PRINT(" "); PRINTL(payload);
  client.publish(topicstring, payload);
}

void GetMqttTopic(char* topicbuf, const char* nodeName, int8_t sensorType, char* sensorID) {

  char sensorTypeStr[20];
  switch (sensorType) {
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

    default:
      strcpy(sensorTypeStr, "unknown");
  }
  sprintf(topicbuf, "devices/%s/sensors/%s/%s", nodeName, sensorID, sensorTypeStr);

}


void FormatPayload(char* payloadbuf, int8_t sensorType, int16_t rawPayload) {

  if (rawPayload == -1000) {
    payloadbuf = "davesnothere" ;
    return;
  }
  if (rawPayload == -2000) {
    payloadbuf = "error" ;
    return;
  }

  switch (sensorType) {
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

    default:
      sprintf(payloadbuf, "%d", rawPayload);
  }
}


//void ths(struct sensorpacket &sensor) {
//
//  /*
//   -1000 = sensor not available
//   -2000 = sensor error
//   */
//
//
//  //client.publish(tmpstring, String((float)sensor.temp/10.0).c_str());
//  char valarr[10] = "";
//
//  // Power supply
//  sprintf(tmpstring, "devices/%s/sensors/%s/supplyvoltage", NodeName, sensor.id);
//  if (sensor.pwr > -1000) {
//    sprintf(valarr, "%.2f", (float)sensor.pwr / 1000.0);
//    client.publish(tmpstring, valarr);
//  } else if (sensor.pwr == -2000) {
//    client.publish(tmpstring, "error");
//  }
//
//  // Temperature
//  sprintf(tmpstring, "devices/%s/sensors/%s/temperature", NodeName, sensor.id);
//  if (sensor.temp > -1000) {
//    sprintf(valarr, "%.1f", (float)sensor.temp / 10.0);
//    client.publish(tmpstring, valarr);
//  } else if (sensor.temp == -2000) {
//    client.publish(tmpstring, "error");
//  }
//
//  // Humidity
//  sprintf(tmpstring, "devices/%s/sensors/%s/humidity", NodeName, sensor.id);
//  if (sensor.rh > -1000) {
//    sprintf(valarr, "%.1f", (float)sensor.rh / 10.0);
//    client.publish(tmpstring, valarr);
//  } else if (sensor.rh == -2000) {
//    client.publish(tmpstring, "error");
//  }
//
//  // Pressure
//  sprintf(tmpstring, "devices/%s/sensors/%s/pressure", NodeName, sensor.id);
//  if (sensor.p > -1000) {
//    sprintf(valarr, "%d", sensor.p);
//    client.publish(tmpstring, valarr);
//  } else if (sensor.p == -2000) {
//    client.publish(tmpstring, "error");
//  }
//
//  // Luminosity
//  sprintf(tmpstring, "devices/%s/sensors/%s/luminosity", NodeName, sensor.id);
//  if (sensor.c > -1000) {
//    sprintf(valarr, "%d", sensor.c);
//    client.publish(tmpstring, valarr);
//  } else if (sensor.c == -2000) {
//    client.publish(tmpstring, "error");
//  }
//}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    PRINT("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(NodeName)) {
      PRINTL("connected");
      sprintf(tmpstring, "devices/%s/status", NodeName);

      client.publish(tmpstring, "connected");

    } else {
      PRINT("failed, rc=");
      PRINT(client.state());
      PRINTL(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void blinkLed(int times, int blinkdelay) {
  for (int cnt = times; cnt > 0; cnt--) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(blinkdelay);
    digitalWrite(LED_BUILTIN, LOW);
    delay(blinkdelay);
  }
}
