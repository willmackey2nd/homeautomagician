#include <SPI.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "wificonfig.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// Debug prints
#define DEBUG

#ifdef DEBUG
#define PRINT(x)  Serial.print (x)
#define PRINTL(x)  Serial.println (x)
#else
#define PRINT(x)
#define PRINTL(x)
#endif


// Pins
#define ONE_WIRE_BUS_PIN D2


#define INTSENSUPDINTERVAL 15000 // Internal sensor update interval (ms)

char* NodeName = "mlp";
const char* ssid = WIFI_SSID;
const char* password =  WIFI_PASS;
const char* mqttServer = MQTT_SERVER_IP;
const int mqttPort = MQTT_SERVER_PORT;

WiFiClient espClient;
PubSubClient client(espClient);

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS_PIN);

// Pass oneWire reference to DallasTemperature library
DallasTemperature tempSensors(&oneWire);

struct sensorpacket {
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
} ;

// tmp vars
char tmpstring[50];
char tmpstring2[50];
long prevUpdMillis = -INTSENSUPDINTERVAL;
uint8_t deviceCount;

void setup() {

#ifdef DEBUG
  Serial.begin(115200);    // Debugging only
#endif
  delay(500);

  // IO
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // OneWire
  tempSensors.begin();
  // locate devices on the bus
  PRINT("Locating devices...");
  PRINT("Found ");
  deviceCount = tempSensors.getDeviceCount();
  PRINT(deviceCount);
  PRINTL(" devices.");
  PRINTL("");

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

  PRINTL("init done");
  digitalWrite(LED_BUILTIN, LOW);

  digitalWrite(LED_BUILTIN, LOW);
  blinkLed(5, 250);
}



int iii;
void loop() {
  // WiFi connection
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Internal sensors
  if (abs((long)millis() - prevUpdMillis) > INTSENSUPDINTERVAL) {
    internalSensors();
    prevUpdMillis = millis();
  }
}


void internalSensors() {
  struct sensorpacket intsens;
  char* sensorID = "tx";

  

  // Send command to all the sensors for temperature conversion
  tempSensors.requestTemperatures();

  // Display temperature from each sensor
  float temp;
  for (uint8_t i = 0;  i < deviceCount;  i++)
  {
    sprintf(intsens.id, "t%d", i);
    //strcpy(intsens.id, sensorID);
    //String("self").toCharArray(intsens.id, 8);

    // Temperature
    temp = tempSensors.getTempCByIndex(i);
    if (isnan(temp)) {
      PRINTL("temp error");
      intsens.payload = -2000;
    } else {
      intsens.payload = (int)(temp * 10.0);
    }
    Mqttpub(NodeName, intsens.id, 1, intsens.payload);

    PRINT("Sensor ");
    PRINT(i + 1);
    PRINT(" : ");
    PRINT(temp);
    PRINT((char)176);//shows degrees character
    PRINT("C  |  ");
    PRINT(DallasTemperature::toFahrenheit(temp));
    PRINT((char)176);//shows degrees character
    PRINTL("F");
  }
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

    case 6:
      strcpy(sensorTypeStr, "waterlevel/perc");
      break;

    default:
      strcpy(sensorTypeStr, "unknown");
  }
  sprintf(topicbuf, "devices/%s/sensors/%s/%s", nodeName, sensorID, sensorTypeStr);

}


void FormatPayload(char* payloadbuf, int8_t sensorType, int16_t rawPayload) {
  payloadbuf[0] = '\0';

  if (rawPayload == -1000) {
    strcpy(payloadbuf, "davesnothere");
    return;
  }
  if (rawPayload == -2000) {
    strcpy(payloadbuf, "error");
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

    case 6: // Water level percentage
      sprintf(payloadbuf, "%d", rawPayload);
      break;

    default:
      sprintf(payloadbuf, "%d", rawPayload);
  }
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    PRINT("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(NodeName, MQTT_USR, MQTT_PW)) {
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
