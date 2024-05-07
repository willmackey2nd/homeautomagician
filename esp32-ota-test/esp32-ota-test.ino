
/*
  -----------------------
  ElegantOTA - Demo Example
  -----------------------

  Skill Level: Beginner

  This example provides with a bare minimal app with ElegantOTA functionality.

  Github: https://github.com/ayushsharma82/ElegantOTA
  WiKi: https://docs.elegantota.pro

  Works with both ESP8266 & ESP32

  -------------------------------

  Upgrade to ElegantOTA Pro: https://elegantota.pro

*/
#include "wificonfig.h"
#define ESP32
#define TIME_TO_SLEEP 10000000 // sleep time in us

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <WiFiClient.h>
  #include <ESP8266WebServer.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <WebServer.h>
#endif

#include <ElegantOTA.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;
const char *mqttServer = MQTT_SERVER_IP;
const int mqttPort = MQTT_SERVER_PORT;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

char *NodeName = "espsensortest";
char tmpstring[50];

#if defined(ESP8266)
  ESP8266WebServer server(80);
#elif defined(ESP32)
  WebServer server(80);
#endif

// Define pins
#define sensorPin 2
#define sensorPwrPin 3

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(sensorPin);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature tempSensors(&oneWire);

//variabls for blinking an LED with Millis
const int led = 15; // ESP32 Pin to which onboard LED is connected
unsigned long previousMillis = 0;  // will store last time LED was updated
const long interval = 100;  // interval at which to blink (milliseconds)
int ledState = LOW;  // ledState used to set the LED

void setup(void) {
  pinMode(led, OUTPUT);


  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  mqttClient.setServer(mqttServer, mqttPort);
  //mqttClient.setCallback(callback);

  server.on("/", []() {
    server.send(200, "text/plain", "Hi! This is ElegantOTA Demo.");
  });

  ElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");

  // Pin init
  pinMode(sensorPin, INPUT);
  pinMode(sensorPwrPin, OUTPUT);
  digitalWrite(sensorPwrPin, HIGH);
  delay(200);

  // Sensor setup
  tempSensors.begin();

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //rtc_sleep_enable_ultra_low(true);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP);

}

void loop(void) {
  server.handleClient();
  ElegantOTA.loop();

  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  

  //loop to blink without delay
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
  // save the last time you blinked the LED
  previousMillis = currentMillis;
  // if the LED is off turn it on and vice-versa:
  ledState = not(ledState);
  // set the LED with the ledState of the variable:
  digitalWrite(led,  ledState);
  delay(200);
  ledState = not(ledState);
  // set the LED with the ledState of the variable:
  digitalWrite(led,  ledState);


  // Read temperature
  tempSensors.requestTemperatures();
  float temperature = tempSensors.getTempCByIndex(0);
  Serial.println("Temperature: " + String(temperature) + " C");
  char tempString[8];
  dtostrf(temperature, 1, 2, tempString);
  mqttClient.publish("/devices/esptest/temperature", tempString);
  }

  esp_deep_sleep_start();
}


void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
   // PRINT("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(NodeName, MQTT_USR, MQTT_PW)) {
      //PRINTL("connected");
      sprintf(tmpstring, "devices/%s/status", NodeName);

      mqttClient.publish(tmpstring, "connected");

    } else {
    //  PRINT("failed, rc=");
     // PRINT(client.state());
     // PRINTL(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}