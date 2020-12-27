
#define DEBUG

#ifdef DEBUG
#define PRINT(x)  Serial.print (x)
#define PRINTL(x)  Serial.println (x)
#else
#define PRINT(x)
#define PRINTL(x)
#endif

#include <LowPower.h>

// ask_transmitter.pde
// -*- mode: C++ -*-
// Simple example of how to use RadioHead to transmit messages
// with a simple ASK transmitter in a very simple way.
// Implements a simplex (one-way) transmitter with an TX-C1 module
// Tested on Arduino Mega, Duemilanova, Uno, Due, Teensy, ESP-12

#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif

RH_ASK driver(100,11,12,10);
// RH_ASK driver(2000, 4, 5, 0); // ESP8266 or ESP32: do not use pin 11 or 2
// RH_ASK driver(2000, 3, 4, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85),
// RH_ASK driver(2000, PD14, PD13, 0); STM32F4 Discovery: see tx and rx on Orange and Red LEDS

#include "DHT.h" // Modified lines 155, 156 and 160 !!!
#define DHTPIN 7     // what digital pin the DHT22 is conected to
#define DHTPWRPIN 6     // what digital pin the DHT22 is conected to
#define DHTTYPE DHT22   // there are multiple kinds of DHT sensors
DHT dht(DHTPIN, DHTTYPE);

#define SENDINTERVAL 20 // interval between sending shit
uint16_t timepassed = 99999; // seconds since last send

struct sensorpacket {
  char id[8] = "THS0001";
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
} sensorpacket1;



void setup()
{
  // IO
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DHTPWRPIN, OUTPUT);
  pinMode(DHTPIN, INPUT );

  digitalWrite(LED_BUILTIN, HIGH);

#ifdef DEBUG
  Serial.begin(9600);	  // Debugging only
#endif


PRINT("configured 433 packet size: "); PRINTL(sizeof(sensorpacket));

  if (!driver.init())
    PRINTL("init failed");

  digitalWrite(LED_BUILTIN, LOW);
  blinkLed(5, 250);
}



long millisp;
void loop()
{
  updateValues2();
  delay(1000);
  /*
  // Sleep for 10 minutes (600 s / 8 s = 75)
  unsigned int sleepCounter;
  for (sleepCounter = 75; sleepCounter > 0; sleepCounter--)
  {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }

  PRINTL(millis());
 */
}

void updateValues() {
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));


  // Power up temp/humid/pressure sensor
  digitalWrite(DHTPWRPIN, HIGH);
  dht.begin();
  delay(2000); // Need to wait at least 2s for stable readings
  float rh = dht.readHumidity();
  float temp = dht.readTemperature();
  sensorpacket1.payload = (int)rh * 10;
  sensorpacket1.payload = (int)temp * 10;
  
  digitalWrite(DHTPWRPIN, LOW);
  digitalWrite(DHTPIN, LOW); //disable the internal pullup resistor (enabled by dht.begin)
  //pinmode(DHTPIN, INPUT); <-- this (input with pull-up resistor disabled) should consume equal amount of power as output set to low
delay(2000);
  // Voltage
  sensorpacket1.type = 4;
  sensorpacket1.payload = (int)readVcc();
  sendpacket();
  delay(50);

  // Temperature
  sensorpacket1.type = 1;
  if (isnan(temp)) {
    PRINTL("temp error");
    sensorpacket1.payload = -2000;
  } else sensorpacket1.payload = (int)temp * 10;
  sendpacket();
  delay(50);

  // Humidity
  sensorpacket1.type = 2;
  if (isnan(rh)) {
    PRINTL("rh error");
    sensorpacket1.payload = -2000;
  } else sensorpacket1.payload = (int)rh * 10;
  delay(50);

}


void updateValues2() {
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  // Voltage
  sensorpacket1.type = 4;
  sensorpacket1.payload = 101;
  sendpacket();
  delay(50);

  // Temperature
  sensorpacket1.type = 1;
  sensorpacket1.payload = 202;
  sendpacket();
  delay(50);

  // Humidity
  sensorpacket1.type = 2;
  sensorpacket1.payload = 303;
  sendpacket();
  delay(50);

}


void sendpacket() {
 // uint8_t tx_buf[sizeof(sensorpacket)] = {0};
 // memcpy(tx_buf, &sensorpacket1, sizeof(sensorpacket) );
//  byte tx_size = sizeof(sensorpacket);

  //driver.send((uint8_t *)tx_buf, tx_size);
  driver.send((uint8_t *)&sensorpacket1, sizeof(sensorpacket));
  driver.waitPacketSent();
  delay(500);
}



long readVcc() {
  long result; // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}


void blinkLed(int times, int blinkdelay) {
  for (int cnt = times; cnt > 0; cnt--) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(blinkdelay);
    digitalWrite(LED_BUILTIN, LOW);
    delay(blinkdelay);
  }
}
