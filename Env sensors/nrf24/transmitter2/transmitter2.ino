#include <LowPower.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include "DHT.h" // Modified lines 155, 156 and 160 !!!
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>

/////// NODE SPECIFIC!!!!! ////////////////////////////
#define GATEWAYNR 1 // Nr of the gateway to connect to
#define SENSORNR 1 // Nr of this sensor node
#define TEMPSENTYPE 3 // 0 = NONE, 1 = DHT22, 2 = BME280, 3 = BMP280
#define OTHERSENTYPE 0 // 0 = NONE, 6 = CAPACITIVE WATER LEVEL
#define UPDATE_INTERVAL 150 // Update interval. value = seconds / 8. Example: 10 minutes = 600 s / 8 s = 75
///////////////////////////////////////////////////////

#define DHT22 1
#define BME280 2
#define BMP280 3

// Debug print
#define DEBUG

#ifdef DEBUG
#define PRINT(x)  Serial.print (x)
#define PRINTL(x)  Serial.println (x)
#else
#define PRINT(x)
#define PRINTL(x)
#endif

///////////////////////////////////
// Pin config
///////////////////////////////////

// Temp sensor
#define DHTPIN 7     // what digital pin the DHT22 is conected to

#define DHTTYPE DHT22   // there are multiple kinds of DHT sensors

#define WLEVELPIN A0  // Water level sensor analog input
#define TMPSDA A4     // SDA
#define TMPSCL A5     // SCL
#define TMPPWRPIN 6     // Temperature sensor power supply pin
#define SEALEVELPRESSURE_HPA (1013.25)


// NF24
#define NF24_PWR_PIN 8
#define NF24_CE_PIN 9
#define NF24_CSN_PIN 10
// MOSI 11
// MISO 12
// SCK  13

RF24 radio(NF24_CE_PIN, NF24_CSN_PIN); // CE, CSN
uint8_t address[6] = "xGWyy"; // x = sensor number, yy = gateway number
DHT dht(DHTPIN, DHTTYPE);

#if TEMPSENTYPE == BME280
Adafruit_BME280 bme; // I2C
#else
Adafruit_BMP280 bme; // I2C
#endif

#define SENDINTERVAL 20 // interval between sending shit
uint16_t timepassed = 99999; // seconds since last send



struct sensorpacket {
  char id[8] = "THS00YY";
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
} sensorpacket1;

long voltage = 0;
float waterlevelperc = 0;
struct trh {
  float rh = 0;
  float temp = 0;
  float pressure = 0;
} trh1;





long millisp;

unsigned int sendcount = 1;
unsigned int cyclecount = 1;


void setup()
{
  // IO
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TMPPWRPIN, OUTPUT);
  pinMode(DHTPIN, INPUT );

  pinMode(NF24_PWR_PIN, OUTPUT);

  digitalWrite(TMPPWRPIN, HIGH);
  digitalWrite(NF24_PWR_PIN, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
  
#ifdef DEBUG
  Serial.begin(9600);	  // Debugging only
#endif

  // Sensor IDs
  sprintf(address, "%dGW0%d", SENSORNR, GATEWAYNR); // syntax xGWyy x = sensor number yy = gateway
  sprintf(sensorpacket1.id, "THS000%d", SENSORNR);
  PRINT("nRF24 address ");
  for (int i = 0; i < sizeof(address); i++) {
    PRINT((char)address[i]);
  }
  PRINTL("");
  PRINT("sensor ID "); PRINTL(sensorpacket1.id);

  //InitNRF24();
  PRINT("configured 433 packet size: "); PRINTL(sizeof(sensorpacket));

  //  radio.begin();
  //  radio.openWritingPipe(address);
  //  radio.setPALevel(RF24_PA_MAX);
  //  radio.setDataRate(RF24_250KBPS );
  //  radio.stopListening();

  digitalWrite(LED_BUILTIN, LOW);
  blinkLed(5, 250);
  PRINTL("Initialized");
}


void InitNRF24() {
  radio.begin();
  if (radio.failureDetected) {
    radio.failureDetected = 0;           // Reset the detection value
    PRINTL("NRF24 failure recovery");
  }

  if (!radio.isChipConnected()) {
    PRINTL("NRF24 chip not connected");
  }
  radio.openWritingPipe(address); // Re-configure pipe addresses
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS );
  radio.stopListening();
  PRINTL("nRF24 inited");
}


void loop()
{
  updateValues();

  InitNRF24();

  sendValues();

  sendcount++;

  sleep();

  cyclecount++;
}


void updateValues() {
  PRINTL("Updating sensor values");
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  // Power up temp/humid/pressure sensor
  voltage = readVcc();

  switch (TEMPSENTYPE) {
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


  switch (OTHERSENTYPE) {
    case 1:
      break;
    case 6: // CAPACITIVE WATER LEVEL
      ReadCapWH();
      break;

   default:
      ;
      break;
  }
}


void readDHT(trh &_trh) {
  digitalWrite(TMPPWRPIN, HIGH);
  dht.begin();
  delay(2500); // Need to wait at least 2s for stable readings
  _trh.rh = dht.readHumidity();
  _trh.temp = dht.readTemperature();
  digitalWrite(TMPPWRPIN, LOW);
  digitalWrite(DHTPIN, LOW); //disable the internal pullup resistor (enabled by dht.begin) to prevent current leak
  //pinmode(DHTPIN, INPUT); <-- this (input with pull-up resistor disabled) should consume equal amount of power as output set to low

  PRINT("temperature: ");
  PRINTL(_trh.temp);
  PRINT("Pressure: ");
  PRINTL(_trh.pressure);
  PRINT("humidity: ");
  PRINTL(_trh.rh);
}


void readBME280(trh &_trh) {
  digitalWrite(TMPPWRPIN, HIGH);
  delay(100);

  bool status;
  status = bme.begin(0x76);
  if (!status) {
    PRINTL("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
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

  digitalWrite(TMPPWRPIN, LOW);
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

  PRINT("Voltage: ");
  PRINTL(result / 1000.0);
  return result;
}


// Water level (3.3v reference)
const int wlHigh = 376;
const int wlLow = 876;

void ReadCapWH() {
  digitalWrite(TMPPWRPIN, HIGH);
  delay(200);
  int raw = analogRead(WLEVELPIN);

  waterlevelperc = 100 * ((float)(raw - wlLow) / (float)(wlHigh - wlLow));
  waterlevelperc = constrain(waterlevelperc, 0, 100);
  PRINT("Water level %: ");
  PRINTL(waterlevelperc);

  digitalWrite(TMPPWRPIN, LOW);
}


void sendValues() {
  PRINTL("Sending data");

  // Voltage
  sensorpacket1.type = 4;
  sensorpacket1.payload = (int)voltage;
  sendpacket();

  // Temperature
  sensorpacket1.type = 1;
  if (TEMPSENTYPE != 0) {
    if (isnan(trh1.temp)) {
      PRINTL("temp error");
      sensorpacket1.payload = -2000;
    } else {
      sensorpacket1.payload = (int)(trh1.temp * 10.0);
    }
    sendpacket();
  }


  // Humidity
  if (TEMPSENTYPE == 1 || TEMPSENTYPE == 2) {
    sensorpacket1.type = 2;
    if (isnan(trh1.rh)) {
      PRINTL("rh error");
      sensorpacket1.payload = -2000;
    } else {
      sensorpacket1.payload = (int)(trh1.rh * 10.0);
    }
    sendpacket();
  }


  // Pressure
  if (TEMPSENTYPE == 2 || TEMPSENTYPE == 3) {
    sensorpacket1.type = 3;
    if (isnan(trh1.pressure)) {
      PRINTL("rh error");
      sensorpacket1.payload = -2000;
    } else {
      sensorpacket1.payload = (int)(trh1.pressure);
    }
    sendpacket();
  }

  // Water level
  if (OTHERSENTYPE == 6) {
    sensorpacket1.type = 6;
    sensorpacket1.payload = (int)round(waterlevelperc);
    PRINTL( sensorpacket1.payload);
    sendpacket();
  }
}


void sendpacket() {
  radio.write(&sensorpacket1, sizeof(sensorpacket1));
}


void sleep() {
  PRINTL("Going to sleep");
  delay(100);

  // Disable I2C to prevent current leak
  shutdownI2C();

  // nRF24 sleep
  radio.powerDown();

  // atmega sleep for 10 minutes (600 s / 8 s = 75)
  unsigned int sleepCounter;
  for (sleepCounter = 75; sleepCounter > 0; sleepCounter--)
  {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}


void shutdownI2C() {
  Wire.end();
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
}


void blinkLed(int times, int blinkdelay) {
  for (int cnt = times; cnt > 0; cnt--) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(blinkdelay);
    digitalWrite(LED_BUILTIN, LOW);
    delay(blinkdelay);
  }
}
