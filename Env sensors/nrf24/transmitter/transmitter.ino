#include <LowPower.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include "DHT.h" // Modified lines 155, 156 and 160 !!!
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

/////// NODE SPECIFIC!!!!! ////////////////////////////
#define GATEWAYNR 1 // Nr of the gateway to connect to
#define SENSORNR 2 // Nr of this sensor node
#define TMPSENTYPE 2 // 1 = DHT22, 2 = BME280
///////////////////////////////////////////////////////

// Debug print
//#define DEBUG

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
#define DHTPWRPIN 6     // what digital pin the DHT22 is conected to
#define DHTTYPE DHT22   // there are multiple kinds of DHT sensors

#define TMPSDA A4     // SDA
#define TMPSCL A5     // SCL
#define TMPPWRPIN 6     // what digital pin the DHT22 is conected to
#define SEALEVELPRESSURE_HPA (1013.25)


// NF24
#define NF24_PWR_PIN 8
#define NF24_CE_PIN 9
#define NF24_CSN_PIN 10


RF24 radio(NF24_CE_PIN, NF24_CSN_PIN); // CE, CSN
uint8_t address[6] = "xGWyy"; // x = sensor number, yy = gateway number
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BME280 bme; // I2C

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
  */
  int8_t packetnr = 0; // running packet number
  int8_t type = 0;
  int16_t payload = 0;
} sensorpacket1;


struct trh {
  float rh = 0;
  float temp = 0;  
  float pressure = 0;
} trh1;


long voltage = 0;

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
}



void loop()
{

  /*
   * Due to some feature in RF24 library, it requires 1 run cycle after powering down the radio
   * (using external VCC cut circuit, not the powerdown method) to avoid re-initialization problems
   * when waking up again. Therefore must take turns in main loop between sending and powering down.
   */
   
  if (cyclecount % 2 == 1) {
    PRINTL("Update values");
    updateValues();

    

    digitalWrite(NF24_PWR_PIN, LOW); // PNP 
    delay(100);
    PRINTL("re-init radio");
    InitNRF24();
    PRINTL("send values");
    sendValues();

    sendcount++;
  } else {
   // radio.powerDown();

    
//    digitalWrite(NF24_CE_PIN, LOW);
//    digitalWrite(NF24_CSN_PIN, LOW);
//    digitalWrite(SCK, LOW);
//    digitalWrite(MISO, HIGH);
//    digitalWrite(MOSI, LOW);
    
    
    digitalWrite(NF24_PWR_PIN, HIGH); // PNP 
    PRINTL("power down");

    pinMode(NF24_CE_PIN, INPUT);
    //pinMode(NF24_CSN_PIN, INPUT);
    digitalWrite(NF24_CSN_PIN, LOW);
    pinMode(SCK, INPUT);
    pinMode(MISO, INPUT);
    pinMode(MOSI, INPUT);

 
    // Sleep for 10 minutes (600 s / 8 s = 75)
    unsigned int sleepCounter;
    for (sleepCounter = 75; sleepCounter > 0; sleepCounter--)
    {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    }


    pinMode(NF24_CE_PIN, OUTPUT);
   // pinMode(NF24_CSN_PIN, OUTPUT);
   // digitalWrite(NF24_CSN_PIN, HIGH);
    pinMode(SCK, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(MOSI, OUTPUT);

    PRINTL(millis());

  }

  cyclecount++;

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
  voltage = readVcc();

  switch (TMPSENTYPE) {
    case 1: // DHT
      readDHT(trh1);
     break;
    
    case 2: // BME280
      readBME280(trh1);
     break;
    
    default: 
     break;
  }

}


void readDHT(trh &_trh) {
  digitalWrite(DHTPWRPIN, HIGH);
  dht.begin();
  delay(2500); // Need to wait at least 2s for stable readings
  _trh.rh = dht.readHumidity();
  _trh.temp = dht.readTemperature();
  digitalWrite(DHTPWRPIN, LOW);
  digitalWrite(DHTPIN, LOW); //disable the internal pullup resistor (enabled by dht.begin) to prevent current leak
  //pinmode(DHTPIN, INPUT); <-- this (input with pull-up resistor disabled) should consume equal amount of power as output set to low
}


void readBME280(trh &_trh) {
  digitalWrite(DHTPWRPIN, HIGH);

  bool status;
  status = bme.begin();  
  if (!status) {
    PRINTL("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  
  _trh.rh = bme.readHumidity();
  _trh.temp = bme.readTemperature();
  _trh.pressure = bme.readPressure() / 100.0F;
 
  digitalWrite(DHTPWRPIN, LOW);
}


void sendValues() {
  // Voltage
  sensorpacket1.type = 4;
  sensorpacket1.payload = (int)voltage;
  sendpacket();


  // Temperature
  sensorpacket1.type = 1;
  if (isnan(trh1.temp)) {
    PRINTL("temp error");
    sensorpacket1.payload = -2000;
  } else {
    sensorpacket1.payload = (int)(trh1.temp * 10.0);
  }
  sendpacket();


  // Humidity
  sensorpacket1.type = 2;
  if (isnan(trh1.rh)) {
    PRINTL("rh error");
    sensorpacket1.payload = -2000;
  } else {
    sensorpacket1.payload = (int)(trh1.rh * 10.0);
  }
  sendpacket();

}



void sendpacket() {
  radio.write(&sensorpacket1, sizeof(sensorpacket1));
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
