#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile
//RH_ASK rf;
RH_ASK rf(2000, 12, 4, 5); // ESP8266: do not use pin 11


// PIN D2 = RF receive (interrupt 0)

#define DEBUG

#ifdef DEBUG
#define PRINT(x)  Serial.print (x)
#define PRINTL(x)  Serial.println (x)
#else
#define PRINT(x)
#define PRINTL(x)
#endif

struct {
  int temp = 0;
  int rh = 0;
  int p = 0;
  int r;
  int g;
  int b;
  int c;
} env;
//byte rx_buf[sizeof(env)] = {0};

/*union {
  int32_t combined;
  struct {
    int16_t speedref, dir;
  } decomp;
  } controls;*/

long prevRXmillis;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);


  if (!rf.init())
    Serial.println("init failed");
  
  PRINTL("Init done");
}

void loop() {
  // put your main code here, to run repeatedly:
  //uint8_t rx_buf[RH_ASK_MAX_MESSAGE_LEN];
  
  uint8_t rx_buf[(int)(ceil(sizeof(env)))];
  uint8_t buflen = sizeof(rx_buf);
  if (rf.recv(rx_buf, &buflen)) // Non-blocking
  {
    memcpy(&env, rx_buf, sizeof(env));

    PRINT("T/RH: "); PRINT(env.temp); PRINT(" / "); PRINTL(env.rh); 
  
    prevRXmillis = millis();
    
  }


  // Reset controls 500ms after last receive
  if (millis() - prevRXmillis > 500) {
    ;
  }

}
