// Debug print
#define DEBUG

#ifdef DEBUG
#define PRINT(x)  Serial.print (x)
#define PRINTL(x)  Serial.println (x)
#else
#define PRINT(x)
#define PRINTL(x)
#endif

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}


void plotScale(uint16_t mins, uint16_t maxs) {
  Serial.print(mins);
  Serial.print(" ");
  Serial.print(maxs);
  Serial.print(" ");
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t val;
  val = analogRead(A0);

  plotScale(0,1024);
  PRINTL(analogRead(A0));
}
