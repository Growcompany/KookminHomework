#define PIN_LED 13
bool loop1=true;

void setup() {
  pinMode(PIN_LED, OUTPUT);
}

void loop() {
  while(loop1)
  {
  digitalWrite(PIN_LED, 1);
  delay(1000);
  for(int i=0; i<5; i++)
    {  
    digitalWrite(PIN_LED, 0);
    delay(100);
    digitalWrite(PIN_LED, 1);
    delay(100);   
    }
  loop1=false;
  digitalWrite(PIN_LED, 0);
  }
  
}
