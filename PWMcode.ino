float duty =0;
int period =0;
int count =0;

void setup() {
  pinMode(7,OUTPUT);
  set_period(100);
  set_duty(100);
  Serial.begin(9600);
  }

  void set_period(int period1){
    period = period1; 
    count = 1000000/period1; //1초동안 설정된 period로 몇번 깜빡일지 저
  }

  void set_duty(int duty1){
    duty=duty1;
  }

void loop() {
  for(int i=0; i<count; i++){
    float On=float(duty/100)*abs(int(count/2-i));
    float Off=count/2-On;
    int On_time=On/(On+Off)*period;
    int Off_time=Off/(On+Off)*period;
    
    digitalWrite(7,HIGH);
    delayMicroseconds(On_time);
    digitalWrite(7,LOW);
    delayMicroseconds(Off_time); //period와 duty에 따른 LED조절

    Serial.println(On_time);
    Serial.println(Off_time);
    }

} 
