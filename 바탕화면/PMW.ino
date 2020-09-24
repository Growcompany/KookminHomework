float SmoothPower = 500;

void setup() {
    pinMode(6,OUTPUT);

}

void loop() {
  for (int i=0;i<SmoothPower;i++){
    float pwm_value = 255.0*(1.0 -  abs((2.0*(i/SmoothPower))-1.0));
    analogWrite(6,int(pwm_value));
    delay(2);
  }
}
