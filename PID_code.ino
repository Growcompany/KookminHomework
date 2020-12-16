#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define _DIST_TARGET 255 
#define _INTERVAL_SERIAL 100 

#define _DUTY_MIN 754 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1417 // servo neutral position (90 degree)
#define _DUTY_MAX 1972 // servo full counterclockwise position (180 degree)

#define _ITERM_MAX 100

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_serial;

int Read = 0;
float distance = 0.0;
float elapsedTime, time, timePrev;        //Variables for time control
float distance_previous_error, distance_error, dist_filtered,dist_target;
int period = 50;  
int duty_target, duty_curr;

double Setpoint, Input, Output, ServoOutput;                                       


// PID parameters
#define _KP 8 
#define _KD 3200
#define _KI 0.2

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

Servo myservo;

void setup() {
  Serial.begin(57600);  
  myservo.attach(10); 
  myservo.write(82);
  pinMode(PIN_IR,INPUT);  
  time = millis();
  dist_target=_DIST_TARGET;
}

void loop() {
  if (millis() > time+period)
  {
    time = millis();    
    distance = get_dist(100); 
    if (distance<9.8) {dist_filtered=100+(distance-7.3)*20;}
    if (distance>=9.8 && distance<14.2) {dist_filtered=150+(distance-9.8)*11.3;}
    if (distance>=14.2 && distance<17.5) {dist_filtered=200+(distance-14.2)*15.1;}
    if (distance>=17.5 && distance<20.7) {dist_filtered=250+(distance-17.5)*15.6;}
    if (distance>=20.7 && distance<23.6) {dist_filtered=300+(distance-20.7)*16.1;}
    if (distance>=23.6 && distance<26.8) {dist_filtered=350+(distance-23.6)*15.6;}
    if (distance>=26.8) {dist_filtered=400+(distance-26.8)*10.2;}  
    distance_error = (_DIST_TARGET - dist_filtered)*0.1;   
    pterm = _KP * distance_error;
    float dist_diference = distance_error - distance_previous_error;     
    dterm = _KD*((distance_error - distance_previous_error)/period);
    iterm += (_KI * distance_error);

    if(abs(iterm)> _ITERM_MAX) iterm=0;
    if(iterm> _ITERM_MAX) iterm = _ITERM_MAX;
    if(iterm< -_ITERM_MAX) iterm = -_ITERM_MAX;
  
    control = pterm + iterm + dterm;  
    control = map(control, -150, 150, 0, 150);
  
    if(control < 20){control = 20;}
    if(control > 160) {control = 160; } 
    
    duty_target = 32 + control;
    duty_curr =myservo.read(); 
    myservo.write(duty_target);  
    distance_previous_error = distance_error;
  }

  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;
  // output the read value to the serial port
  if(event_serial) {
  event_serial = false; // 
  Serial.print("IR:");
  Serial.print(dist_filtered);
  Serial.print(",T:");
  Serial.print(dist_target);
  Serial.print(",P:");
  Serial.print(map(pterm,-1000,1000,510,610));
  Serial.print(",D:");
  Serial.print(map(dterm,-1000,1000,510,610));
  Serial.print(",I:");
  Serial.print(map(iterm,-1000,1000,510,610));
  Serial.print(",DTT:");
  Serial.print(map(duty_target,1000,2000,410,510));
  Serial.print(",DTC:");
  Serial.print(map(duty_curr,1000,2000,410,510)); 
  Serial.println(",-G:245, +G:265,m:0,M:800");
  last_sampling_time_serial = millis(); // 마지막 serial event 처리 시각 기록
  }
  
}

float get_dist(int n)
{
  long sum=0;
  for(int i=0;i<n;i++)
  {
    sum=sum+analogRead(PIN_IR);
  }  
  float adc=sum/n;
  float distance_cm = 17569.7 * pow(adc, -1.2062);
  return(distance_cm);
}
