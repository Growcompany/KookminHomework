#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_TARGET 255 
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 410 // maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.1 // EMA weight of new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.

#define _DUTY_MIN 754 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1417 // servo neutral position (90 degree)
#define _DUTY_MAX 1972 // servo full counterclockwise position (180 degree)

// Servo speed control
#define _SERVO_ANGLE 30        // servo 각도 설정
#define _SERVO_SPEED 30        // servo 속도 설정

// Event periods
#define _INTERVAL_DIST 5 
#define _INTERVAL_SERVO 2 
#define _INTERVAL_SERIAL 5 

// PID parameters
#define _KP 0.0 


// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw,dist_cali, dist_ema, dist_prev, alpha; // unit: mm
float dist_target;
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
Servo myservo;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;


void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;
  
// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000); //[3128]

}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  // Event generator //이벤트 실행 간격 구현 
/////////////////////
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) event_dist = true;
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) event_servo = true;
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;


// get a distance reading from the USS
  if(event_serial) {
  dist_raw = ir_distance();
  if (dist_raw>60 && dist_raw<98)
  {dist_cali = 50*(dist_raw-68)/30+100;}
    else if (dist_raw>=98 && dist_raw<148)
    {dist_cali = 50*(dist_raw-98)/50+150;}
    else if (dist_raw>=148 && dist_raw<184)
    {dist_cali = 50*(dist_raw-148)/36+200;}
    else if (dist_raw>=184 && dist_raw<218)
    {dist_cali = 50*(dist_raw-184)/34+250;}
    else if (dist_raw>=218 && dist_raw<244)
    {dist_cali = 50*(dist_raw-218)/26+300;}
    else if (dist_raw>=244 && dist_raw<288)
    {dist_cali = 50*(dist_raw-244)/44+350;}
    else if (dist_raw>=288)
    {dist_cali = 50*(dist_raw-288)/60+400;}
    
  dist_ema = alpha*dist_cali+(1-alpha)*dist_ema;
  }

// output the read value to the serial port
  if(event_serial) {
  event_serial = false; // 
  Serial.print("raw:");
  Serial.print(dist_cali);
  Serial.print(",ema:");
  Serial.print(dist_ema);
  Serial.print(",servo:");
  Serial.print(myservo.read());  
  Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  last_sampling_time_serial = millis(); // 마지막 serial event 처리 시각 기록
  }

  if(event_servo) {
    if(dist_ema <= 255) {
         myservo.writeMicroseconds(int(1417-(dist_ema-255)*2.5));
      }
      else if(dist_ema > 255){
         myservo.writeMicroseconds(int(1417-(dist_ema-255)*2));
      }
   }
   
// update last sampling time
  last_sampling_time += INTERVAL;
}
