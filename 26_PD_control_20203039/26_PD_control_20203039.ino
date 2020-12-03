#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9

#define PIN_SERVO 10                    //[3030]set PIN_SERVO 10
#define PIN_IR A0     //[3037]set PIN_IR A0

// Framework setting
#define _DIST_TARGET 255            // [3040] middle point of the rail plate(target point=25.5cm)
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.35  //[3023] set EMA value

// Servo range
#define _DUTY_MIN 1325   // 229mm 75 degree          
#define _DUTY_NEU 1675   // 194mm 109 degree 
#define _DUTY_MAX 2050   // 159mm 146 degree

// Servo speed control
#define _SERVO_ANGLE 71.0   //[3030] set servo angle range limit 146-63
#define _SERVO_SPEED 200.0 // [3040] Servo's angular speed(angle's amount of change per sec) 

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

// PID parameters
#define _KP 1.5//[3039] constant value for PID control
#define _KD 65.0

//Using in erasing noise
#define DELAY_MICROS  1000 //Park WooHuyk , Ema_alpha 0.35, INTERVAL_DIST 30
//==========================================================================//
//#define LENGTH 30   //Kim TaeWan
//#define k_LENGTH 8

//////////////////////
// global variables //
//////////////////////
// Servo instance     //[3046]
Servo myservo;                      //[3030]servo

const float coE[] = {0.0000633, -0.0197211, 2.8440471, -21.4650916};

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_cali, dist_ema, dist_alpha;      //[3034]distance detected by IR sensor and distance applied ema filter
float dist_min, dist_max;
// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;   //[3039] int value for action execution at interval

bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval;  //[3039] degree of servo per interval
int duty_target, duty_curr;      //[3030]servo target location, servo current location

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;
// [3042] current error, previous error and constant k, p,i,d

//Using in erasing noise
float samples_num = 4; //Park WooHuyk , Ema_alpha 0.35, INTERVAL_DIST 30
//float dist_list[LENGTH]; //Kim TaeWan

void setup() {
  // initialize GPIO pins for LED and attach servo
  pinMode(PIN_LED, OUTPUT);          //[3030]connect LED to PIN_LED[3027]
  myservo.attach(PIN_SERVO);  //[3039] connect Servo to PIN_SERVO

  // initialize global variables
  dist_target = _DIST_TARGET;   //dist
  dist_min = _DIST_MIN;                      //[3030] min range of measured value
  dist_max = _DIST_MAX;   //[3032] max range of measured value
  error_curr = _DIST_TARGET;
  error_prev = _DIST_TARGET;
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;
  event_dist = false;
  event_servo = false;
  event_serial = false;
  duty_curr = _DUTY_NEU;
  dist_alpha = _DIST_ALPHA;

  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);//[3030] initiate Servo's value to make the rail parallel

  // initialize serial port
  Serial.begin(57600);  //[3039] set speed of Serial monitor

  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0); //[3039]
  /*
    _SERVO_ANGLE/_SERVO_SPEED=
    (_DUTY_MAX - _DUTY_MIN_)* INTERVAL_SERVO/duty_chg_per_interval =
    interval repeat times *INTERVAL_SERVO=
    time taken to rotate as much as _SERVO_ANGLE
  */

}

//////////////////////
// helper method //
//////////////////////
void update_event_bools(void) {
  if (millis() > last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (millis() > last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (millis() > last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }
}

// method to estimate distance
float ir_distance(void) { // return value unit: mm
  float val; //[3031]
  float volt = float(analogRead(PIN_IR)); //[3031]
  val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0; //[3031]
  return val; //[3031]
}

//read = [121, 164, 186, 209, 235, 271]
//made by Choo HunJune
//https://colab.research.google.com/drive/1GvAVNB1u0lRZyzYpgCEGIWVllIJZ7vXD?usp=sharing#scrollTo=Kk_VTDtHRh1E
// method to correct raw distance
float ir_distance_cali(void) {
  float x = ir_distance();
  dist_cali = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
  return dist_cali;
}
//=================================
//made by Park WooHuyk
//https://github.com/pwh9882/IR_filter/blob/master/noise_filter/noise_filter.ino
float under_noise_filter(void) {
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance_cali();
    if (currReading > largestReading) {
      largestReading = currReading;
    }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float ir_distance_filtered(void) {
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) {
      lowestReading = currReading;
    }
  }
  dist_ema = _DIST_ALPHA * lowestReading + (1 - _DIST_ALPHA) * dist_ema;
  return dist_ema;
}


void loop() {
  /////////////////////
  // Event generator //
  /////////////////////
  update_event_bools();

  ////////////////////
  // Event handlers //
  ////////////////////

  if (event_dist) {
    float dist_filtered;
    event_dist = false;    // [3037]
    // get a distance reading from the distance sensor
    dist_filtered = ir_distance_filtered();   // [3037]
    dist_raw = dist_filtered;

    // PID control logic
    error_curr = dist_target - dist_filtered;
    pterm = error_curr;
    dterm = error_curr - error_prev;
    
    control = _KP * pterm + _KD * dterm;

    // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

    // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
    else if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

    error_prev = error_curr;
  }

  if (event_servo) {
    event_servo = false;  //[3034]

    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if (duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if (duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if (duty_curr < duty_target) duty_curr = duty_target;
    }//[3034]

    // update servo position
    myservo.writeMicroseconds(duty_curr);//[3034]

  }

  if (event_serial) {

    event_serial = false;
    
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    

  }
}



/* //linear cali
  float ir_distance_cali(float ir_dist){
  float x = ir_dist;
  dist_cali = 100+(250.0/(252-68))*(ir_dist-68);
  return dist_cali;
  }*/

/*
  //made by Kim TaeWan
  //https://gist.github.com/T-WK/715fde5441b68110a239bc1e0eda1db4
  float ir_distance_filtered() {
  int cnt = 0;
  float sum = 0;
  while (cnt < LENGTH) //the number of LENGTH
  {
    dist_raw=ir_distance();
    dist_list[cnt] = ir_distance_cali();
    sum += dist_list[cnt];
    cnt++;
  }


  // selection sort
  for (int i = 0; i < LENGTH-1; i++){
    for (int j = i+1; j < LENGTH; j++){
      if (dist_list[i] > dist_list[j]) {
        float tmp = dist_list[i];
        dist_list[i] = dist_list[j];
        dist_list[j] = tmp;
      }
    }
  }

  // dist_cali calculation process
  for (int i = 0; i < k_LENGTH; i++) {
    sum -= dist_list[i];
  }
  for (int i = 1; i <= k_LENGTH; i++) {
    sum -= dist_list[LENGTH-i];
  }

  dist_cali = sum/(LENGTH-2*k_LENGTH);

  // ema filter
  dist_ema = dist_alpha*dist_cali + (1-dist_alpha)*dist_ema;
  return dist_ema;
  } */

/*
  // [3030]
    Serial.print("dist_target:");
    Serial.print(dist_target);
    Serial.print(",dist_cali:");
    Serial.print(dist_cali);
    Serial.print(",dist_ema:");
    Serial.print(dist_ema);
    Serial.print(",error_curr:");
    Serial.print(error_curr);
    Serial.print(",duty_target:");
    Serial.print(duty_target);
    Serial.print(",duty_curr:");
    Serial.println(duty_curr);
*/
