// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)
#define _DIST_N 30 // the number of Samples to get a median value
// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, median; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
int N=_DIST_N;
float samples[_DIST_N]={0};
float sortedS[_DIST_N]={0};

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO,INPUT);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;
// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
  median = Get_Median(dist_raw);
  
// output the read value to the serial port
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.print("median:");
  Serial.print(map(median,0,400,100,500));
  Serial.print(",");
  Serial.println("Max:500");

// turn on the LED if the distance is between dist_min and dist_max
  if(dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255);
  }
  else {
    analogWrite(PIN_LED, 0);
  }

// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.
  return reading;
}

//get median number function
float Get_Median(float Raw){
  float ans;
  int a, pos;
  //updating samples
  for(int i=0; i<N-1;i++){
    samples[i]=samples[i+1];
  }
  samples[N-1]=Raw;

  //input samples to sortedS
  for(int i=0; i<N; i++){
    sortedS[i]=samples[i];
  }

 
  //bubble sorting: sortedS
  for(int i=N; i>0; i--){
    for(int j=0; j<i-1;j++){
      if (sortedS[j]>sortedS[j+1]){
        temp=sortedS[j+1];
        sortedS[j+1]=sortedS[j];
        sortedS[j]=temp;
      }
    }
  }

  //return median num in sortedS
  if(N%2) {
    a=(int)((N-1)/2);
    ans= sortedS[a];
  }
  else {
    a=(int)(N/2);
    ans= (sortedS[a]+sortedS[a-1])/2.0;
  }
  
  return ans;
}
