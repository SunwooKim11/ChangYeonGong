#include <Servo.h>

Servo myservo;

#define PIN_SERVO 10
#define PIN_IR A0

void setup() {
  myservo.attach(PIN_SERVO);
  Serial.begin(57600);

}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
float raw_dist = ir_distance();
float modified_dist = 250*(raw_dist - 119)/(247-119) +150;
  Serial.print("min:0,max:500,raw_dist:");
  Serial.print(raw_dist);
  Serial.print("modified_dist:");
  Serial.println(modified_dist);
  if(modified_dist<255) myservo.writeMicroseconds(2050);
  else myservo.writeMicroseconds(1200);
  delay(20);

}
