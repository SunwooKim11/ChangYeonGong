#define PIN_LED 7 
int T;
double D,P;
void setup() {
  pinMode(PIN_LED, OUTPUT);
  P=set_period(100); //P'sunit:ms
  T=(int)(5/P); //5ms=P*T
  Serial.begin(115200);
}
  
void loop() {

  for(int i=0;i<101;i++){
    D=set_duty(i);
    for(int j=0;j<T;j++){
    //one cycle
      digitalWrite(PIN_LED, 0);
      delayMicroseconds((int)(P*1000.0*D)); //delayMicroseconds(1000)==delay(1)
      digitalWrite(PIN_LED, 1);
      delayMicroseconds((int)(P*1000.0*(1-D)));
      Serial.println(i);
    }
    
  }
  for(int i=99;i>0;i--){
    D=set_duty(i);
    for(int j=0; j<T;j++){
    //one cycle
      digitalWrite(PIN_LED, 0);
      delayMicroseconds((int)(P*1000.0*D)); //delayMicroseconds(1000)==delay(1)
      digitalWrite(PIN_LED, 1);
      delayMicroseconds((int)(P*1000.0*(1-D)));
      Serial.println(i);
    }
    
  }
}

double set_duty(int duty){

  return ((double)duty/100.0);
}

double set_period(int period){
  //us->ms
  return ((double)period/1000.0);
}
