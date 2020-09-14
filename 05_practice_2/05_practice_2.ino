#define PIN_LED 7
unsigned int cnt,toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  cnt=toggle=0;
}
void loop(){
  digitalWrite(PIN_LED, toggle);
  delay(1000);
  while(cnt<10){
    toggle = toggle_state(toggle);
    digitalWrite(PIN_LED, toggle);
    delay(100);
    ++cnt;
  }
  digitalWrite(PIN_LED, 1);
  while(1){
    delay(1000);
  }
}
int toggle_state(int toggle){
  return !(toggle);
}
