#define PIN_LED 7
unsigned int cnt, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  cnt=toggle=0;
  digitalWrite(PIN_LED, toggle);
}

void loop() {
  delay(1000);
  while(cnt<10){
  toggle =toggle_state(toggle);
  digitalWrite(PIN_LED, toggle);
  delay(100);
  ++cnt;
  }
  toggle =toggle_state(toggle);
  digitalWrite(PIN_LED, toggle);
  while(1){
    ;
  }
}

int toggle_state(int toggle){
  return !(toggle);
}
