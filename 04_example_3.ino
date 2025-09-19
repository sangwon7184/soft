#define PIN_LED 13  

int count = 0;
int toggle = 0;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    ; 
  }
  Serial.println("Hello World!");
  digitalWrite(PIN_LED, toggle);
}

void loop() {
  Serial.println(++count);
  toggle = toggle_state(toggle); 
  digitalWrite(PIN_LED, toggle); 
  delay(1000);
}

int toggle_state(int t) {
  return !t; 
}
