// 핀 번호 상수 정의
const int ledPin = 7;

void setup() {
  // 7번 핀을 출력 모드로 설정
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // 1. LED 켜기 (HIGH)
  digitalWrite(ledPin, LOW);
  delay(1000); // 1초 대기

  // 2. LED 5회 깜빡이기
  for (int i = 0; i < 5; i++) {
    digitalWrite(ledPin, HIGH);   
    delay(100);                 
    digitalWrite(ledPin, LOW); 
    delay(100);                  
  }

  // 3. LED 끄기
  digitalWrite(ledPin, HIGH);

  // 4. 무한 대기 (loop 종료)
  while (1) {
    // 아무 동작 없음
  }
}
