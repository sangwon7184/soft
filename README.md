  #include <Servo.h>
  
  // Arduino pin assignment
  #define PIN_LED   9   // LED active-low
  #define PIN_TRIG  12  // sonar sensor TRIGGER
  #define PIN_ECHO  13  // sonar sensor ECHO
  #define PIN_SERVO 10  // servo motor
  
  // configurable parameters for sonar
  #define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
  #define INTERVAL 25      // sampling interval (unit: msec)
  #define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
  #define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
  #define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)
  
  #define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
  #define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance
  
  #define _EMA_ALPHA 0.2    // EMA weight of new sample (range: 0 to 1)
                            // Setting EMA to 1 effectively disables EMA filter.
  
  // Target Distance
  #define _TARGET_LOW  250.0
  #define _TARGET_HIGH 290.0
  
  // duty duration for myservo.writeMicroseconds()
  // NEEDS TUNING (servo by servo)
   
  #define _DUTY_MIN 500 // servo full clockwise position (0 degree)
  #define _DUTY_MAX 2700 // servo full counterclockwise position (180 degree)
  
  // global variables
  float  dist_ema, dist_prev = _DIST_MAX; // unit: mm
  unsigned long last_sampling_time;       // unit: ms
  int _DUTY_NEU = (_DUTY_MIN + _DUTY_MAX)/2;
  
  Servo myservo;
  
  void setup() {
    // initialize GPIO pins
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
    pinMode(PIN_ECHO, INPUT);     // sonar ECHO
    digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 
  
    myservo.attach(PIN_SERVO); 
    myservo.writeMicroseconds(_DUTY_NEU);
  
    // initialize USS related variables
    dist_prev = _DIST_MIN; // raw distance output from USS (unit: mm)
  
    // initialize serial port
    Serial.begin(57600);
  }
  
  void loop() {
    float  dist_raw, dist_filtered;
    
    // wait until next sampling time.
    // millis() returns the number of milliseconds since the program started. 
    // will overflow after 50 days.
    if (millis() < last_sampling_time + INTERVAL)
      return;
  
    // get a distance reading from the USS
    dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);
  
    // the range filter
    // 범위 필터
    if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX) || (dist_raw < _DIST_MIN)) {
      dist_filtered = dist_prev; // 범위 밖이면 이전 값 유지
      digitalWrite(PIN_LED, HIGH); // LED ON (범위 밖)
    } else {
      dist_filtered = dist_raw;
      dist_prev = dist_raw;
      digitalWrite(PIN_LED, LOW); // LED OFF (범위 안)
    }
  
    // Modify the below line to implement the EMA equation
    
    dist_ema = _EMA_ALPHA * dist_filtered + (1 - _EMA_ALPHA) * dist_ema; 
    
    // adjust servo position according to the USS read value
    // add your code here!

    // 거리 → 각도 매핑
    int angle;
    if (dist_ema <= _DIST_MIN) {
      angle = 0;
    } else if (dist_ema >= _DIST_MAX) {
      angle = 180;
    } else {
      angle = map((int)dist_ema, (int)_DIST_MIN, (int)_DIST_MAX, 0, 180);
    }
  
    // 각도 → 펄스폭 변환 후 서보 제어
    int duty = map(angle, 0, 180, _DUTY_MIN, _DUTY_MAX);
    myservo.writeMicroseconds(duty);
  
  
    // output the distance to the serial port
    Serial.print("Min:");    Serial.print(_DIST_MIN);
    Serial.print(",Low:");   Serial.print(_TARGET_LOW);
    Serial.print(",dist:");  Serial.print(dist_raw);
    Serial.print(",Servo:"); Serial.print(myservo.read());  
    Serial.print(",High:");  Serial.print(_TARGET_HIGH);
    Serial.print(",Max:");   Serial.print(_DIST_MAX);
    Serial.println("");
   
    // update last sampling time
    last_sampling_time += INTERVAL;
  }
  
  // get a distance reading from USS. return value is in millimeter.
  float USS_measure(int TRIG, int ECHO)
  {
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(PULSE_DURATION);
    digitalWrite(TRIG, LOW);
    
    return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
  }
