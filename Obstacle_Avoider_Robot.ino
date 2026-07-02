// ================================================================
//  4WD Obstacle Avoider — 
//  HC-SR04 + IR sensor: left (pin 2), right (A0) + Buzzer(4) + L298N + Servo(A5)
//
//  (IR Sensor): 1 = clear, 0 = object detected
//  
//  L298N:
//  OUT1/OUT2 = RIGHT motors (enA pin 3, in1 pin 9, in2 pin 8)
//  OUT3/OUT4 = LEFT  motors (enB pin 5, in3 pin 7, in4 pin 6)
// ================================================================

#include <Servo.h>

#define enA         3
#define in1         9
#define in2         8
#define in3         7
#define in4         6
#define enB         5

#define BUZZER      4
#define ECHO_PIN   A2
#define TRIG_PIN   A3
#define SERVO_PIN  A5

#define SIDE_LEFT_PIN   2
#define SIDE_RIGHT_PIN  A0

#define LEFT_SIDE_ENABLED   true
#define RIGHT_SIDE_ENABLED  true

#define LEFT_DETECTED   LOW
#define RIGHT_DETECTED  LOW

#define OBSTACLE_DIST       32
#define TRULY_BLOCKED_DIST  12

#define SPEED_LEFT          150
#define SPEED_RIGHT         150
#define SPEED_TURN          150

//Use only if motors have diffrent speeds : it's to keep both side motors work same
#define LEFT_TRIM     75
#define RIGHT_TRIM     0

#define TIME_BACKUP         400
#define TIME_BUMP_BACKUP    300
#define TIME_TURN_90        460
#define TIME_TURN_180       920
#define SERVO_CENTER         80
#define SERVO_RIGHT          30
#define SERVO_LEFT          150
#define SERVO_SETTLE        350
#define US_SAMPLES             5 //Ultrasonic samples
#define OBSTACLE_CONFIRM       2

#define STUCK_TIME_LIMIT_MS  10000
#define MAX_BUMP_REPEATS_BEFORE_ESCAPE 2

Servo scanServo;

long dist_front = 400;
long dist_left  = 400;
long dist_right = 400;

uint8_t obstacleStreak = 0; //using 8 bit int to save memory 

unsigned long lastClearMoveTime = 0;
uint8_t consecutiveBumpCount = 0;

void setup()
{
  Serial.begin(9600); // open serial monitor

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  pinMode(SIDE_LEFT_PIN,  INPUT);
  pinMode(SIDE_RIGHT_PIN, INPUT);

  scanServo.attach(SERVO_PIN);
  servoCenter();

  stopMotors();

  delay(2000);
  
  // Keep the robot in a open space to test all the components first 
  Serial.println(F("================================"));
  Serial.println(F("   STARTUP TEST BEGINNING"));
  Serial.println(F("================================"));

  testBuzzer();
  testServo();
  testUltrasonic();
  testSideSensors();
  testMotors();

  Serial.println(F("================================"));
  Serial.println(F("   ALL TESTS COMPLETE"));
  Serial.println(F("   OBSTACLE AVOIDER STARTING"));
  Serial.println(F("================================"));

  beep(100); delay(100);
  beep(100); delay(100);
  beep(100);

  lastClearMoveTime = millis();
  delay(1000);
}

void loop()
{
  bool leftBump  = LEFT_SIDE_ENABLED  && (digitalRead(SIDE_LEFT_PIN)  == LEFT_DETECTED);
  bool rightBump = RIGHT_SIDE_ENABLED && (digitalRead(SIDE_RIGHT_PIN) == RIGHT_DETECTED);

  dist_front = readDistance();

  Serial.print(F("F=")); Serial.print(dist_front);
  Serial.print(F(" Lb=")); Serial.print(leftBump);
  Serial.print(F(" Rb=")); Serial.println(rightBump);

  if (millis() - lastClearMoveTime > STUCK_TIME_LIMIT_MS) {
    Serial.println(F("WATCHDOG: stuck too long - forcing escape"));
    forcedEscape();
    lastClearMoveTime = millis();
    consecutiveBumpCount = 0;
    return;
  }

  if (leftBump && rightBump) {
    Serial.println(F("BOTH SIDES BUMPED - backing up"));
    beep(80);
    moveBackward();
    delay(TIME_BUMP_BACKUP);
    stopMotors();
    delay(100);
    registerBump();
    return;
  }

  if (leftBump) {
    Serial.println(F("LEFT BUMP - backing up then steering right"));
    beep(60);
    moveBackward();
    delay(TIME_BUMP_BACKUP);
    stopMotors();
    delay(100);
    rotateRight90();
    registerBump();
    return;
  }

  if (rightBump) {
    Serial.println(F("RIGHT BUMP - backing up then steering left"));
    beep(60);
    moveBackward();
    delay(TIME_BUMP_BACKUP);
    stopMotors();
    delay(100);
    rotateLeft90();
    registerBump();
    return;
  }

  if (dist_front <= OBSTACLE_DIST) {
    obstacleStreak++;
  } else {
    obstacleStreak = 0;
  }

  if (obstacleStreak >= OBSTACLE_CONFIRM) {
    obstacleStreak = 0;
    obstacleRoutine();
    registerBump();
  } else {
    moveForward();
    lastClearMoveTime = millis();
    consecutiveBumpCount = 0;
  }

  delay(10);
}

void registerBump()
{
  consecutiveBumpCount++;
  if (consecutiveBumpCount >= MAX_BUMP_REPEATS_BEFORE_ESCAPE) {
    Serial.println(F("Too many repeated bumps - forcing escape"));
    forcedEscape();
    consecutiveBumpCount = 0;
    lastClearMoveTime = millis();
  }
}

void forcedEscape()
{
  beep(200); delay(100); beep(200); delay(100); beep(200);

  moveBackward();
  delay(700);
  stopMotors();
  delay(150);

  if ((millis() / 1000) % 2 == 0) {
    rotateLeft180();
  } else {
    rotateRight90();
    rotateRight90();
  }

  stopMotors();
  delay(200);
}

void testBuzzer()
{
  Serial.println(F(""));
  Serial.println(F("TEST 1 - BUZZER"));
  for (int i = 0; i < 3; i++) {
    beep(200);
    delay(200);
  }
  Serial.println(F("BUZZER DONE"));
  delay(500);
}

void testServo()
{
  Serial.println(F(""));
  Serial.println(F("TEST 2 - SERVO"));
  scanServo.write(SERVO_RIGHT);
  Serial.println(F("RIGHT"));
  delay(700);
  scanServo.write(SERVO_LEFT);
  Serial.println(F("LEFT"));
  delay(700);
  servoCenter();
  Serial.println(F("CENTER"));
  delay(500);
  Serial.println(F("SERVO DONE"));
  delay(500);
}

void testUltrasonic()
{
  Serial.println(F(""));
  Serial.println(F("TEST 3 - ULTRASONIC"));
  for (int i = 0; i < 6; i++) {
    long d = readDistance();
    Serial.print(F("Distance = "));
    Serial.print(d);
    Serial.println(F(" cm"));
    if (d == 0) {
      Serial.println(F("!! 0 = no signal - check wiring !!"));
      beep(500);
    } else {
      beep(80);
    }
    delay(500);
  }
  Serial.println(F("ULTRASONIC DONE"));
  delay(500);
}

void testSideSensors()
{
  Serial.println(F(""));
  Serial.println(F("TEST 4 - SIDE SENSORS"));
  Serial.println(F("Reading 8 times - put hand near LEFT then RIGHT sensor"));

  for (int i = 0; i < 8; i++) {
    int l = digitalRead(SIDE_LEFT_PIN);
    int r = digitalRead(SIDE_RIGHT_PIN);

    bool lDetected = LEFT_SIDE_ENABLED  && (l == LEFT_DETECTED);
    bool rDetected = RIGHT_SIDE_ENABLED && (r == RIGHT_DETECTED);

    Serial.print(F("LEFT raw="));  Serial.print(l);
    Serial.print(F(" det=")); Serial.print(lDetected);
    Serial.print(F("   RIGHT raw=")); Serial.print(r);
    Serial.print(F(" det=")); Serial.println(rDetected);

    if (lDetected || rDetected) {
      beep(80);
    }

    delay(400);
  }

  Serial.println(F("SIDE SENSOR TEST DONE"));
  delay(500);
}

void testMotors()
{
  Serial.println(F(""));
  Serial.println(F("TEST 5 - MOTORS"));
  delay(300);

  Serial.println(F("LEFT motors FORWARD"));
  analogWrite(enB, 150);
  analogWrite(enA, 0);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
  delay(1500);
  stopMotors();
  delay(500);

  Serial.println(F("RIGHT motors FORWARD"));
  analogWrite(enA, 150);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);  digitalWrite(in4, LOW);
  delay(1500);
  stopMotors();
  delay(500);

  Serial.println(F("ALL motors FORWARD (with trim)"));
  setSpeed(SPEED_LEFT + LEFT_TRIM, SPEED_RIGHT + RIGHT_TRIM);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  delay(1500);
  stopMotors();
  delay(500);

  Serial.println(F("ALL motors BACKWARD"));
  setSpeed(SPEED_LEFT + LEFT_TRIM, SPEED_RIGHT + RIGHT_TRIM);
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  delay(1500);
  stopMotors();
  delay(500);

  Serial.println(F("ROTATE LEFT"));
  setSpeed(SPEED_TURN, SPEED_TURN);
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  delay(1500);
  stopMotors();
  delay(500);

  Serial.println(F("ROTATE RIGHT"));
  setSpeed(SPEED_TURN, SPEED_TURN);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  delay(1500);
  stopMotors();
  delay(500);

  Serial.println(F("MOTORS DONE"));
  delay(500);
}

void obstacleRoutine()
{
  beep(60);
  stopMotors();
  delay(100);

  moveBackward();
  delay(TIME_BACKUP);
  stopMotors();
  delay(100);

  scanServo.write(SERVO_RIGHT);
  delay(SERVO_SETTLE);
  dist_right = readDistance();

  scanServo.write(SERVO_LEFT);
  delay(SERVO_SETTLE);
  dist_left = readDistance();

  servoCenter();
  delay(SERVO_SETTLE);

  Serial.print(F("Scan L=")); Serial.print(dist_left);
  Serial.print(F("  R="));    Serial.println(dist_right);

  if (dist_left <= TRULY_BLOCKED_DIST && dist_right <= TRULY_BLOCKED_DIST) {
    blockedRoutine();
    return;
  }

  if (dist_left > dist_right) {
    rotateLeft90();
  } else {
    rotateRight90();
  }

  long postTurnCheck = readDistance();
  Serial.print(F("Post-turn F=")); Serial.println(postTurnCheck);

  if (postTurnCheck <= OBSTACLE_DIST) {
    Serial.println(F("Still blocked after turn - turning again"));
    if (dist_left > dist_right) {
      rotateLeft90();
    } else {
      rotateRight90();
    }
  }
}

void blockedRoutine()
{
  stopMotors();
  Serial.println(F("ALL SIDES BLOCKED"));

  for (int attempt = 0; attempt < 3; attempt++)
  {
    sendSOS();

    scanServo.write(SERVO_RIGHT); delay(SERVO_SETTLE);
    dist_right = readDistance();

    scanServo.write(SERVO_LEFT);  delay(SERVO_SETTLE);
    dist_left = readDistance();

    servoCenter();                delay(SERVO_SETTLE);

    if (dist_left > TRULY_BLOCKED_DIST || dist_right > TRULY_BLOCKED_DIST) {
      Serial.println(F("Side cleared - resuming"));
      beep(300);
      if (dist_left > dist_right) rotateLeft90();
      else                        rotateRight90();
      return;
    }
  }

  Serial.println(F("Escape maneuver"));
  moveBackward();
  delay(800);
  stopMotors();
  delay(200);
  rotateLeft180();
  beep(200); delay(100); beep(200);
}

long readDistance()
{
  long samples[US_SAMPLES];
  int  valid = 0;

  for (int i = 0; i < US_SAMPLES; i++)
  {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long dur = pulseIn(ECHO_PIN, HIGH, 25000);
    if (dur > 0) {
      samples[valid] = dur / 29 / 2;
      valid++;
    }
    delay(5);
  }

  if (valid == 0) return 0;

  for (int i = 1; i < valid; i++) {
    long key = samples[i];
    int  j   = i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }

  int start = (valid >= 4) ? 1 : 0;
  int end   = (valid >= 4) ? valid - 1 : valid;

  long total = 0;
  for (int i = start; i < end; i++) total += samples[i];

  return total / (end - start);
}

void setSpeed(int leftSpd, int rightSpd)
{
  analogWrite(enB, constrain(leftSpd,  0, 255));
  analogWrite(enA, constrain(rightSpd, 0, 255));
}

void stopMotors()
{
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

void moveForward()
{
  setSpeed(SPEED_LEFT + LEFT_TRIM, SPEED_RIGHT + RIGHT_TRIM);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
}

void moveBackward()
{
  setSpeed(SPEED_LEFT + LEFT_TRIM, SPEED_RIGHT + RIGHT_TRIM);
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
}

void rotateLeft90()
{
  stopMotors(); delay(50);
  setSpeed(SPEED_TURN, SPEED_TURN);
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  delay(TIME_TURN_90);
  stopMotors(); delay(50);
}

void rotateRight90()
{
  stopMotors(); delay(50);
  setSpeed(SPEED_TURN, SPEED_TURN);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  delay(TIME_TURN_90);
  stopMotors(); delay(50);
}

void rotateLeft180()
{
  stopMotors(); delay(50);
  setSpeed(SPEED_TURN, SPEED_TURN);
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  delay(TIME_TURN_180);
  stopMotors(); delay(50);
}

void servoCenter()
{
  scanServo.write(SERVO_CENTER);
}

void beep(int ms)
{
  digitalWrite(BUZZER, HIGH);
  delay(ms);
  digitalWrite(BUZZER, LOW);
}

void sendSOS()
{
  for (int i = 0; i < 3; i++) { beep(150); delay(150); }
  delay(250);
  for (int i = 0; i < 3; i++) { beep(450); delay(150); }
  delay(250);
  for (int i = 0; i < 3; i++) { beep(150); delay(150); }
  delay(800);
}