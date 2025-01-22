// 모터 드라이버 핀 정의
const int ENA = 9;  // 오른쪽 모터 (MOTORA)
const int IN1 = 8;
const int IN2 = 7;
const int IN3 = 6;
const int IN4 = 5;
const int ENB = 3;  // 왼쪽 모터 (MOTORB)

char lastCommand = 'X';  // 마지막으로 실행된 명령을 저장

void setup() {
  // 모든 모터 제어 핀을 출력으로 설정
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("ready for commands");
  stopMotors();  // 초기 상태는 정지
}

void setMotors(int leftSpeed, int rightSpeed) {
  // 왼쪽 모터 방향 설정
  if (leftSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    leftSpeed = -leftSpeed;
  }
  
  // 오른쪽 모터 방향 설정
  if (rightSpeed >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    rightSpeed = -rightSpeed;
  }
  
  // 모터 속도 설정
  analogWrite(ENB, leftSpeed);
  analogWrite(ENA, rightSpeed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch(command) {
      case 'W':
      case 'w':
        setMotors(100, 100);  // 전진
        Serial.println("Moving forward");
        break;
      case 'S':
      case 's':
        setMotors(-100, -100);  // 후진
        Serial.println("Moving backward");
        break;
      case 'A':
      case 'a':
        setMotors(10,190 );  // 좌회전
        Serial.println("Turning left");
        break;
      case 'D':
      case 'd':
        setMotors(190, 10);  // 우회전
        Serial.println("Turning right");
        break;
      case 'X':
      case 'x':
        stopMotors();  // 정지
        Serial.println("Stopped");
        break;
      default:
        Serial.println("Unknown command");
        break;
    }
  }
}
