const int ENA = 9;  // 오른쪽 모터 (MOTORA)
const int IN1 = 8;
const int IN2 = 7;
const int IN3 = 6;
const int IN4 = 5;
const int ENB = 3;  // 왼쪽 모터 (MOTORB)

void setup() 
{
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  Serial.begin(9600);
}

void setMotors(int leftSpeed, int rightSpeed) 
{
  // 왼쪽 모터 방향 설정
  if (leftSpeed >= 0) 
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } 
  else 
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    leftSpeed = -leftSpeed;
  }
  
  // 오른쪽 모터 방향 설정
  if (rightSpeed >= 0) 
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } 
  else 
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    rightSpeed = -rightSpeed;
  }
  
  // 모터 속도 설정
  analogWrite(ENB, leftSpeed);
  analogWrite(ENA, rightSpeed);
}

void loop() 
{
  if (Serial.available() > 0) 
  {
    String input = Serial.readStringUntil('\n');
    int commaIndex = input.indexOf(',');
    
    if (commaIndex != -1) 
    {
      int leftPWM = input.substring(0, commaIndex).toInt();
      int rightPWM = input.substring(commaIndex + 1).toInt();
      
      setMotors(leftPWM, rightPWM);
      
      Serial.print("Left PWM: ");
      Serial.print(leftPWM);
      Serial.print(", Right PWM: ");
      Serial.println(rightPWM);
    }
  }
}