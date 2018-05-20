
void setup() {
  remote_setup();
  motor_setup();
  distance_sensor_setup();
  horn_setup();
  servo_setup();
}
void loop() {
  remote_loop();
  motor_loop();
  distance_sensor_loop();
  emergency_loop();
  horn_loop();
}
//****************servo*********************//
#include <Servo.h>
Servo face_direction_servo;
const int servo_pin = 9; //남는 디지털 번호
void servo_setup(){
  face_direction_servo.attach(servo_pin);
  facing_front();
}
void facing_front(){
  face_direction_servo.write(90);
}
void facing_left(){
  face_direction_servo.write(150);
}
void facing_right(){
  face_direction_servo.write(30);
}

//******************************************//
//****************horn*********************//
const int horn_pin = 14; //A0
bool horn_state_changed = false;
void horn_setup(){
  pinMode(horn_pin,OUTPUT);
}
void horn_loop(){
  process_horn_output();
}
void process_horn_output(){
  static unsigned long prevMillis = 0;
  const unsigned long intervalMillis = 1;
  static int horn_cnt = 0;
  if(horn_state_changed){
    unsigned long currMillis = millis();
    if(currMillis - prevMillis >= intervalMillis){
      prevMillis = currMillis;
      horn_cnt++;
      if(horn_cnt == 1000){
        horn_cnt = 0;
        horn_state_changed = false;
      }else{
        digitalWrite(horn_pin, horn_cnt%2);
      }
    }
  }
}
//*****************************************//
//****************remote*********************//
char remote_input;
bool remote_input_changed = false;

void remote_setup(){
  Serial.begin(9600);
}

void remote_loop(){
  check_remote_input(); //사용자의 입력을 받는 함수
  distribute_remote_input(); 
}

void check_remote_input(){
    if(Serial.available()){
      extern bool EMERGENCY_STATE_ENABLE;
      if(EMERGENCY_STATE_ENABLE) return;
      remote_input = Serial.read();
      remote_input_changed = true;
    }
}
enum {
  GOFORWARD ='f',
  GOBACKWARD = 'b',
  TURNLEFT = 'l',
  TURNRIGHT = 'r',
  STOP = 's',
  GOFORWARDLEFT = 'L',
  GOFORWARDRIGHT = 'R',
};
char driving_action = STOP;
bool driving_action_changed = false;

void distribute_remote_input(){
  if(remote_input_changed){
    remote_input_changed = false;
    if(remote_input == GOFORWARD ||
       remote_input == GOBACKWARD ||
       remote_input == TURNLEFT ||
       remote_input == TURNRIGHT ||
       remote_input == STOP){
      driving_action = remote_input;
      driving_action_changed = true;
    }
  }
}
//*****************************************//

//****************모터*********************//
const int frontLeft=0;
const int frontRight=2;
const int backLeft=4;
const int backRight=6;

void motor_setup(){
  pinMode(frontLeft,OUTPUT);
  pinMode(frontRight,OUTPUT);
  pinMode(backLeft,OUTPUT);
  pinMode(backRight,OUTPUT);
}
void motor_loop(){
  process_driving_action();
}
void process_driving_action(){
  if(driving_action_changed){
    driving_action_changed = false;
    switch(driving_action){
      case GOFORWARD:
        go_forward();
        break;
      case GOBACKWARD:
        go_backward();
        break;
      case TURNLEFT:
        turn_left();
        break;
      case TURNRIGHT:
        turn_right();
        break;
      case STOP:
        stop_driving();
        break;
      case GOFORWARDLEFT:
        turn_left();
        break;
      case GOFORWARDRIGHT:
        turn_right();
        break;      
      default:
        break;
    }
  }
}
void go_forward(){
  digitalWrite(frontLeft,HIGH);
  analogWrite(frontLeft+1,250);
  digitalWrite(frontRight,HIGH);
  analogWrite(frontRight+1,250);
  digitalWrite(backLeft,HIGH);
  analogWrite(backLeft+1,250);
  digitalWrite(backRight,HIGH);
  analogWrite(backRight+1,250);
}
void go_backward(){
  digitalWrite(frontLeft,LOW);
  analogWrite(frontLeft+1,250);
  digitalWrite(frontRight,LOW);
  analogWrite(frontRight+1,250);
  digitalWrite(backLeft,LOW);
  analogWrite(backLeft+1,250);
  digitalWrite(backRight,LOW);
  analogWrite(backRight+1,250);
}
void turn_left(){
  digitalWrite(frontLeft,LOW);
  analogWrite(frontLeft+1,250);
  digitalWrite(frontRight,HIGH);
  analogWrite(frontRight+1,250);
  digitalWrite(backLeft,LOW);
  analogWrite(backLeft+1,250);
  digitalWrite(backRight,HIGH);
  analogWrite(backRight+1,250);
}
void turn_right(){
  digitalWrite(frontLeft,HIGH);
  analogWrite(frontLeft+1,250);
  digitalWrite(frontRight,LOW);
  analogWrite(frontRight+1,250);
  digitalWrite(backLeft,HIGH);
  analogWrite(backLeft+1,250);
  digitalWrite(backRight,LOW);
  analogWrite(backRight+1,250);
}
void stop_driving(){
  digitalWrite(frontLeft,HIGH);
  analogWrite(frontLeft+1,0);
  digitalWrite(frontRight,HIGH);
  analogWrite(frontRight+1,0);
  digitalWrite(backLeft,HIGH);
  analogWrite(backLeft+1,0);
  digitalWrite(backRight,HIGH);
  analogWrite(backRight+1,0);
}

//*****************************************//

//****************초음파 센서*********************//
#include <PinChangeInterrupt.h>

const int trigPin = 11;
const int echoPin = 12;
unsigned long distance_input = 60; //cm
bool distance_input_changed = false;

void echoIsr(){
  static unsigned long echoBegin = 0;
  static unsigned long echoEnd = 0;

  unsigned int echoPinState = digitalRead(echoPin);
  if(echoPinState == HIGH){
    echoBegin = micros();
  }else{
    echoEnd = micros();
    unsigned long echoDuration = echoEnd - echoBegin;
    distance_input = echoDuration / 58;
    distance_input_changed = true;
  }
}
void distance_sensor_setup(){
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  attachPCINT(digitalPinToPCINT(echoPin),echoIsr,CHANGE);
}
void distance_sensor_loop(){
  triggering_distance_sensor();
  check_distance_sensor_input();
}
void triggering_distance_sensor(){
  static unsigned long prevMillis = 0;
  const unsigned long intervalMillis = 100;
  unsigned long currentMillis = millis();
  if(currentMillis - prevMillis >= intervalMillis){
      prevMillis = currentMillis;
      extern bool EMERGENCY_STATE_ENABLE;
      if(EMERGENCY_STATE_ENABLE) return;
      ultrasonic_sensor_triggering();
  }
}
void ultrasonic_sensor_triggering(){
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
}
unsigned long leftDistance = 0;
unsigned long rightDistance = 0;
void check_left_distance(){
  leftDistance = distance_input;
  Serial.print("left(cm): ");
  Serial.println(leftDistance);
}
void check_right_distance(){
  rightDistance = distance_input;
  Serial.print("right(cm): ");
  Serial.println(rightDistance);
}
void turn_left_or_right(){
  if(leftDistance >= rightDistance){
    driving_action = GOFORWARDLEFT;
    driving_action_changed = true;
  }else{
    driving_action = GOFORWARDRIGHT;
    driving_action_changed = true;
  }
}
enum {
  SOMETHING_NEAR = 30
};
bool EMERGENCY_STATE_ENABLE = false;
void check_distance_sensor_input(){
  if(distance_input_changed){
    distance_input_changed = false;
    Serial.print("Distance: ");
    Serial.print(distance_input);
    Serial.println(" cm");

    static int emergencyLevel = 0;
    if(distance_input <= SOMETHING_NEAR){
      emergencyLevel++;
      if(emergencyLevel == 7){
        //stop
        EMERGENCY_STATE_ENABLE = true;
        emergencyLevel = 0;
      }
    }else{
        if(emergencyLevel >= 0){
          emergencyLevel--;  
        }
    }
    
  }
}
//************************************//

//****************비상상황***************//
void emergency_loop(){
  //비상 상태 처리
  static unsigned long prevMillis = 0;
  const unsigned long intervalMillis = 100;
  static int process_emergency_state = 0;
  if(EMERGENCY_STATE_ENABLE){
    unsigned long currentMillis = millis();
    if(currentMillis - prevMillis >= intervalMillis){
        prevMillis = currentMillis;
        process_emergency_state++;
        if(process_emergency_state == 1){
          //1. 정지한다
          driving_action = STOP;
          driving_action_changed = true;
          //2. LED 킨다
          //3. 경적을 울린다
          horn_state_changed = true;
        }else if(process_emergency_state == 50){
          //4. 후진한다
          driving_action = GOBACKWARD;
          driving_action_changed = true;
        }else if(process_emergency_state == 150){
          // 멈춘다
          driving_action = STOP;
          driving_action_changed = true;
          //5. 좌우를 살핀다
          facing_left();
          //왼쪽 거리, 오른쪽 거리 비교해서 방향 선택
          //6. 물체가 없는 쪽으로 회전한다
        }else if(process_emergency_state == 160){
          ultrasonic_sensor_triggering();
        }else if(process_emergency_state == 165){
          check_left_distance();
        }else if(process_emergency_state == 170){
          facing_right();
        }else if(process_emergency_state == 175){
          ultrasonic_sensor_triggering();
        }else if(process_emergency_state == 180){
          check_right_distance();
        }else if(process_emergency_state == 200){
          facing_front();
          turn_left_or_right();
        }else if(process_emergency_state == 250){
          //7. 비상 상태를 해제한다
          EMERGENCY_STATE_ENABLE = false;
          horn_state_changed = false;
          //8. LED 끈다
          //9. 주행 명령을 기다린다
          driving_action = GOFORWARD;
          driving_action_changed = true;          
        }
    }
  }
}
//************************************//
