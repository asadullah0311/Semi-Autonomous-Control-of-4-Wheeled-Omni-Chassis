/*CODE INCLUDES IR, MPU-6050 & LSA SENSOR*/

#include <MPU6050_light.h>
#include <PS2X_lib.h>
#include <Wire.h>

// IR sensor variables
int IRpin = 2;
volatile int count_circle = 0, dir_flag = 0;
int prev_IR = 0; //0 for black, 1 for white

//PID
const int Kp = 1;
const int Kd = 1;

//LSA
const int setPoint = 0;
const int minspeed = 100;
const int maxspeed = 200;
int last_PID_Error = 0, PID_error, motorSpeed, backlsa, leftlsa;
int speed1, speed2, speed3, speed4;
int goingl = 0; //flag for going left

//Motor Variables
int motor1DIR = 52;
int motor1PWM = 11;
int motor2DIR = 50;
int motor2PWM = 10;
int motor3DIR = 48;
int motor3PWM = 9;
int motor4DIR = 42;
int motor4PWM = 7;

//MPU
int yawError;
float yaw;
MPU6050 mpu(Wire);

//PS2
int ps2_error = 0;
byte type = 0;
byte vibrate = 0;

PS2X ps2x;



void Stop();

void setup() {
  
  ps2_error = ps2x.config_gamepad(47, 51, 49, 53, false, false);

  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);

  digitalWrite(motor1PWM, LOW);
  digitalWrite(motor2PWM, LOW);
  digitalWrite(motor3PWM, LOW);
  digitalWrite(motor4PWM, LOW);

  pinMode(IRpin, INPUT);

  pinMode(52, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(50, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(7, OUTPUT);

  /*---------------- Calibrating MPU-6050 -----------------*/
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { }

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  //delay(1000);
  mpu.calcOffsets();
  Serial.println("Done!\n");

}

void loop() {
  ps2x.read_gamepad(false, vibrate);
  /*------------- Obtaining the angle from the sensor --------------*/
  mpu.update();
  yaw = mpu.getAngleZ();
  Serial.println(yaw);
  /*------------- Storing the values in a variable which aren't in the range ------*/
  if (yaw > 2 || yaw < -2) {
    yawError = yaw;
  }

  backlsa = Serial1.read();
  leftlsa = Serial2.read();
  prev_IR = digitalRead(IRpin);

  Serial.print(backlsa);
  Serial.print("  ||  ");
  Serial.println(leftlsa);

  // If no line is detected, stay at the position
  if (backlsa == 255 && leftlsa == 255) {

    analogWrite(motor1PWM, 0);
    analogWrite(motor2PWM, 0);
    analogWrite(motor3PWM, 0);
    analogWrite(motor4PWM, 0);

    digitalWrite(motor1DIR, LOW);
    digitalWrite(motor2DIR, LOW);
    digitalWrite(motor3DIR, LOW);
    digitalWrite(motor4DIR, LOW);

    Serial.println("Stop, Stay calm");

  }
  else if (backlsa != 255 && leftlsa == 255) {
    //goingforward
    //goingl=0
    if (digitalRead(IRpin) == 0 && prev_IR == 1)
    {
      Stop();
    }
    else
    {
      digitalWrite(motor1DIR, LOW);
      digitalWrite(motor2DIR, LOW);
      digitalWrite(motor3DIR, LOW);
      digitalWrite(motor4DIR, LOW);

      PID_error = backlsa - setPoint;
      motorSpeed = Kp * PID_error + Kd * (PID_error - last_PID_Error);
      last_PID_Error = PID_error;

      // Adjust the motor speed based on calculated value
      speed1 = minspeed + motorSpeed;
      speed2 = minspeed - motorSpeed;
      speed3 = minspeed + motorSpeed;
      speed4 = minspeed - motorSpeed;

      if (speed1 > maxspeed) speed1 = maxspeed;
      if (speed2 > maxspeed) speed2 = maxspeed;
      if (speed3 > maxspeed) speed3 = maxspeed;
      if (speed4 > maxspeed) speed4 = maxspeed;

      if (speed1 < 0) speed1 = 0;
      if (speed2 < 0) speed2 = 0;
      if (speed3 < 0) speed3 = 0;
      if (speed4 < 0) speed4 = 0;


      // Writing the motor speed value as output to hardware motor
      analogWrite(motor1PWM, speed1);
      analogWrite(motor2PWM, speed2);
      analogWrite(motor3PWM, speed3);
      analogWrite(motor4PWM, speed4);
      //    Serial.println(speed1);
      //    Serial.println(speed2);
      //    Serial.println(speed3);
      //    Serial.println(speed4);
      goingl = 0 ;
      Serial.println("Forward");
    }
  }

  else if (backlsa == 255 && 0 < leftlsa < 70 ) {
    //going left
    if (digitalRead(IRpin) == 0 && prev_IR == 1)
    {
      Stop();
    }
    else
    {
      //goingl=1 karro
      digitalWrite(motor1DIR, HIGH);
      digitalWrite(motor2DIR, HIGH);
      digitalWrite(motor3DIR, HIGH);
      digitalWrite(motor4DIR, HIGH);

      PID_error = leftlsa - setPoint;
      motorSpeed = Kp * PID_error + Kd * (PID_error - last_PID_Error);
      last_PID_Error = PID_error;

      // Adjust the motor speed based on calculated value
      speed1 = minspeed + motorSpeed;
      speed2 = minspeed - motorSpeed;
      speed3 = minspeed + motorSpeed;
      speed4 = minspeed - motorSpeed;

      if (speed1 > maxspeed) speed1 = maxspeed;
      if (speed2 > maxspeed) speed2 = maxspeed;
      if (speed3 > maxspeed) speed3 = maxspeed;
      if (speed4 > maxspeed) speed4 = maxspeed;

      if (speed1 < 0) speed1 = 0;
      if (speed2 < 0) speed2 = 0;
      if (speed3 < 0) speed3 = 0;
      if (speed4 < 0) speed4 = 0;

      // Writing the motor speed value as output to hardware motor
      analogWrite(motor1PWM, speed1);
      analogWrite(motor2PWM, speed2);
      analogWrite(motor3PWM, speed3);
      analogWrite(motor4PWM, speed4);
      //    Serial.println(speed1);
      //    Serial.println(speed2);
      //    Serial.println(speed3);
      //    Serial.println(speed4);
      goingl = 1;
      Serial.println("Left");
    }
  }

  else if (backlsa != 255 && leftlsa != 255) //junction detected
  {
    if (digitalRead(IRpin) == 0 && prev_IR == 1)
    {
      Stop();
    }
    else
    {
      if (goingl == 0) {
        // going forward
        //tell to go left
        digitalWrite(motor1DIR, HIGH);
        digitalWrite(motor2DIR, HIGH);
        digitalWrite(motor3DIR, HIGH);
        digitalWrite(motor4DIR, HIGH);

        PID_error = leftlsa - setPoint;
        motorSpeed = Kp * PID_error + Kd * (PID_error - last_PID_Error);
        last_PID_Error = PID_error;

        // Adjust the motor speed based on calculated value
        speed1 = minspeed + motorSpeed;
        speed2 = minspeed - motorSpeed;
        speed3 = minspeed + motorSpeed;
        speed4 = minspeed - motorSpeed;

        if (speed1 > maxspeed) speed1 = maxspeed;
        if (speed2 > maxspeed) speed2 = maxspeed;
        if (speed3 > maxspeed) speed3 = maxspeed;
        if (speed4 > maxspeed) speed4 = maxspeed;

        if (speed1 < 0) speed1 = 0;
        if (speed2 < 0) speed2 = 0;
        if (speed3 < 0) speed3 = 0;
        if (speed4 < 0) speed4 = 0;

        // Writing the motor speed value as output to hardware motor
        analogWrite(motor1PWM, speed1);
        analogWrite(motor2PWM, speed2);
        analogWrite(motor3PWM, speed3);
        analogWrite(motor4PWM, speed4);
        //      Serial.println(speed1);
        //      Serial.println(speed2);
        //      Serial.println(speed3);
        //      Serial.println(speed4);
        Serial.println("Left after Junction");
      }
      else if (goingl == 1) { // tell to go backward

        digitalWrite(motor1DIR, LOW);
        digitalWrite(motor2DIR, LOW);
        digitalWrite(motor3DIR, LOW);
        digitalWrite(motor4DIR, LOW);

        PID_error = backlsa - setPoint;
        motorSpeed = Kp * PID_error + Kd * (PID_error - last_PID_Error);
        last_PID_Error = PID_error;

        // Adjust the motor speed based on calculated value
        speed1 = minspeed + motorSpeed;
        speed2 = minspeed - motorSpeed;
        speed3 = minspeed + motorSpeed;
        speed4 = minspeed - motorSpeed;

        if (speed1 > maxspeed) speed1 = maxspeed;
        if (speed2 > maxspeed) speed2 = maxspeed;
        if (speed3 > maxspeed) speed3 = maxspeed;
        if (speed4 > maxspeed) speed4 = maxspeed;

        if (speed1 < 0) speed1 = 0;
        if (speed2 < 0) speed2 = 0;
        if (speed3 < 0) speed3 = 0;
        if (speed4 < 0) speed4 = 0;

        // Writing the motor speed value as output to hardware motor
        analogWrite(motor1PWM, speed1);
        analogWrite(motor2PWM, speed2);
        analogWrite(motor3PWM, speed3);
        analogWrite(motor4PWM, speed4);
        //      Serial.println(speed1);
        //      Serial.println(speed2);
        //      Serial.println(speed3);
        //      Serial.println(speed4);
        Serial.println("Back After Junc.");
      }
    }
  }
}
void Stop() {
  count_circle++;
  Serial.print("Count_circle = ");
  Serial.println(count_circle);

  if (count_circle == 10)
  {
    if (dir_flag == 0)
      dir_flag = 1;
    else
      dir_flag = 0;
    count_circle = 0;

  }
  if ((dir_flag == 1) && (count_circle == 3 || count_circle == 5 || count_circle == 8))
  {
    analogWrite(motor1PWM, 0);
    analogWrite(motor2PWM, 0);
    analogWrite(motor3PWM, 0);
    analogWrite(motor4PWM, 0);

    if (yaw > 2) {

      PID_error = yawError - setPoint;
      motorSpeed = Kp * PID_error + Kd * (PID_error - last_PID_Error);
      last_PID_Error = PID_error;

      speed1 = minspeed + motorSpeed;
      speed2 = minspeed + motorSpeed;
      speed3 = minspeed + motorSpeed;
      speed4 = minspeed + motorSpeed;

      if (speed1 > maxspeed) speed1 = maxspeed;
      if (speed2 > maxspeed) speed2 = maxspeed;
      if (speed3 > maxspeed) speed3 = maxspeed;
      if (speed4 > maxspeed) speed4 = maxspeed;

      if (speed1 < 0) speed1 = 0;
      if (speed2 < 0) speed2 = 0;
      if (speed3 < 0) speed3 = 0;
      if (speed4 < 0) speed4 = 0;

      /*-------All clockwise--------*/
      digitalWrite(52, HIGH);
      analogWrite(11, speed1);
      digitalWrite(50, HIGH);
      analogWrite(10, speed2);
      digitalWrite(48, HIGH);
      analogWrite(9, speed3);
      digitalWrite(42, HIGH);
      analogWrite(7, speed4);

      if (yaw == 0) {
        analogWrite(motor1PWM, 0);
        analogWrite(motor2PWM, 0);
        analogWrite(motor3PWM, 0);
        analogWrite(motor4PWM, 0);
      }
    }
    if (yaw < -2) {
      PID_error = yawError - setPoint;
      motorSpeed = Kp * PID_error + Kd * (PID_error - last_PID_Error);
      last_PID_Error = PID_error;

      speed1 = minspeed - motorSpeed;
      speed2 = minspeed - motorSpeed;
      speed3 = minspeed - motorSpeed;
      speed4 = minspeed - motorSpeed;

      if (speed1 > maxspeed) speed1 = maxspeed;
      if (speed2 > maxspeed) speed2 = maxspeed;
      if (speed3 > maxspeed) speed3 = maxspeed;
      if (speed4 > maxspeed) speed4 = maxspeed;

      if (speed1 < 0) speed1 = 0;
      if (speed2 < 0) speed2 = 0;
      if (speed3 < 0) speed3 = 0;
      if (speed4 < 0) speed4 = 0;

      /*-------All anti-clockwise--------*/
      digitalWrite(52, LOW);
      analogWrite(11, speed1);
      digitalWrite(50, LOW);
      analogWrite(10, speed2);
      digitalWrite(48, LOW);
      analogWrite(9, speed3);
      digitalWrite(42, LOW);
      analogWrite(7, speed4);
      if (yaw == 0) {
        analogWrite(motor1PWM, 0);
        analogWrite(motor2PWM, 0);
        analogWrite(motor3PWM, 0);
        analogWrite(motor4PWM, 0);
      }
    }
  }
  Serial.println("Launch now ");

  //Code for Launching
}
//prev_IR = 0;
