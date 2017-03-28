#include <Servo.h>
Servo drive_LeftMotor;
Servo drive_RightMotor;
Servo slideMotor;
Servo sweepMotor;

int slideUp;
int slideDown;
int sweepSidePos;
int wallDone;
int stage;
int driveCor=5;//change this to modify how much the drive motors should be corrected when bot too close/too far from wall
int wallTolerance=10;
int leftMotorSpeed;
int rightMotorSpeed;

const int slideMotorPin;
const int sweepMotorPin;
const int leftMotorPin;
const int rightMotorPin;
const int pyramidButton;

float uSData[3];
int irData = 0;

void setup() {
  Serial.begin(2400);

  pinMode(slideMotorPin, OUTPUT);
  slideMotor.attach(slideMotorPin);
  pinMode(sweepMotorPin, OUTPUT);
  sweepMotor.attach(sweepMotorPin);
  pinMode(leftMotorPin, OUTPUT);
  drive_LeftMotor.attach(leftMotorPin);
  pinMode(rightMotorPin, OUTPUT);
  drive_RightMotor.attach(rightMotorPin);
  slideMotor.write(slideUp);//start with the slide up so bot can traverse power packs
}
void loop() {
  switch (stage) {
    case (1):
      //sweep arm from back to side incase cube is in back corner
      sweepMotor.writeMicroseconds(1600);
      delay(100);//change this delay to increase/decrease the sweep angle
      sweepMotor.writeMicroseconds(1500);//sweep arm should be in left position now

    case (2):
      leftMotorSpeed=1600;
      rightMotorSpeed=1600;
      drive_LeftMotor.writeMicroseconds(leftMotorSpeed);
      drive_RightMotor.writeMicroseconds(rightMotorSpeed);
      //check side ultrasonic sensor to see if we are close enough to the wall
      if (uSData[2] > sideVal+wallTolerance) {
        leftMotorSpeed-=driveCor;
        rightMotorSpeed+=driveCor;
        //too far from wall, need to correct path
      }
      else if (usData[2]<sideVal-wallTolerance){
        leftMotorSpeed+=driveCor;
        rightMotorSpeed-=driveCor;
      }
      if (uSData[0] < wallDone) {
        //the bot has sweeped the whole wall, now we need to sweep the corner and then move on
        stage++;
      }

    case (3):
      if (uSData[1] < pyramidSensed) {
        //pyramid is sensed
        slideMotor.write(slideDown);//drop the slide
        drive_LeftMotor.write();
        drive_RightMotor.write();
      }
  }
}
void checkSensor(const int uStoCheck) { //sensors are numbered 0-2 (0 is front, 1 is raised front, 2 is side)
  digitalWrite(uStoCheck, HIGH);
  delayMicroseconds(10);
  digitalWrite(uStoCheck, LOW);
  uSData[uStoCheck] = pulseIn(uStoCheck, HIGH, 10000);
}
void checkIR() {
  if (Serial.available() > 0) {
    irData = Serial.read();
  }
}

