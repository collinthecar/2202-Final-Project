#include <Servo.h>
#include <Wire.h>
#include <uSTimer2.h>
#include <EEPROM.h>
Servo drive_LeftMotor;
Servo drive_RightMotor;
Servo slideMotor;
Servo sweepMotor;

const int leftMotorOffsetAddressL = 0;
const int leftMotorOffsetAddressH = 1;

const int calibrateBtn = 13;

int leftMotorOffset;
byte b_LowByte;
byte b_HighByte;

int calInitialized = false;
long calStartTime;
int startingPos;
int slideUp;
int slideDown;
int sweepSidePos;
int wallDone;
int stage;
int sideVal;
int backVal;
int driveCor = 5; //change this to modify how much the drive motors should be corrected when bot too close/too far from wall
int wallTolerance = 10;
int defaultDriveSpeed = 1700;
int leftMotorSpeed;
int rightMotorSpeed;
int pyramidSensed;//ping time where pyramid is infront of us

const int slideMotorPin;
const int sweepMotorPin = 10;
const int leftMotorPin = 9;
const int rightMotorPin = 8;
const int pyramidButton;

float uSData[3];
int irData = 0;

void setup() {
  Serial.begin(2400);
  Wire.begin();
  pinMode(2, OUTPUT);
  pinMode(3, INPUT);
  pinMode(4, OUTPUT);
  pinMode(5, INPUT);
  pinMode(6, OUTPUT);
  pinMode(7, INPUT);
  pinMode(13, INPUT_PULLUP);

  Serial.println("Commenced");
  pinMode(slideMotorPin, OUTPUT);
  slideMotor.attach(slideMotorPin);
  pinMode(sweepMotorPin, OUTPUT);
  sweepMotor.attach(sweepMotorPin);
  pinMode(leftMotorPin, OUTPUT);
  drive_LeftMotor.attach(leftMotorPin);
  pinMode(rightMotorPin, OUTPUT);
  drive_RightMotor.attach(rightMotorPin);
  drive_LeftMotor.writeMicroseconds(1500);
  drive_RightMotor.writeMicroseconds(1500);
  slideMotor.write(slideUp);//start with the slide up so bot can traverse power packs
  b_LowByte = EEPROM.read(leftMotorOffsetAddressL);
  b_HighByte = EEPROM.read(leftMotorOffsetAddressH);
  leftMotorOffset = word(b_LowByte, b_HighByte);
  Serial.println("EEPROM read");
  //stage=1;
}
void loop() {
  /*
    Serial.println(stage);
    Serial.print("Left motor speed:");
    Serial.println(leftMotorSpeed);
    Serial.print("Right motor speed:");
    Serial.println(rightMotorSpeed);
  */
  checkSensor(1);
  Serial.println(uSData[1]);
  Serial.print("Motor offset:");
  Serial.println(leftMotorOffset);
  leftMotorOffset = constrain(leftMotorOffset, -50, 50);
  leftMotorSpeed = defaultDriveSpeed + leftMotorOffset;
  rightMotorSpeed = defaultDriveSpeed;
  //Calibration
  if (checkButton() == true && (millis() - calStartTime > 10000)) {
    Serial.println("Calibration initiated");
    calStartTime = millis();
    checkSensor(1);
    startingPos = uSData[1];
    leftMotorSpeed = defaultDriveSpeed + leftMotorOffset;
    rightMotorSpeed = defaultDriveSpeed;
    drive_LeftMotor.writeMicroseconds(leftMotorSpeed);
    drive_RightMotor.writeMicroseconds(rightMotorSpeed);
    while (millis() - calStartTime < 3000) {
      //do nothing until 3 seconds have passed
    }
    Serial.print("LEFT MOTOR SPEED: ");
    Serial.println(leftMotorSpeed);
    Serial.print("RIGHT MOTOR SPEED: ");
    Serial.println(rightMotorSpeed);
    checkSensor(1);
    leftMotorOffset += (startingPos - uSData[1]) * .05; //motor offset is a function of the final difference in ping times
    leftMotorOffset = constrain(leftMotorOffset, -50, 50);
  }

  EEPROM.write(leftMotorOffsetAddressL, lowByte(leftMotorOffset));
  EEPROM.write(leftMotorOffsetAddressH, highByte(leftMotorOffset));
  drive_LeftMotor.writeMicroseconds(1500);
  drive_RightMotor.writeMicroseconds(1500);
  Serial.println("Motors Stopped");

  stage=2;
  switch (stage) {
    case (1):
      //sweep arm from back to side incase cube is in back corner
      sweepMotor.writeMicroseconds(1300);
      delay(300);//change this delay to increase/decrease the sweep angle
      sweepMotor.writeMicroseconds(1500);//sweep arm should be in left position now
      Serial.println("Arm swept");
    //stage++;
    case (2):
      checkSensor(1);
      checkSensor(0);
      drive_LeftMotor.writeMicroseconds(leftMotorSpeed);
      drive_RightMotor.writeMicroseconds(rightMotorSpeed);
      //check side ultrasonic sensor to see if we are close enough to the wall
      if (uSData[1] > sideVal + wallTolerance) {

        leftMotorSpeed -= driveCor;
        rightMotorSpeed += driveCor;
        //too far from wall, need to correct path
      }
      else if (uSData[1] < sideVal - wallTolerance) {
        leftMotorSpeed += driveCor;
        rightMotorSpeed -= driveCor;
        //too close to wall, need to correct path
      }
      Serial.println();
      if (uSData[2] < wallDone) {
        //the bot has sweeped the whole wall, now we need to sweep the corner and then move on

        //stage++;
      }
    case (3):
      //need to back up from the wall first
      drive_LeftMotor.writeMicroseconds(1400 - leftMotorOffset);
      drive_RightMotor.writeMicroseconds(1400);
      while (uSData[0] < wallDone + 50) { //once ultrasonic ping is 50 away from the wall we can stop backing up
        checkSensor(0);
      }
      drive_LeftMotor.writeMicroseconds(1500);
      drive_RightMotor.writeMicroseconds(1500);
      stage++;
    case (4):
      drive_LeftMotor.writeMicroseconds(1600);
      delay(100);
      do {
        checkSensor(2);//keep turning until we are against a wall again
      } while (uSData[2] > sideVal);
      drive_LeftMotor.writeMicroseconds(1400 - leftMotorOffset);
      drive_RightMotor.writeMicroseconds(1400);
      do {
        checkSensor(3);
      } while (uSData[4] > backVal); //keep backing up until we are close enough to the wall behind us
      drive_LeftMotor.writeMicroseconds(1500);
      drive_RightMotor.writeMicroseconds(1500);//stop
      sweepMotor.writeMicroseconds(1400);
      delay(350);//sweep arm
      sweepMotor.writeMicroseconds(1500);
    case (5):
      drive_LeftMotor.writeMicroseconds(1600);
      drive_RightMotor.writeMicroseconds(1550);
      do {
        checkIR();
      } while (irData != 69 || irData != 65);
      drive_LeftMotor.writeMicroseconds(leftMotorSpeed);
      drive_RightMotor.writeMicroseconds(rightMotorSpeed);//drive straight towards the pyramid
      stage++;
    case (6):
      if (uSData[1] < pyramidSensed) {
        //pyramid is sensed
        slideMotor.write(slideDown);//drop the slide
        drive_LeftMotor.writeMicroseconds(1400);
        drive_RightMotor.writeMicroseconds(1400);
      }
  }
}
boolean checkButton() {
  int buttonVal = digitalRead(calibrateBtn);
  if (buttonVal == LOW) {
    delay(20);
    buttonVal = digitalRead(calibrateBtn);
    if (buttonVal == LOW) {
      return true;
    }
    else {
      return false;
    }
  }
  else {
    return false;
  }
}
void checkSensor(int uStoCheck) { //sensors are numbered 0-2 (0 is front, 1 is side, 2 is back)
  int x = uStoCheck * 2 + 2;
  int totalPing = 0;
  for (int i = 0; i < 10; i++) {
    digitalWrite(x, HIGH);
    delayMicroseconds(10);
    digitalWrite(x, LOW);
    totalPing += pulseIn(x + 1, HIGH, 10000);
  }
  if (totalPing / 10 > 0) {
    uSData[uStoCheck] = totalPing / 10;
  }
  Serial.print("Sensor ");
  Serial.print(uStoCheck);
  Serial.print(":");
  Serial.println(uSData[uStoCheck]);
}
void checkIR() {
  if (Serial.available() > 0) {
    irData = Serial.read();
  }
}

