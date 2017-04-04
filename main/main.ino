#include <Servo.h>
#include <Wire.h>
#include <uSTimer2.h>
#include <EEPROM.h>
#include <I2CEncoder.h>

Servo drive_LeftMotor;
Servo drive_RightMotor;
Servo slideMotor;
Servo sweepMotor;
bool calInitialized = false;
I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

const int ci_I2C_SDA = A4;
const int ci_I2C_SCL = A5;

const int leftMotorOffsetAddressL = 0;
const int leftMotorOffsetAddressH = 1;
const int rightMotorOffsetAddressL = 2;
const int rightMotorOffsetAddressH = 3;


const int calibrateBtn = 13;

int leftMotorOffset;
int rightMotorOffset;
byte b_LowByte;
byte b_HighByte;

long calStartTime;
int startingPos;
int slideUp = 90;
int slideDown = 120;
int sweepSidePos;
int wallDone;
int stage;
int sideVal = 420;
int backVal;
int driveCor = 5; //change this to modify how much the drive motors should be corrected when bot too close/too far from wall
int wallTolerance = 10;
int defaultDriveSpeed = 1700;
int leftMotorSpeed;
int rightMotorSpeed;
int pyramidSensed = 1600; //ping time where pyramid is infront of us

const int slideMotorPin = 11;
const int sweepMotorPin = 10;
const int leftMotorPin = 9;
const int rightMotorPin = 8;

long uSData[2];
int irData = 0;

void checkSensor(int uStoCheck);
void checkIR();
boolean checkButton();

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
  slideMotor.write(slideUp);//start with the slide up so bot can traverse power packs
  b_LowByte = EEPROM.read(leftMotorOffsetAddressL);
  b_HighByte = EEPROM.read(leftMotorOffsetAddressH);
  leftMotorOffset = word(b_LowByte, b_HighByte);
  b_LowByte=EEPROM.read(rightMotorOffsetAddressL);
  b_HighByte=EEPROM.read(rightMotorOffsetAddressH);
  rightMotorOffset=word(b_LowByte,b_HighByte);
  
  Serial.println("EEPROM read");
  //stage=1;
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
}

void loop() {
  
    
    Serial.print("Left motor offset:");
    Serial.println(leftMotorOffset);
    Serial.print("Right motor offset:");
    Serial.println(rightMotorOffset);
    /*
    Serial.print("LEFT ENCODER: ");
    Serial.println(encoder_LeftMotor.getRawPosition());
    Serial.print("RIGHT ENCODER: ");
    Serial.println(encoder_RightMotor.getRawPosition());
    */
  
  leftMotorSpeed = defaultDriveSpeed + leftMotorOffset;
  rightMotorSpeed = defaultDriveSpeed+rightMotorOffset;
  //Calibration
  /*Ultrasonic calibration
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
  */
  stage = 0;
  if (checkButton() == true && (millis() - calStartTime > 10000)) {
    if (!calInitialized) {
      Serial.println("Calibration initiated");
      calStartTime = millis();
      encoder_LeftMotor.zero();
      encoder_RightMotor.zero();
      calInitialized = true;
      drive_LeftMotor.writeMicroseconds(leftMotorSpeed);
      drive_RightMotor.writeMicroseconds(rightMotorSpeed);

    }
    if (millis() - calStartTime > 5000) {
      drive_LeftMotor.writeMicroseconds(1500);
      drive_RightMotor.writeMicroseconds(1500);
      if (encoder_LeftMotor.getRawPosition() > encoder_RightMotor.getRawPosition()) {
        leftMotorOffset = (encoder_LeftMotor.getRawPosition() - encoder_RightMotor.getRawPosition()) / 4;
        rightMotorOffset = 0;
      }
      else {
        rightMotorOffset = (encoder_RightMotor.getRawPosition() - encoder_LeftMotor.getRawPosition()) / 4;
        leftMotorOffset = 0;
      }
      Serial.print("Left motor offset: ");
      Serial.println(leftMotorOffset);
      Serial.print("Right motor offset: ");
      Serial.println(rightMotorOffset);
    }

  }

  EEPROM.write(leftMotorOffsetAddressL, lowByte(leftMotorOffset));
  EEPROM.write(leftMotorOffsetAddressH, highByte(leftMotorOffset));
  EEPROM.write(rightMotorOffsetAddressL, lowByte(rightMotorOffset));
  EEPROM.write(rightMotorOffsetAddressH, highByte(rightMotorOffset));

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
  digitalWrite(x, HIGH);
  delayMicroseconds(10);
  digitalWrite(x, LOW);
  uSData[uStoCheck] = pulseIn(x + 1, HIGH, 10000);
}
void checkIR() {
  if (Serial.available() > 0) {
    irData = Serial.read();
  }
}

