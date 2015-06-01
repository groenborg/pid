#include <Engine.h>
#include <pid_stabilizer.h>
#include <sliding_buffer.h>
#include <Controller.h>
#include <Servo.h>
#include <PS3BT.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Wire.h>

// variables
float angles[3]; // yaw pitch roll
float angleOffset = 0.0f;
float speedSum = 0.0f;

// PID koefficients
float Kp = 2.8f;   
float Ki = 4.7f;   
float Kd = 110.0f;

float maxVal = 115.0f;
float minVal = 55.0f;
float md = (maxVal + minVal)/2;
float range = md - minVal;

// objects
Engine *engine; 
Controller control;
FreeSixIMU sixDOF = FreeSixIMU();
PidControl pidControl(Kp,Ki,Kd, 900);

// timer test variables
long previousMills = 0; 
unsigned long currentMills = 0; 
long interval = 8; // interval for wait

// LED 
int powerLED = 26; // RED 
int calibrationLED = 28; // YELLOW
int USBLED = 30; // YELLOW
int readyLED = 32; // GREEN

void setup() {
  // pin setup and power on
  pinMode(powerLED, OUTPUT);
  pinMode(calibrationLED, OUTPUT);
  pinMode(USBLED, OUTPUT);
  pinMode(readyLED, OUTPUT);
  digitalWrite(powerLED, HIGH);
  
  // setting up engine and calibration
  engine = new Engine(2,3,4,5);
  engine->armEngine();
  Wire.begin();
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
  digitalWrite(calibrationLED,HIGH);
  calibrateGyro();
  digitalWrite(calibrationLED,LOW);
  
  //digitalWrite(USBLED,HIGH);
  // control.connectController();
  //digitalWrite(USBLED,LOW);
  
  digitalWrite(readyLED, HIGH);
  delay(1000);
}

boolean angleShift = false;

void loop() {
  currentMills = millis();
  
  // getting angles and calculating error value
  sixDOF.getEuler(angles);
  float error = (angles[2] - angleOffset)/180.0f;
  speedSum = pidControl.calcPID(error); 
  
  // check boundaries so we wont exceed max engine power
  if(speedSum > 1.0f) speedSum = 1.0f;
  if(speedSum < -1.0f) speedSum = -1.0f;
  
  // calculating speed and setting throttle
  float actSpeed = calculateSpeed();
  float actSpeedOpp = (md * 2) - actSpeed;  
  engine->engineSW(actSpeed + 0.0f);
  engine->engineSR(actSpeedOpp + 0.0f);
  
  // Setting a time alignment so we calculate at a fixed interval
  previousMills = currentMills;
  currentMills = millis();
  if(interval - (currentMills - previousMills) > 0){
    delay(interval - (currentMills - previousMills));
  }
  
}

//Calculates angle offset by avg of 4000 measurements
void calibrateGyro(){
  for(int x = 0; x < 4000; ++x){
    sixDOF.getEuler(angles);
    angleOffset += angles[2];
  }
  angleOffset = angleOffset/4000.0f;
}

float calculateSpeed(){
  return (speedSum * range) + md;
}


