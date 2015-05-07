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

Engine *engine; 
Controller control;
FreeSixIMU sixDOF = FreeSixIMU();

float angles[3]; // yaw pitch roll
float angleOffset = 0.0f;

float error; 
float errorSum = 0.0f;
float speedSum = 0.0f;
float Kp = 2.0f;   
float Ki = 4.0f;   
float Kd = 100.0f;   
float maxVal = 100.0f;
float minVal = 25.0f;
float md = (maxVal + minVal)/2;
float range = md - minVal;
int bufferSize = 1000;

PidControl pidControl(Kp,Ki,Kd, bufferSize);

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing");
  engine = new Engine(2,3,4,5);
  engine->armEngine();
  Serial.println("Done arming");  
  //control.connectController();
  Wire.begin();
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
  calibrateGyro();
  Serial.print("offSet is:    ");
  Serial.println(angleOffset);
  delay(8000);
  Serial.print("MOVE HANDS");
}

void loop() {
  sixDOF.getEuler(angles);
   
  error = (angles[2] - angleOffset)/180.0f;
  
  speedSum = pidControl.calcPID(error); 
  
  if(speedSum > 1.0f) speedSum = 1.0f;
  if(speedSum < -1.0f) speedSum = -1.0f;
  
  //fix
  float actSpeed = calculateSpeed();
  float actSpeedOpp = (md * 2) - actSpeed;
  
    
   engine->engineSW(actSpeed + 0.0f);
   engine->engineSR(actSpeedOpp + 0.0f);
  
  

    //Serial.println("angle: ");
/*
    Serial.print(error);
    Serial.print(";");
//    Serial.print("     Proportional:    ");
    Serial.print(p);
  //  Serial.print("     Integral:    ");
    Serial.print(";");
    Serial.print(i);
//    Serial.print("     Derivative:    ");
    Serial.print(";");
    Serial.print(d);
    Serial.println();
    Serial.print("     SpeedSum:    ");
    Serial.println(speedSum);
    
    Serial.print("     Actual Speed SW:    ");
    Serial.println(actSpeed);
    Serial.println();
    Serial.print("     Actual Speed SR:    ");
    Serial.println(actSpeedOpp);
    Serial.println();
  
*/
}

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


