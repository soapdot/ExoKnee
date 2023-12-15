/* ==============================================
ExoSuit Project Main: 
=================================================
For EMGs; 
Value is between 0-1000 where >500 is high
MuscleUse is on scale of 0-3: 0 = x<50, 1 = 50<x<400 (calf power to stand), 2 = 400<x<1000 (high flex)
May add levels based on average findings (0-3)
=================================================
For IMUs; x, y, z defined as:
. x forward (pointing past knee), 
. y up (pointing toward body of user), 
. z away from leg (toward observer)
when looking at the leg from the side view 
=================================================*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "ExoKnee.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

void setup() {
  Serial.begin(2400); // 2400 FOR IMU & 9600 FOR EMG (IMU/EMG Rec: 115200)
  // configure inputs/outputs
  pinMode(EStop_PIN, INPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT);
  setupIMU();
  setupButton();
}

void loop() {
  //check estop button for input
  EStopCheck();
  //update vals
  //Serial.println("FirstLoop");
  loopIMU(0);
  loopButton(0);
  //first check for sitting motion; if sitting, don't need to run other movement functions for now
  
  sitting = SitCheck(); //IMU_RT_ax, IMU_RT_ay
  while (sitting == true) { //while sitting, until standing:
    //output to c++ : sit mode
    EStopCheck();
    loopIMU(0);
    standing = StandUpCheck(); //stay in StandUpCheck function (won't be doing other movement func during sit)
  }
  EStopCheck();
  loopButton(0);
  RStance, LStance, moving = SwingStanceCheck();
  while (moving == true) { //continuing from last check
    EStopCheck();
    loopButton(0);
    RStance, LStance, moving = SwingStanceCheck();
  }
}
