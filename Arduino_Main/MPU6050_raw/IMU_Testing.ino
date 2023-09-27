/* ============================================
ExoSuit Project Basics:
Run code and type 1 (& enter) for MPU6050_raw()
Run code and type 2 (& enter) for SitCheck()
=============================================== 
MPU6050_raw(): 
Trying individual MPU6050 IMU Sensor 
Only have one plugged into the I2C connection
This will be called "accelgyro" 
===============================================
SitCheck():
Trying SitCheck with one MPU6050 IMU Sensor 
Only have one plugged into the I2C connection
This will be the Right Thigh IMU "IMU_RT"
x, y, z defined as:
. x forward (pointing past knee), 
. y up (pointing toward body of user), 
. z away from leg (toward observer)
when looking at the leg from the side view 
===============================================*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define OUTPUT_READABLE_IMU_RT
#define OUTPUT_READABLE_ACCELGYRO

bool standing = true, sitting = false, moving = false; //standing is default state

MPU6050 IMU_RT(0x69);     //MPU6050 IMU_L(0x69); // <-- use for AD0 high

int16_t IMU_RT_ax, IMU_RT_ay, IMU_RT_az;
int16_t IMU_RT_gx, IMU_RT_gy, IMU_RT_gz;

int CodeSelector;

void setup() {
  Serial.begin(38400);
  Serial.print("Do you want to run MPU6050_raw (1) or SitCheck (2)?");
  //wait for user input
  while (Serial.available() == 0) {
  }
  //get user input into CodeSelecter
  CodeSelector = Serial.parseInt();
  if (CodeSelector == 1) {      //single imu test
    setupIMU(); 
  }
  else if (CodeSelector == 2) { //SitCheck
    setupSC(); 
  }
}

void loop() {
  if (CodeSelector == 1) {      //single imu test
    loopIMU(); 
  }
  else if (CodeSelector == 2) { //SitCheck
    loopSC1(); 
  }
}

