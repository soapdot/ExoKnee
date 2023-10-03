/* ==============================================
ExoSuit Project Main: 
=================================================
For EMGs; 
Value is between 0-1000 where >500 is high
For now, only counting MuscleUse as a bool
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

// uncomment "OUTPUT_READABLE_IMU_L" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_IMU_RT
#define OUTPUT_READABLE_IMU_RC
#define OUTPUT_READABLE_IMU_LC

#define LED_PIN 13

void setup() {
  Serial.begin(9600); // 2400 FOR IMU + 9600 FOR EMG (IMU/EMG Rec: 115200)
  // configure inputs/outputs
  pinMode(EStop_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT);
  setupIMU();
  setupEMG();
  delay(5000); //delay 5s
  Serial.println(IMU_LC.testConnection() ? "MPU6050 LC connection successful" : "MPU6050 LC connection failed");
  Serial.println(IMU_RC.testConnection() ? "MPU6050 RC connection successful" : "MPU6050 RC connection failed");
  delay(5000); //delay 5s

}

void loop() {
  //check estop button for input
  EStopCheck();
  //update vals
  loopIMU();
  loopEMG();
  //val from loopEMG updates MuscleUse val 
  MuscleUseRT = ReadEMG(EMG_RT_VAL);
  MuscleUseRC = ReadEMG(EMG_RC_VAL);
  //first check for sitting motion; if sitting, don't need to run other movement functions for now
  sitting = SitCheck(); //EMG_RC_VAL, MuscleUseRC, IMU_RT_ay, IMU_RT_ax
  while ((sitting == true)&&(standing == false)) { //while sitting, until standing:
    //output to c++ : sit mode
    EStopCheck();
    loopEMG(); //update val
    standing = StandUpCheck(); //stay in StandUpCheck function (won't be doing other movement func during sit)
    if (standing == false) {
      delay(500); //wait 0.5s before checking again
    }
  }
  //how to get moving = true initially? 
  while (moving == true) { //continuing from last check
    EStopCheck();
    RStance, LStance, moving = SwingStanceCheck();
  }
}
