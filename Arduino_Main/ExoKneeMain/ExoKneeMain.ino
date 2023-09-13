
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

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
#define OUTPUT_READABLE_IMU_L

#define LED_PIN 13
bool blinkState = false;
bool standing = true, sitting = false, moving = false; //standing is default state
//bool currStand = true, currSit = false, currMove = false; 
int MuscleUseRC = 0, MuscleUseRT = 0;

// I2C addr: AD0 low = 0x68 (default) | AD0 high = 0x69
MPU6050 IMU_L(0x68); 
MPU6050 IMU_RT(0x69);         //MPU6050 IMU_L(0x69); // <-- use for AD0 high
MPU6050 IMU_RC(0x68, &Wire);  //MPU6050 IMU_L(0x68, &Wire1); // <-- use for AD0 low, but 2nd Wire (TWI/I2C) object

int16_t IMU_L_ax, IMU_L_ay, IMU_L_az;
int16_t IMU_L_gx, IMU_L_gy, IMU_L_gz;

int16_t IMU_RT_ax, IMU_RT_ay, IMU_RT_az;
int16_t IMU_RT_gx, IMU_RT_gy, IMU_RT_gz;

int16_t IMU_RC_ax, IMU_RC_ay, IMU_RC_az;
int16_t IMU_RC_gx, IMU_RC_gy, IMU_RC_gz;

// EMG: Analog Pins
int EMG_RT_VAL;
int EMG_RC_VAL;
const int EMG_RT_PIN = A1;
const int EMG_RC_PIN = A2;

int readEMG(int EMG_VAL) {
  int MuscleUse;
  if (EMG_VAL > 500) {
    MuscleUse = 1;
  }
  else {
    MuscleUse = 0;
  }
  return MuscleUse;
}

bool SitCheck() { //function is hardcoded w these for sitting IMU_RT_ay, IMU_RT_ax, sitting, standing
  // right now the user must move in a straight line forwards only OR sit (backwards may be seen as a sit)
  if ((IMU_RT_ay < 0)&&(IMU_RT_ax < 0)) { // moving/rotating backwards & down (a or g?)
    sitting = true;
    standing = false;
  }
  else {
    sitting = false;
    //moving and standing decided by StanceCheck
  }
  return sitting;
}

bool StandUpCheck() { //stay in function until standing MuscleUseRC, moving, standing, sitting
  while (sitting == true) {
    moving = false;
    //MuscleUseRC = readEMG(EMG_RC_VAL); //given by funct header
    if (MuscleUseRC == 1) { //user activating calf, indicates starting to stand 
      standing = true;
      delay(2000); //delay 2s while user is in process of standing
      sitting = false; //escape loop
    }
  }
  return standing;
}

void setup() {
  Serial.begin(9600); // 2400 FOR IMU + 9600 FOR EMG (IMU/EMG Rec: 115200)
  // configure Arduino LED pin for output
  pinMode(LED_PIN, OUTPUT);
  setupIMUL();
  setupIMUR();
  setupEMGRT();
  setupEMGRC();
  delay(5000); //delay 5s
  Serial.println(IMU_L.testConnection() ? "MPU6050 L connection successful" : "MPU6050 L connection failed");
  Serial.println(IMU_RC.testConnection() ? "MPU6050 RC connection successful" : "MPU6050 RC connection failed");
  delay(5000); //delay 5s

}

void loop() {
  //update vals
  loopIMUL();
  loopIMUR();
  loopEMGRT();
  loopEMGRC();

  //update interpretations
  MuscleUseRT = readEMG(EMG_RT_VAL);
  MuscleUseRC = readEMG(EMG_RC_VAL);
  //first check for sitting motion; if sitting, don't need to run other movement functions for now
  sitting = SitCheck(); //EMG_RC_VAL, MuscleUseRC, IMU_RT_ay, IMU_RT_ax
  while ((sitting == true)&&(standing == false)) { //while sitting, until standing:
    //output to c++ : sit mode
    standing = StandUpCheck(); //stay in StandUpCheck function (won't be doing other movement func during sit)
  }

}
