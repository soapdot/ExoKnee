
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

// I2C addr: AD0 low = 0x68 (default) | AD0 high = 0x69
MPU6050 IMU_L; 
MPU6050 IMU_RT(0x69);         //MPU6050 IMU_L(0x69); // <-- use for AD0 high
MPU6050 IMU_RC(0x68, &Wire); //MPU6050 IMU_L(0x68, &Wire1); // <-- use for AD0 low, but 2nd Wire (TWI/I2C) object

int16_t IMU_L_ax, IMU_L_ay, IMU_L_az;
int16_t IMU_L_gx, IMU_L_gy, IMU_L_gz;

int16_t IMU_RT_ax, IMU_RT_ay, IMU_RT_az;
int16_t IMU_RT_gx, IMU_RT_gy, IMU_RT_gz;

int16_t IMU_RC_ax, IMU_RC_ay, IMU_RC_az;
int16_t IMU_RC_gx, IMU_RC_gy, IMU_RC_gz;

// EMG: Analog Pins
int EMG_TOP_VAL;
int EMG_TOP_PIN = A1;

int EMG_BOT_VAL;
int EMG_BOT_PIN = A2;

void setup() {
  Serial.begin(9600); // 2400 FOR IMU + 9600 FOR EMG 
  // configure Arduino LED pin for output
  pinMode(LED_PIN, OUTPUT);
  setupIMUL();
  setupIMUR();
  setupEMGT();
  setupEMGB();
  delay(5000); //delay 5s
  Serial.println(IMU_L.testConnection() ? "MPU6050 L connection successful" : "MPU6050 L connection failed");
  Serial.println(IMU_RC.testConnection() ? "MPU6050 RC connection successful" : "MPU6050 RC connection failed");
  delay(5000); //delay 5s

}

void loop() {
  loopIMUL();
  delay(5000); //delay 5s
  loopIMUR();
  delay(5000); //delay 5s
  loopEMGT();
  loopEMGB();

}
