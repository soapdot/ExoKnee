/* ============================================
ExoSuit Project Basics: 
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

bool SitCheck() { //function is hardcoded w these for sitting IMU_RT_ay, IMU_RT_ax, sitting, standing
  // right now the user must move in a straight line forwards only OR sit (backwards may be seen as a sit)
  if ((IMU_RT_ay < 0)&&(IMU_RT_ax < 0)) { // moving/rotating backwards & down (a or g?)
    Serial.println("SitCheck Results: Yes Sit");
    sitting = true;
    standing = false;
  }
  else {
    Serial.println("SitCheck Results: No Sit");
    sitting = false;
    //moving and standing decided by StanceCheck
  }
  return sitting;
}

void setupSC() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize device
  Serial.println("Initializing I2C (RT) device...");
  IMU_RT.initialize();

  // verify connection
  Serial.println("Testing device (RT) connection...");
  Serial.println(IMU_RT.testConnection() ? "MPU6050 RT connection successful" : "MPU6050 RT connection failed");
  // use the code below to change accel/gyro offset values
  
  Serial.println("Updating internal RT sensor offsets...");
  // -76	-2359	1688	0	0	0
  Serial.print(IMU_RT.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(IMU_RT.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(IMU_RT.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(IMU_RT.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(IMU_RT.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(IMU_RT.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  IMU_RT.setXGyroOffset(220);
  IMU_RT.setYGyroOffset(76);
  IMU_RT.setZGyroOffset(-85);
  Serial.print(IMU_RT.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(IMU_RT.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(IMU_RT.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(IMU_RT.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(IMU_RT.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(IMU_RT.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
}

void loopSC0() {
  // read raw accel/gyro measurements from device
  IMU_RT.getMotion6(&IMU_RT_ax, &IMU_RT_ay, &IMU_RT_az, &IMU_RT_gx, &IMU_RT_gy, &IMU_RT_gz);

  // these methods (and a few others) are also available
  //IMU_RT.getAcceleration(&IMU_L_ax, &IMU_L_ay, &IMU_L_az);
  //IMU_RT.getRotation(&IMU_L_gx, &IMU_L_gy, &IMU_L_gz);

  #ifdef OUTPUT_READABLE_IMU_RT
      // display tab-separated accel/gyro x/y/z values
      Serial.print("a/g:\t");
      Serial.print(IMU_RT_ax); Serial.print("\t");
      Serial.print(IMU_RT_ay); Serial.print("\t");
      Serial.print(IMU_RT_az); Serial.print("\t");
      Serial.print(IMU_RT_gx); Serial.print("\t");
      Serial.print(IMU_RT_gy); Serial.print("\t");
      Serial.println(IMU_RT_gz);
  #endif

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

void loopSC1() {
  loopSC0(); //update values
  sitting = SitCheck(); //check if sitting
  delay(500); //delay 0.5s before rechecking
}
