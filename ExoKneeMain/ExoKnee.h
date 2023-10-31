/* ==============================================
ExoSuit Project Main: 
5 sensors: EMG_LC, EMG_RC, IMU_LC, IMU_RT, IMU_RC
Abbreviations: R/L = Right/Left, T/C = Thigh/Calf
=================================================
IMU (inertial measurement unit) Sensor for RT/RC
R wired to: I2C pins "SDA/SCL" on Arduino MEGA
L wired to: pins above AREF on Arduino MEGA
=================================================
For IMUs; x, y, z defined as:
. x forward (pointing past knee), 
. y up (pointing toward body of user), 
. z away from leg (toward observer)
when looking at the leg from the side view 
=================================================
EMG (Electromyography) Sensor for RC/LC
Wired to: +-9V/1A, A2 (RC) & A1 (LC)
=================================================
For EMGs; 
Value is between 0-1000 where >500 is high
For now, only counting MuscleUse as a bool
May add levels based on average findings (0-3)
=================================================*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

// tab-separated list of the accel X/Y/Z & gyro X/Y/Z values in decimal. Easy to read, not to parse, slow(er) over UART.
#define OUTPUT_READABLE_IMU_RT
#define OUTPUT_READABLE_IMU_RC
#define OUTPUT_READABLE_IMU_LC

#define LED_PIN 13
#define EMG_LC_PIN A1
#define EMG_RC_PIN A2
#define EStop_PIN 9
#define EN_PIN 4
#define R_PWM_PIN 5
#define L_PWM_PIN 6

bool blinkState = false;              //updates LED
int MuscleUseRC = 0, MuscleUseLC = 0; //updated by EMG 
int EStop = 0;                        //updated by estop button
int speed = 0;                        //used by LA, speed 0-255 
int percentExtend = 0;  //not actually %, but an int out of 2000 (time for max extend/retract at max speed)
bool standing = true, sitting = false, moving = false; //standing is default
bool RStance = true, LStance = true;                   //standing = stanced both legs

// IMU Sensors: I2C, 3.3V, GND; DI interrupt, DI AD0
MPU6050 IMU_RT(0x68), IMU_RC(0x69); // I2C addr: AD0 low = 0x68 (default) | AD0 high = 0x69
//MPU6050 IMU_LC(0x68); //MPU6050 IMU_L(0x68, &Wire1); // <-- use for AD0 low, but 2nd Wire (TWI/I2C) object

int16_t IMU_LC_ax, IMU_LC_ay, IMU_LC_az, IMU_LC_gx, IMU_LC_gy, IMU_LC_gz; //a = accelerometer value
int16_t IMU_RT_ax, IMU_RT_ay, IMU_RT_az, IMU_RT_gx, IMU_RT_gy, IMU_RT_gz; //g = gyroscope value
int16_t IMU_RC_ax, IMU_RC_ay, IMU_RC_az, IMU_RC_gx, IMU_RC_gy, IMU_RC_gz; //xyz = directions defined at top of file

// EMG Sensors: 5V, GND; AI input
int EMG_LC_VAL, EMG_RC_VAL; //analog input of muscle sensor 0-1023

// Motor Controller/Linear Actuator used for clarity to reader
int EN = EN_PIN, R_PWM = R_PWM_PIN, L_PWM = L_PWM_PIN;

int ReadEMG(int EMG_VAL) {
  int MuscleUse;
  if (EMG_VAL < 50) {
    MuscleUse = 0;
  }
  else if ((EMG_VAL > 50) && (EMG_VAL < 1000)){ //catching all usage right now; will expand 
    MuscleUse = 1;
  }
  return MuscleUse;
}

//setup/loops
// the setup routine runs once when you press reset:
void setupEMGLC() {
  // initialize serial communication at 9600 bits per second:
  EMG_LC_VAL = 0;
  Serial.println("Initializing A1 Device [EMG_LC]");
  MuscleUseLC = ReadEMG(EMG_LC_VAL);
  pinMode(EMG_LC_PIN, INPUT);
}
void setupEMGRC() {
  // initialize serial communication at 9600 bits per second:
  EMG_RC_VAL = 0;
  Serial.println("Initializing A2 Device [EMG_RC]");
  MuscleUseRC = ReadEMG(EMG_RC_VAL);
  pinMode(EMG_RC_PIN, INPUT);
}
void setupEMG() {
  setupEMGLC();
  setupEMGRC();
} 
/*
void setupIMUL() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize device
  Serial.println("Initializing I2C (LC) device...");
  IMU_LC.initialize();

  // verify connection
  Serial.println("Testing device (LC) connection...");
  Serial.println(IMU_LC.testConnection() ? "MPU6050 LC connection successful" : "MPU6050 LC connection failed");
  
  // use the code below to change accel/gyro offset values

  Serial.println("Updating internal LC sensor offsets...");
  // -76	-2359	1688	0	0	0
  Serial.print(IMU_LC.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(IMU_LC.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(IMU_LC.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(IMU_LC.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(IMU_LC.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(IMU_LC.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  IMU_LC.setXGyroOffset(220);
  IMU_LC.setYGyroOffset(76);
  IMU_LC.setZGyroOffset(-85);
  Serial.print(IMU_LC.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(IMU_LC.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(IMU_LC.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(IMU_LC.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(IMU_LC.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(IMU_LC.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
}*/

void setupIMUR() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize device
  Serial.println("Initializing I2C (R) devices...");
  IMU_RT.initialize();
  IMU_RC.initialize();

  // verify connection
  Serial.println("Testing device (R) connections...");
  Serial.println(IMU_RT.testConnection() ? "MPU6050 RT connection successful" : "MPU6050 RT connection failed");
  Serial.println(IMU_RC.testConnection() ? "MPU6050 RC connection successful" : "MPU6050 RC connection failed");

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

  // use the code below to change accel/gyro offset values

  Serial.println("Updating internal RC sensor offsets...");
  // -76	-2359	1688	0	0	0
  Serial.print(IMU_RC.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(IMU_RC.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(IMU_RC.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(IMU_RC.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(IMU_RC.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(IMU_RC.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  IMU_RC.setXGyroOffset(220);
  IMU_RC.setYGyroOffset(76);
  IMU_RC.setZGyroOffset(-85);
  Serial.print(IMU_RC.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(IMU_RC.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(IMU_RC.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(IMU_RC.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(IMU_RC.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(IMU_RC.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
}
void setupIMU() {
  //setupIMUL();
  setupIMUR();
}

// the loop routine runs over and over again forever:
void loopEMGLC() {
  EMG_LC_VAL = analogRead(EMG_LC_PIN);
  //float voltage = EMG_TOP_VAL * (5.0 / 1023.0); // Analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  MuscleUseLC = ReadEMG(EMG_LC_VAL);
  Serial.println(EMG_LC_VAL);
}
void loopEMGRC() {
  EMG_RC_VAL = analogRead(EMG_RC_PIN);
  //float voltage = EMG_BOT_VAL * (5.0 / 1023.0);
  MuscleUseRC = ReadEMG(EMG_RC_VAL);
  Serial.println(EMG_RC_VAL);
}
void loopEMG() {
  loopEMGLC();
  loopEMGRC();
}
/*
void loopIMUL() {
  // read raw accel/gyro measurements from device
  IMU_LC.getMotion6(&IMU_LC_ax, &IMU_LC_ay, &IMU_LC_az, &IMU_LC_gx, &IMU_LC_gy, &IMU_LC_gz);

  // these methods (and a few others) are also available
  //IMU_L.getAcceleration(&IMU_L_ax, &IMU_L_ay, &IMU_L_az);
  //IMU_L.getRotation(&IMU_L_gx, &IMU_L_gy, &IMU_L_gz);

  #ifdef OUTPUT_READABLE_IMU_LC
    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(IMU_LC_ax/1000); Serial.print("\t");
    Serial.print(IMU_LC_ay/1000); Serial.print("\t");
    Serial.print(IMU_LC_az/1000); Serial.print("\t");
    Serial.print(IMU_LC_gx/100); Serial.print("\t");
    Serial.print(IMU_LC_gy/100); Serial.print("\t");
    Serial.println(IMU_LC_gz/100);
  #endif
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}*/

void loopIMUR() {
  // read raw accel/gyro measurements from device
  IMU_RT.getMotion6(&IMU_RT_ax, &IMU_RT_ay, &IMU_RT_az, &IMU_RT_gx, &IMU_RT_gy, &IMU_RT_gz);
  IMU_RC.getMotion6(&IMU_RC_ax, &IMU_RC_ay, &IMU_RC_az, &IMU_RC_gx, &IMU_RC_gy, &IMU_RC_gz);

  // these methods (and a few others) are also available
  //IMU_RT.getAcceleration(&IMU_L_ax, &IMU_L_ay, &IMU_L_az);
  //IMU_RT.getRotation(&IMU_L_gx, &IMU_L_gy, &IMU_L_gz);

  #ifdef OUTPUT_READABLE_IMU_RT
    // display tab-separated accel/gyro x/y/z values
    Serial.print("RT a/g:\t");
    Serial.print(IMU_RT_ax/1000); Serial.print("\t");
    Serial.print(IMU_RT_ay/1000); Serial.print("\t");
    Serial.print(IMU_RT_az/1000); Serial.print("\t");
    Serial.print(IMU_RT_gx/100); Serial.print("\t");
    Serial.print(IMU_RT_gy/100); Serial.print("\t");
    Serial.println(IMU_RT_gz/100);
  #endif

  #ifdef OUTPUT_READABLE_IMU_RC
    // display tab-separated accel/gyro x/y/z values
    Serial.print("RC a/g:\t");
    Serial.print(IMU_RC_ax/1000); Serial.print("\t");
    Serial.print(IMU_RC_ay/1000); Serial.print("\t");
    Serial.print(IMU_RC_az/1000); Serial.print("\t");
    Serial.print(IMU_RC_gx/100); Serial.print("\t");
    Serial.print(IMU_RC_gy/100); Serial.print("\t");
    Serial.println(IMU_RC_gz/100);
  #endif

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}
void loopIMU() {
  //loopIMUL();
  loopIMUR();
}

//input processing functions

void EStopCheck() {
  EStop = digitalRead(EStop_PIN);
  if (EStop == 1) {
  Serial.println("EStop: Emergency Stop Engaged!");
    while (1) {
      Serial.println("EStop: In endless while loop"); //this stops code from doing anything else
      delay(3000); //delay 3s before writing again
    }
  }
}

bool SitCheck() { //function is hardcoded w these for sitting IMU_RT_ay, IMU_RT_ax, sitting, standing
  // right now the user must move in a straight line forwards only OR sit (backwards may be seen as a sit)
  if (IMU_RT_ax/1000 >= 13) { // thigh is parallel to ground (a)
    Serial.println("SitCheck Results: Yes Sit");
    sitting = true;
    standing = false;
  }
  else {
    Serial.println("SitCheck Results: No Sit");
    sitting = false;
  }
  return sitting;
}

bool StandUpCheck() { 
  while (standing == false) {
    EStopCheck();
    loopIMU();
    if (IMU_RT_ay/1000 > 14) {
      standing = true;  //escape loop
      sitting = false; 
      Serial.println("StandUpCheck: Finished Stand");
    }
    else { //not starting stand 
      Serial.println("StandUpCheck: Sitting");
    }
  }
  return standing;
}

bool SwingStanceCheck() {
  int standCount = 0;
  if (standing == true) { //rstance = true, moving = false; check if starting movement
    delay(300); //delay .3s, chance to start moving
    EStopCheck();
    loopEMGRC(); //update emg/muscleuse val
    if ((MuscleUseRC == 1) && (MuscleUseLC == 0)) {  // starting movement with RStance, LSwing 
      Serial.println("SwingStanceCheck: Starting Moving, RStance/LSwing");
      moving = true;
      standing = false;
      RStance = true;
      LStance = false;
      return RStance, LStance, moving;
    }
    else if ((MuscleUseRC == 0) && (MuscleUseLC == 0)) { // starting movement with RSwing, LStance
      Serial.println("SwingStanceCheck: Starting Moving, LStance/RSwing");
      moving = true;
      standing = false;
      RStance = false;
      LStance = true;
      return RStance, LStance, moving;
    }
    else { //still standing
      Serial.println("SwingStanceCheck: Standing");
      return RStance, LStance, moving;
    }
  }
  else if (moving == true) { // in movement, check if stopping or update phase of movement
    loopEMG(); //update emgs & muscleuse vals
    if (RStance == true) { //standing on right leg at last check
      if ((MuscleUseRC == 0) && (MuscleUseLC == 1)) { //if no longer standing on right leg, right in swing 
        RStance = false;
        LStance = true;
        moving = true;
        return RStance, LStance, moving;
      }
      else if ((MuscleUseRC == 1) && (MuscleUseLC == 0)) { //still standing on right leg, left in swing
        return RStance, LStance, moving;
      }
      else { // neither in swing; standing
        moving = false;
        LStance = true;
        standing = true;
        return RStance, LStance, moving;
      }
    }
    else if (LStance == true) { //standing on left leg at last check
      if ((MuscleUseLC == 0) && (MuscleUseRC == 1)) { //if no longer standing on left leg, left in swing 
        RStance = true;
        LStance = false;
        moving = true;
        return RStance, LStance, moving;
      }
      else if ((MuscleUseLC == 1) && (MuscleUseRC == 0)) { //still standing on left leg, right in swing
        return RStance, LStance, moving;
      }
      else { // neither in swing; standing
        moving = false;
        LStance = true;
        standing = true;
        return RStance, LStance, moving;
      }
    }
  }
}

//output functions (LA = linear actuator)
int LABoundsCheck(int speedI, int delayTime, int ExOrRe) {
  int perExtend1 = 0; //don't want to update perExtend before
  if (ExOrRe == 0) { //extension bound check
    perExtend1 = percentExtend + delayTime * speedI / 100;
    if (perExtend1 > 2000) {
      delayTime = (2000 - percentExtend) * 100 / speedI; //delayTime = time itd take to extend fully
      percentExtend = 2000; //max extend
    }
    else { //if not, continue as normal
      percentExtend = percentExtend - delayTime * speedI / 100;
    }
  }
  else if (ExOrRe == 1) { //retraction bound check 
    perExtend1 = percentExtend - delayTime * speedI / 100;
    if (perExtend1 < 0) {
      delayTime = (percentExtend) * 100 / speedI; //delayTime = time itd take to retract fully
      percentExtend = 0; //min extend
    }
    else { //if not, continue as normal
      percentExtend = percentExtend - delayTime * speedI / 100;
    }
  }
  return delayTime;
}

void extendLA() {
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 255);
  int fullDelay = (2000-percentExtend);
  for (int intvl = (fullDelay)/10; intvl < fullDelay; intvl += (fullDelay)/10) {
    EStopCheck();
    delay(intvl);
  }
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
  percentExtend = 2000;
}

void retractLA() {
  analogWrite(R_PWM, 255);
  analogWrite(L_PWM, 0);
  int fullDelay = percentExtend;
  for (int intvl = fullDelay/10; intvl < fullDelay; intvl += fullDelay/10) { //checks every (max 2s/10 = .2s)
    EStopCheck();
    delay(intvl);
  }
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
  percentExtend = 0;
}

void extendVarLA(int speedI, int delayTime) {
  if ((speedI > 100) || (speedI < 0)) {
    Serial.println("ExtendVarLA: Input speed out of bounds. Int [0-100]");
    return;
  }
  delayTime = LABoundsCheck(speedI, delayTime, 0); //will overwrite delayTime with time it takes at speedI to fully extend if past limits. does nothing otherwise.
  speedI = speedI * 255 / 100; //convert to actual speed
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, speedI);
  for (int intvl = (delayTime)/10; intvl < delayTime; intvl+=(delayTime)/10) { //checks every (max 2s/10 = .2s)
    EStopCheck();
    delay(intvl);
  }
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
}

void retractVarLA(int speedI, int delayTime) {
  if ((speedI > 100) || (speedI < 0)) {
    Serial.println("RetractVarLA: Input speed out of bounds. Int [0-100]");
    return;
  }
  delayTime = LABoundsCheck(speedI, delayTime, 1); //will overwrite delayTime with time it takes at speedI to fully retract if past limits. does nothing otherwise.
  speedI = speedI * 255 / 100; //convert to actual speed
  analogWrite(R_PWM, speedI);
  analogWrite(L_PWM, 0);
  for (int intvl = (delayTime)/10; intvl < delayTime; intvl+=(delayTime)/10) { //checks every (max 2s/10 = .2s)
    EStopCheck();
    delay(intvl);
  }
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
}

void stopLA() {
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
  //digitalWrite(EN, 0); //to disable
}
