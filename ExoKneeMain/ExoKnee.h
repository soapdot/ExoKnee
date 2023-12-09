/* ==============================================
ExoSuit Project Main: 
5 sensors: Button_LF, Button_RF, IMU_RT, IMU_RC
Abbreviations: R/L = Right/Left, T/C/F = Thigh/Calf/Foot
=================================================
IMU (inertial measurement unit) Sensor for RT/RC
R wired to: I2C pins "SDA/SCL" on Arduino MEGA
=================================================
For IMUs; x, y defined as:
. x: 
. y:  
=================================================
Button_LF: under left foot, wired to 
Button_RF: under right foot, wired to
=================================================*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

//#define LED_PIN 13
#define Button_LF_PIN A0
#define Button_RF_PIN A1
#define INT_PIN 53
#define EStop_PIN 3
#define EN_PIN 6
#define R_PWM_PIN 4
#define L_PWM_PIN 5

int EStop = 0;                        //updated by estop button
int speed = 0;                        //used by LA, speed 0-255 
int percentExtend = 0;  //not actually %, but an int out of 2000 (time for max extend/retract at max speed)
bool standing = true, sitting = false, moving = false; //standing is default
bool RStance = true, LStance = true;                   //standing = stanced both legs

// IMU Sensors: I2C, 3.3V, GND; DI interrupt, DI AD0
MPU6050 IMU_RT(0x68), IMU_RC(0x69); // I2C addr: AD0 low = 0x68 (default) | AD0 high = 0x69

int16_t IMU_RT_ax, IMU_RT_ay, IMU_RT_az; //a = accelerometer value
int16_t IMU_RC_ax, IMU_RC_ay, IMU_RC_az; //xyz = directions defined at top of file

// Foot Buttons
int Button_LF, Button_RF;

// Motor Controller/Linear Actuator used for clarity to reader
int EN = EN_PIN, R_PWM = R_PWM_PIN, L_PWM = L_PWM_PIN;

//setup/loops
// the setup routine runs once when you press reset:
void setupButton() {
  // initialize serial communication at 9600 bits per second:
  Button_LF = 0;
  Button_RF = 0;
  Serial.println("Initializing Step Switches");
  pinMode(Button_LF_PIN, INPUT);
  pinMode(Button_RF_PIN, INPUT);
}

void setupIMU() {
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
  Serial.print("RT Accel Offset: ");
  Serial.print(IMU_RT.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(IMU_RT.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(IMU_RT.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print("\n");
  
  Serial.println("Updating internal RC sensor offsets...");
  // -76	-2359	1688	0	0	0
  Serial.print("RC Accel Offset: ");
  Serial.print(IMU_RC.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(IMU_RC.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(IMU_RC.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print("\n");
}


// the loop routine runs over and over again forever:
void loopButton(int printx) {
  Button_LF = digitalRead(Button_LF_PIN);
  Button_RF = digitalRead(Button_RF_PIN);
  if (Button_RF == 1) {
    Button_RF = 0;
  }
  else {
    Button_RF = 1;
  }

  if (printx == 1) {
    Serial.print("LF / RF:\t");
    Serial.print(Button_LF); Serial.print(" /t "); Serial.println(Button_RF);
  }
}

void loopIMU(int printx) {
  // read raw accel measurements from device
  IMU_RT.getAcceleration(&IMU_RT_ax, &IMU_RT_ay, &IMU_RT_az);
  IMU_RC.getAcceleration(&IMU_RC_ax, &IMU_RC_ay, &IMU_RC_az);
  if (printx == 1) {
    Serial.print("RT x / y:\t");
    Serial.print(IMU_RT_ax/1000); Serial.print(" / "); Serial.println(IMU_RT_ay/1000); 
    Serial.print("RC x / y:\t");
    Serial.print(IMU_RC_ax/1000); Serial.print(" / "); Serial.println(IMU_RC_ay/1000);
  }
}

//input processing functions
void EStopCheck() {
  EStop = analogRead(EStop_PIN);
  if (EStop > 900) {
    Serial.println("EStop: Emergency Stop Engaged!");
    while (1) {
      Serial.println("EStop: In endless while loop"); //this stops code from doing anything else
      delay(3000); //delay 3s before writing again
    }
  }
}

void assistLEDs(int BUTTON_PIN) {
  digitalWrite(BUTTON_PIN, 1);
  delay(3000);
  digitalWrite(BUTTON_PIN, 0);
}

int KneeAngle() {
  loopIMU(0);
  // stright T: 16 0 
  // straight C: 16 0
  // sitting T: 0 16
  // start stand T: 11 11 
  // raise leg C: -11 11 (check)
}

bool SitCheck() { // IMU_RT_ay, IMU_RT_ax, sitting, standing
  if (IMU_RT_ax/1000 > 13) { // thigh is parallel to ground (a)
    Serial.println("SitCheck Results: Yes Sit");
    sitting = true; standing = false;
  }
  else {
    Serial.println("SitCheck Results: Not Sit");
    sitting = false; standing = true;
    RStance = true; LStance = true;
  }
  return sitting;
}

bool StandUpCheck() { 
  if (IMU_RT_ay/1000 > 12) {
    standing = true; sitting = false; 
    LStance = true; RStance = true;
    Serial.println("StandUpCheck: Finished Stand");
  }
  else if (IMU_RT_ay/1000 > 10) {
    Serial.println("StandUpCheck: Starting Stand");
    Serial.println("StandUpCheck: Starting Extend Assistance L/R (0.3s)");
    while ((IMU_RT_ay/1000 > 12) == false) {
      loopIMU(0);
      if (IMU_RT_ay/1000 > 12) {
        Serial.println("StandUpCheck: Finished Stand");
      }
    }
    //TODO: assist L + R (bend)
    //assistLEDs(Button_LF_PIN);
    //assistLEDs(Button_RF_PIN);
  }
  else { //not starting stand 
    standing = false; sitting = true; 
    Serial.println("StandUpCheck: Sitting");
  }
  return standing;
}

bool SwingStanceCheck() {
  //EStopCheck();
  loopButton(0); //update 
  if (standing == true) { //rstance = true, moving = false; check if starting movement
    if ((Button_RF == 1) && (Button_LF == 0)) {  // starting movement with RStance, LSwing 
      Serial.println("SwingStanceCheck: Starting Moving, RStance/LSwing");
      moving = true; standing = false;
      LStance = false; RStance = true; 
      //TODO: start assistance L
      Serial.println("Starting Bend Assistance L (0.3s)");
      //assistLEDs(Button_LF_PIN);
    }
    else if ((Button_RF == 0) && (Button_LF == 1)) { // starting movement with RSwing, LStance
      Serial.println("SwingStanceCheck: Starting Moving, LStance/RSwing");
      moving = true; standing = false;
      LStance = true; RStance = false; 
      //TODO: start assistance R
      Serial.println("Starting Bend Assistance R (0.3s)");
      //assistLEDs(Button_RF_PIN);
    }
    else { //still standing
      Serial.println("SwingStanceCheck: Standing");
      moving = false; standing = true;
      LStance = true; RStance = true; 
    }
    return LStance, RStance, moving;
  }
  else if (moving == true) { // in movement, check if stopping or update phase of movement
    if (RStance == true) { //standing on right leg at last check
      if ((Button_LF == 1) && (Button_RF == 0)) { //if no longer standing on right leg, right in swing 
        Serial.println("SwingStanceCheck: Change movement, RSwing/LStance");
        LStance = true; RStance = false; 
        //TODO: start assistance R
        Serial.println("Starting Bend Assistance R (0.3s)");
        //assistLEDs(Button_RF_PIN);
      }
      else if ((Button_LF == 0) && (Button_RF == 1)) { //still standing on right leg, left in swing
        Serial.println("SwingStanceCheck: Continue movement, RStance/LSwing");
        //everything should be/stay at expected values
      }
      else if ((Button_LF == 1) && (Button_RF == 1)) { // neither in swing; standing
        Serial.println("SwingStanceCheck: Stop movement, stand");
        moving = false; standing = true;
        LStance = true; RStance = true;
      }
      return LStance, RStance, moving;
    }
    else if (LStance == true) { //standing on left leg at last check
      if ((Button_LF == 0) && (Button_RF == 1)) { //if no longer standing on left leg, left in swing 
        Serial.println("SwingStanceCheck: Change movement, RStance/LSwing");
        LStance = false; RStance = true;
        //TODO: start assistance L
        Serial.println("Starting Bend Assistance L (0.3s)");
        //assistLEDs(Button_LF_PIN);
      }
      else if ((Button_LF == 1) && (Button_RF == 0)) { //still standing on left leg, right in swing
        Serial.println("SwingStanceCheck: Continue movement, RSwing/LStance");
        //everything should be/stay at expected values
      }
      else if ((Button_LF == 1) && (Button_RF == 1)) { // neither in swing; standing
        Serial.println("SwingStanceCheck: Stop movement, stand");
        moving = false; standing = true;
        LStance = true; RStance = true;
      }
      return LStance, RStance, moving;
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
