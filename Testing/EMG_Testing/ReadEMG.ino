/* ============================================
ExoSuit Project Basics: 
Trying ReadEMG with one EMG Sensor 
Updates "MuscleUse" to be 0 or 1 based on low or high muscle usage
EMG sensor will be Right Calf sensor "EMG_RC" in A2
===============================================*/
bool standing = false, sitting = true, moving = false; //assumed user is sitting upon code start
bool RStance = false, LStance = false;
int MuscleUseRC = 0, MuscleUseRT = 0;
int EMG_RC_VAL;
const int EMG_RC_PIN = A2;

int ReadEMG(int EMG_VAL) {
  int MuscleUse;
  if (EMG_VAL > 500) {
    MuscleUse = 1;
  }
  else {
    MuscleUse = 0;
  }
  return MuscleUse;
}
//void setup() {//uncomment this to test as main func
void setupEMGRead() {//comment this to test as main func
  Serial.begin(9600); // 2400 FOR IMU + 9600 FOR EMG (IMU/EMG Rec: 115200)
  EMG_RC_VAL = 0;
  Serial.println("Initializing A2 Device [EMG_RC]");
  pinMode(EMG_RC_PIN, INPUT);
}

//void loop() {//uncomment this to test as main func
void loopEMGRead() {//comment this to test as main func
  //update val
  EMG_RC_VAL = analogRead(EMG_RC_PIN);
  Serial.println(EMG_RC_VAL);
  //logic
  MuscleUseRC = ReadEMG(EMG_RC_VAL);
  Serial.println("Loop: MuscleUseRC Updated");
  Serial.println(MuscleUseRC);
  delay(500);
}