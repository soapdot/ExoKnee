/* ============================================
ExoSuit Project Basics: 
Trying StandUpCheck with one EMG Sensor 
Test ReadEMG first; this builds on that
This function keeps code in loop until user is standing
EMG sensor will be Right Calf sensor "EMG_RC" in A2
===============================================*/

bool StandUpCheck() { //stay in function until standing MuscleUseRC, moving, standing, sitting
  if (sitting == true) {
    moving = false;
    Serial.println("StandUpCheck: Sitting");
    MuscleUseRC = ReadEMG(EMG_RC_VAL); 
    if (MuscleUseRC == 1) { //user activating calf, indicates starting to stand 
      Serial.println("StandUpCheck: Starting Stand, 2s til next results");
      standing = true;
      delay(2000); //delay 2s while user is in process of standing
      Serial.println("StandUpCheck: Standing now (theoretically)");
      sitting = false; //escape loop
    }
  }
  Serial.println("StandUpCheck: Escaped sitting loop");
  return standing; 
}
//void setup() {//uncomment this to test as main func
void setupSUC() {//comment this to test as main func
  Serial.begin(9600); // 2400 FOR IMU + 9600 FOR EMG (IMU/EMG Rec: 115200)
  EMG_RC_VAL = 0;
  Serial.println("Initializing A2 Device [EMG_RC]");
  pinMode(EMG_RC_PIN, INPUT);
}

//void loop() {//uncomment this to test as main func
void loopSUC() {//comment this to test as main func
  //update val
  EMG_RC_VAL = analogRead(EMG_RC_PIN);
  Serial.println(EMG_RC_VAL);
  //logic
  MuscleUseRC = ReadEMG(EMG_RC_VAL);
  while ((sitting == true)&&(standing == false)) { //while sitting, until standing:
    //update val that lets us know when standing
    EMG_RC_VAL = analogRead(EMG_RC_PIN);
    Serial.println(EMG_RC_VAL);
    //call standupcheck
    standing = StandUpCheck(); 
    if (standing == false) {
      Serial.println("Loop: delaying 0.5s before check");
      delay(500); //wait 0.5s before checking again
    }
  }
  Serial.println("Loop: Escaped main's sitting loop");

}