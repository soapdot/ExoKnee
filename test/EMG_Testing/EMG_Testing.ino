/* ============================================
ExoSuit Project Basics:
Run code and type 1 (& enter) for EMG_analog_read()
Run code and type 2 (& enter) for ReadEMG()
Run code and type 3 (& enter) for StandUpCheck()
For all, only need 1 EMG in A2 slot
=============================================== 
EMG_analog_read(): 
Trying individual EMG Sensor 
This code converts raw analog input to voltage (0-1000)
and prints graphical representation via serial plotter
===============================================
ReadEMG():
Taking EMG_analog_read, which gives a number
Code determines whether MuscleUse is 0 or 1
If over 500, high (1); else, low (0)
===============================================
StandUpCheck():
Taking ReadEMG(), which updates MuscleUse
Assuming user is currently sitting
Checks for signs of user standing (EMG_RC -> MuscleUse = 1)
===============================================*/

int CodeSelector;

void setup() {
  Serial.begin(38400);
  Serial.print("Do you want to run EMG_analog_read (1) or ReadEMG (2) or StandUpCheck (3)?");
  //wait for user input
  while (Serial.available() == 0) {
  }
  //get user input into CodeSelecter
  CodeSelector = Serial.parseInt();
  if (CodeSelector == 1) {      //EMG_analog_read
    setupEMG(); 
  }
  else if (CodeSelector == 2) { //ReadEMG
    setupEMGRead(); 
  }
  else if (CodeSelector == 3) { //StandUpCheck
    setupSUC(); 
  }
}

void loop() {
  if (CodeSelector == 1) {      //EMG_analog_read
    loopEMG(); 
  }
  else if (CodeSelector == 2) { //ReadEMG
    loopEMGRead(); 
  }
  else if (CodeSelector == 3) { //StandUpCheck
    setupSUC(); 
  }
}
