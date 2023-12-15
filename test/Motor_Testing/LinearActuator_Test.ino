/* ============================================
ExoSuit Project Basics: 
Testing Linear Actuator functions:
extend, retract, extendVar, retractVar, stop
Wired to: 
. Motor Controller (Gnd, PWM/Dig input pins 4-7)
  . Motor Controller wired to +12V/gnd & motor
. Button (gnd, 5V, DI9, pulldown resistor 1kohm)
===============================================*/
const int EN_PIN = 3;
const int R_PWM_PIN = 5;
const int L_PWM_PIN = 6;
const uint8_t EN = 3;
const uint8_t R_PWM = 5;
const uint8_t L_PWM = 6;
int speed = 0;
char StopLA;
int percentExtend = 0;

int LABoundsCheck(int speedI, int delayTime, int ExOrRe) {
  int perExtend1 = 0; //don't want to update perExtend before
  if (ExOrRe == 0) { //extension bound check
    perExtend1 = percentExtend + delayTime*speedI/100;
    if (perExtend1 > 2000) { 
      delayTime = (2000-percentExtend)/speedI*100; //delayTime = time itd take to retract fully
      percentExtend = 0; //min extend
    }
    else { //if not, continue as normal
      percentExtend = percentExtend - delayTime*speedI/100;
    }
  }
  else if (ExOrRe == 1) { //retraction bound check 
    perExtend1 = percentExtend - delayTime*speedI/100;
    if (perExtend1 < 0) { 
      delayTime = (2000-percentExtend)*100/speedI; //delayTime = time itd take to retract fully
      percentExtend = 0; //min extend
    }
    else { //if not, continue as normal
      percentExtend = percentExtend - delayTime*speedI/100;
    }
  }
  return delayTime;
}

void extendLA() {
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 255);
  delay(2000 - percentExtend); 
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
  percentExtend = 2000;
}

void retractLA() {
  analogWrite(R_PWM, 255);
  analogWrite(L_PWM, 0);
  delay(percentExtend); //full retract
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
  percentExtend = 0;
}

void extendVarLA(int speedI, int delayTime) {
  if ((speedI>100)||(speedI<0)) {
    Serial.println("ExtendVarLA: Input speed out of bounds. Int [0-100]");
    return;
  }
  delayTime = LABoundsCheck(speedI, delayTime, 0); //will overwrite delayTime with time it takes at speedI to fully extend if past limits. does nothing otherwise.
  speedI = speedI*255/100; //convert to actual speed
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, speedI);
  delay(delayTime); 
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
}

void retractVarLA(int speedI, int delayTime) {
  if ((speedI>100)||(speedI<0)) {
    Serial.println("RetractVarLA: Input speed out of bounds. Int [0-100]");
    return;
  }
  digitalWrite(EN_PIN, 1);
  delayTime = LABoundsCheck(speedI, delayTime, 1); //will overwrite delayTime with time it takes at speedI to fully retract if past limits. does nothing otherwise.
  speedI = speedI*255/100; //convert to actual speed
  analogWrite(R_PWM, speedI);
  analogWrite(L_PWM, 0);
  delay(delayTime);
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
}

void stopLA() {
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
  //digitalWrite(EN, 0); //to disable
}

void setupLA() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT); 
  pinMode(L_PWM_PIN, OUTPUT);
}

void loopLA() {
  // actuator working test
  // turn on actuator (full speed) for 2s
  extendLA();
  // retract actuator (half speed 127 = 255/2) for 4s
  retractVarLA(50, 4000); 
  Serial.println("Do you want to quit LA motion? (Y/N)");
  while (Serial.available() == 0) { //wait for user input
  }
  StopLA = Serial.read();
  if (StopLA == 'Y') {
    stopLA();
  }
}