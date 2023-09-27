/* ============================================
ExoSuit Project Basics: 
Testing Linear Actuator functions:
extend, retract, extendVar, retractVar, stop
Wired to: 
. Motor Controller (Gnd, PWM/Dig input pins 4-7)
  . Motor Controller wired to +12V/gnd & motor
. Button (gnd, 5V, DI9, pulldown resistor 1kohm)
===============================================*/

const uint8_t EN = 4;
const uint8_t R_PWM = 5;
const uint8_t L_PWM = 6;
int speed = 0;
char StopLA;

void extendLA() {
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 255);
  delay(2000); //2s, enough for LA to fully extend
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
}

void retractLA() {
  analogWrite(R_PWM, 255);
  analogWrite(L_PWM, 0);
  delay(2000); //2s, enough for LA to fully retract
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
}

void extendVarLA(int speedE, int delayTime) {
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, speedE);
  //TODO: change delay to inversely change with speed
  delay(delayTime); 
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
}

void retractVarLA(int speedR, int delayTime) {
  analogWrite(R_PWM, speedR);
  analogWrite(L_PWM, 0);
  //TODO: change delay to inversely change with speed
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
  pinMode(4, OUTPUT); //EN
  pinMode(5, OUTPUT); //R_PWM
  pinMode(6, OUTPUT); //L_PWM
}

void loopLA() {
  // actuator working test
  // turn on actuator (full speed) for 2s
  extendLA();
  // retract actuator (half speed 127 = 255/2) for 4s
  retractVarLA(127, 4000); 
  Serial.print("Do you want to quit LA motion? (Y/N)");
  while (Serial.available() == 0) { //wait for user input
  }
  StopLA = Serial.read();
  if (StopLA == 'Y') {
    stopLA();
  }
}