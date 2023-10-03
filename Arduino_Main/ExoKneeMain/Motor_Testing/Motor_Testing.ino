/* ============================================
ExoSuit Project Basics:
Run code and type 1 (& enter) for LinearActuator_Test()
Run code and type 2 (& enter) for EStop()
=============================================== 
EStop():
Testing EStop with 1 linear actuator
Extends LA to max at 1/4 speed until EStop
Will switch to retract after 7.5s if no user input
Wired to: 
. Motor Controller (Gnd, PWM/Dig input pins 4-7)
  . Motor Controller wired to +12V/gnd & motor
. Button (gnd, 5V, DI9, pulldown resistor 1kohm)
=============================================== 
LinearActuator_Test(): 
Testing Linear Actuator functions
extend, retract, extendVar, retractVar, stop
===============================================*/

int CodeSelector;

void setup() {
  Serial.begin(38400);
  Serial.print("Do you want to run EStop (enter 1)?");
  //wait for user input
  while (Serial.available() == 0) {
  }
  //get user input into CodeSelecter
  CodeSelector = Serial.parseInt();
  if (CodeSelector == 1) {      //LA_Test
    setupLA(); 
  }
  else if (CodeSelector == 2) {  //EStop
    setupES(); 
  }
}

void loop() {
  if (CodeSelector == 1) {      //LA_Test
    loopLA(); 
  }
  else if (CodeSelector == 2) {  //EStop
    loopES(); 
  }
}
