/* ============================================
ExoSuit Project Basics: 
Testing EStop with 1 linear actuator
Extends LA to max at 1/4 speed until EStop
Will switch to retract after 7.5s if no user input
Wired to: 
. Motor Controller (Gnd, PWM/Dig input pins 4-7)
  . Motor Controller wired to +12V/gnd & motor
. Button (gnd, 5V, DI9, pulldown resistor 1kohm)
===============================================*/

const int EStopPin = 9;

int EStop = 0;
int timeExtend = 0;

void setupES() {
  setupLA();
  pinMode(EStopPin, INPUT);
}

void loopES() {
  if (timeExtend >= 7500) { //2s max speed -> 8s at 1/4 speed
    for (int i=0; i<15; i++) { //7500/500 = 15 (15 intervals of 0.5s = 7.5s)
      EStop = digitalRead(EStopPin);
      if (EStop == 1) {
        Serial.println("Emergency Stop Engaged! (R)");
        while (1) {
          Serial.println("In endless while loop (R)"); //this stops code from doing anything else
          delay(1000); //delay 1s before writing again
        }
      }
      retractVarLA(64, 500); //retract LA at 1/4 speed 
      timeExtend -= 500;
    }
  }
  extendVarLA(64, 500); //extend at 1/4 speed for 0.5s 
  timeExtend += 500;
  EStop = digitalRead(EStopPin);
  if (EStop == 1) {
    Serial.println("Emergency Stop Engaged! (E)");
    while (1) {
      Serial.println("In endless while loop (E)"); //this stops code from doing anything else
      delay(1000); //delay 1s before writing again
    }
  }
}