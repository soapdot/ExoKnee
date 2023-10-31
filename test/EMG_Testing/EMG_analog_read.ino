/* ============================================
ExoSuit Project Basics: 
Trying individual EMG Sensor 
This code converts analog input to voltage 
and prints graphical representation via serial plotter
Only have one plugged into the A2 connection
===============================================*/

int sensorValue;
int sensorPin = A1;
// the setup routine runs once when you press reset:
void setupEMG() { 
  sensorValue = 0;
  pinMode(sensorPin, INPUT);
}

// the loop routine runs over and over again forever:
void loopEMG() {
  // read the input on analog pin 0:
  sensorValue = analogRead(sensorPin);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  //float voltage = sensorValue * (5.0 / 1023.0);
  // print out the value you read:
  Serial.println(sensorValue);
}
