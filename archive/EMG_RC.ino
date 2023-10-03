/* ==============================================
ExoSuit Project Main: 
EMG (Electromyography) Sensor for Right Calf
Wired to: +-9V/1A, A2
=================================================
For EMGs; 
Value is between 0-1000 where >500 is high
For now, only counting MuscleUse as a bool
May add levels based on average findings (0-3)
=================================================*/

// the setup routine runs once when you press reset:
void setupEMGRC() {
  // initialize serial communication at 9600 bits per second:
  EMG_RC_VAL = 0;
  Serial.println("Initializing A2 Device [EMG_RC]");
  pinMode(EMG_RC_PIN, INPUT);
}

// the loop routine runs over and over again forever:
void loopEMGRC() {
  // read the input on analog pin 0:
  EMG_RC_VAL = analogRead(EMG_RC_PIN);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  //float voltage = EMG_BOT_VAL * (5.0 / 1023.0);
  // print out the value you read:
  Serial.println(EMG_RC_VAL);
}
