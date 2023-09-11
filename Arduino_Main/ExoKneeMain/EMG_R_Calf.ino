
// the setup routine runs once when you press reset:
void setupEMGB() {
  // initialize serial communication at 9600 bits per second:
  EMG_BOT_VAL = 0;
  pinMode(EMG_BOT_PIN, INPUT);
}

// the loop routine runs over and over again forever:
void loopEMGB() {
  // read the input on analog pin 0:
  EMG_BOT_VAL = analogRead(EMG_BOT_PIN);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  //float voltage = EMG_BOT_VAL * (5.0 / 1023.0);
  // print out the value you read:
  Serial.println(EMG_BOT_VAL);
}
