#define Button_LF_PIN A0
#define Button_RF_PIN A1

// Foot Buttons
int Button_LF, Button_RF;

void setupB() {
  // initialize serial communication at 9600 bits per second:
  Button_LF = 0;
  Button_RF = 0;
  Serial.println("Initializing Foot Buttons");
  pinMode(Button_LF_PIN, INPUT);
  pinMode(Button_RF_PIN, INPUT);
}

void loopB() {
  Button_LF = digitalRead(Button_LF_PIN);
  Button_RF = digitalRead(Button_RF_PIN);
  if (Button_RF == 1) {
    Button_RF = 0;
  }
  else {
    Button_RF = 1;
  }
  
  Serial.print("LF / RF:\t");
  Serial.print(Button_LF); Serial.print(" / "); Serial.println(Button_RF);
}