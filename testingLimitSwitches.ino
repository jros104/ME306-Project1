// Limit switch pins
const int leftSwitch = 18, rightSwitch = 19, topSwitch = 21, bottomSwitch = 20;

// Motor pins
const int motorOneDir = 4, motorOneSpeed = 5;

volatile bool isRunning;

int motorSpeed;
float initialTime;

void setup() {
  isRunning = true;
  motorSpeed = 0;
  
  Serial.begin(9600);
  // Limit Switche setups
  pinMode(leftSwitch, INPUT);
  pinMode(rightSwitch, INPUT);
  pinMode(topSwitch, INPUT);
  pinMode(bottomSwitch, INPUT);

  // Motor pin setup
  pinMode(motorOneDir, OUTPUT);

  // Interrupt Setup

  cli();
  
  attachInterrupt(digitalPinToInterrupt(leftSwitch), LimitSwitch, RISING);
  attachInterrupt(digitalPinToInterrupt(rightSwitch), LimitSwitch, RISING);
  attachInterrupt(digitalPinToInterrupt(topSwitch), LimitSwitch, RISING);
  attachInterrupt(digitalPinToInterrupt(bottomSwitch), LimitSwitch, RISING);
  
  digitalWrite(motorOneDir, LOW);
  analogWrite(motorOneSpeed, 0);

  initialTime = millis();
  
}

void loop() {
  if (motorSpeed < 150){
    motorSpeed++;
  }
  if (isRunning){
    analogWrite(motorOneSpeed, motorSpeed);
  }else{
    analogWrite(motorOneSpeed, 0);
  }
  Serial.println(digitalRead(topSwitch));
  delay(10);
  if ((millis() - initialTime) > 1000){
    sei();
  }
}

void LimitSwitch(){
  Serial.println("Interrupting");
  isRunning = false;
}
