
// Limit switch pins
const int leftSwitch = 18, rightSwitch = 19, topSwitch = 21, bottomSwitch = 20;

// Motor 1 pins
const int motorOneDir = 4, motorOneSpeed = 5;
// Motor 2 pins
const int motorTwoDir = 7, motorTwoSpeed = 6;

volatile bool isRunning;
volatile bool foundHome = false;

void setup() {
  isRunning = true;
  
  Serial.begin(9600);
  // Limit Switch setups
  pinMode(leftSwitch, INPUT_PULLUP);
  pinMode(rightSwitch, INPUT_PULLUP);
  pinMode(topSwitch, INPUT_PULLUP);
  pinMode(bottomSwitch, INPUT_PULLUP);

  // Motor pin setup
  pinMode(motorOneDir, OUTPUT);
  pinMode(motorTwoDir, OUTPUT);
  
  Stop();

  attachInterrupt(digitalPinToInterrupt(rightSwitch), SwitchLimit, RISING); 
  attachInterrupt(digitalPinToInterrupt(leftSwitch), SwitchLimit, RISING); 
  attachInterrupt(digitalPinToInterrupt(topSwitch), SwitchLimit, RISING); 
  attachInterrupt(digitalPinToInterrupt(bottomSwitch), SwitchLimit, RISING); 
  
  
}

void loop() {
  if (!foundHome){
    FindHome();
  }

  //CheckLimits();

  if (isRunning){
    RightDiagonal(100, HIGH);
  }else{
    Stop();
  }
  Serial.println(isRunning);
  delay(10);
}

void FindHome(){
  bool foundLeft = false, foundBottom = false;

  // Finding the left limit switch
  while(!foundLeft){
    Horizontal(100, HIGH);
    if (digitalRead(leftSwitch)){
      Serial.println(digitalRead(leftSwitch));
      foundLeft = true;
      Stop();
    }
  }
  delay(100);
  while(digitalRead(leftSwitch)){
    Horizontal(100, LOW);
  }
  Stop();
  delay(100);
  
  // Finding the bottom limit switch
  while(!foundBottom){
    Vertical(100, HIGH);
    if (digitalRead(bottomSwitch)){
      foundBottom = true;
      Stop();
    }
  }  
  delay(100);
   while(digitalRead(bottomSwitch)){
    Vertical(100, LOW);
  }
  Stop();
  delay(100);
  Stop();
  foundHome = true;
}

void CheckLimits(){
  if (digitalRead(topSwitch) || digitalRead(bottomSwitch) || digitalRead(rightSwitch) || digitalRead(leftSwitch)){
    isRunning = false; 
  }
}


void SwitchLimit(){
  if(foundHome){
    isRunning = false;
    Serial.println("Right Interrupt");
  }
}

void Horizontal(int motorSpeed, int horDir){
    analogWrite(motorOneSpeed, motorSpeed);
    analogWrite(motorTwoSpeed, motorSpeed);
    digitalWrite(motorOneDir, horDir);
    digitalWrite(motorTwoDir, horDir);
}

void Vertical(int motorSpeed, int verDir){
    analogWrite(motorOneSpeed, motorSpeed);
    analogWrite(motorTwoSpeed, motorSpeed);
    digitalWrite(motorOneDir, !verDir);
    digitalWrite(motorTwoDir, verDir);
}

void LeftDiagonal(int motorSpeed, int verDir){
    analogWrite(motorOneSpeed, motorSpeed*verDir);
    analogWrite(motorTwoSpeed, motorSpeed*!verDir);
    digitalWrite(motorOneDir, !verDir);
    digitalWrite(motorTwoDir, verDir);
}

void RightDiagonal(int motorSpeed, int verDir){
    analogWrite(motorOneSpeed, motorSpeed*!verDir);
    analogWrite(motorTwoSpeed, motorSpeed*verDir);
    digitalWrite(motorOneDir, !verDir);
    digitalWrite(motorTwoDir, verDir);
}

void Stop(){
    analogWrite(motorOneSpeed, 0);
    analogWrite(motorTwoSpeed, 0);
}
