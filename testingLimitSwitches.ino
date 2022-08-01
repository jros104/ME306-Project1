// Limit switch pins
const int leftSwitch = 18, rightSwitch = 19, topSwitch = 21, bottomSwitch = 20;

// Motor 1 pins
const int motorRDir = 4, motorRSpeed = 5;
// Motor 2 pins
const int motorLDir = 7, motorLSpeed = 6;

volatile bool isRunning;
volatile bool foundHome = false;

// Declaring encoder a pins
const int encoderAL = 2, encoderAR = 3; 

// Declaring encoder b pins
const int encoderBL = 46, encoderBR = 44;

void setup() {
  isRunning = true;
  
  Serial.begin(9600);
  // Limit Switch setups
  pinMode(leftSwitch, INPUT_PULLUP);
  pinMode(rightSwitch, INPUT_PULLUP);
  pinMode(topSwitch, INPUT_PULLUP);
  pinMode(bottomSwitch, INPUT_PULLUP);

  // Motor pin setup
  pinMode(motorRDir, OUTPUT);
  pinMode(motorLDir, OUTPUT);

  // Encoder pin setup
  pinMode(encoderAL, INPUT_PULLUP);
  pinMode(encoderAR, INPUT_PULLUP);
  pinMode(encoderBL, INPUT_PULLUP);
  pinMode(encoderBR, INPUT_PULLUP);

  // Encoder A motorL/R interrupts
  //attachInterrupt(digitalPinToInterrupt(encoderAL), EncoderRInterrupt, RISING);
  //attachInterrupt(digitalPinToInterrupt(encoderAL), EncoderLInterrupt, RISING);
  
  Stop();

  delay(100);

  FindHome();
}

void loop() {
  CheckLimits();

  isRunning ? RightDiagonal(255, HIGH) : Stop();

  /*if (isRunning){
    RightDiagonal(255, HIGH);
  }else{
    Stop();
  }*/
  Serial.println(isRunning);
  delay(10);
}

void FindHome(){
  Serial.println("Entering home");
  bool foundLeft = false, foundBottom = false;

  // Finding the left limit switch
  while(!foundLeft){
    Horizontal(200, LOW);
    //Serial.println(digitalRead(leftSwitch));
    if (digitalRead(leftSwitch)){
      foundLeft = true;
      Stop();
    }
  }
  delay(100);
  while(digitalRead(leftSwitch)){
    Horizontal(200, HIGH);
  }
  Stop();
  delay(100);
  
  // Finding the bottom limit switch
  while(!foundBottom){
    Vertical(200, LOW);
    //Serial.println(digitalRead(bottomSwitch));
    if (digitalRead(bottomSwitch)){
      foundBottom = true;
      Stop();
    }
  }  
  delay(100);
  while(digitalRead(bottomSwitch)){
    Vertical(200, HIGH);
  }
  Stop();
  delay(100);
  foundHome = true;
  Serial.println("Exiting home");
}

void CheckLimits(){
  if (digitalRead(topSwitch) || digitalRead(bottomSwitch) || digitalRead(rightSwitch) || digitalRead(leftSwitch)){
    isRunning = false; 
  }
}

void EncoderRInterrupt() {
  
}

void EncoderLInterrupt() {
  
}

/*void SwitchLimit(){
  if(!foundHome){
    isRunning = false;
    Serial.println("Interrupt");
  }
  //isRunning = false;
  //Serial.println("Interrupt");
}*/

void Horizontal(int motorSpeed, int horDir){
    analogWrite(motorRSpeed, motorSpeed);
    analogWrite(motorLSpeed, motorSpeed);
    digitalWrite(motorRDir, horDir);
    digitalWrite(motorLDir, horDir);
}

void Vertical(int motorSpeed, int verDir){
    analogWrite(motorRSpeed, motorSpeed);
    analogWrite(motorLSpeed, motorSpeed);
    digitalWrite(motorRDir, !verDir);
    digitalWrite(motorLDir, verDir);
}

void LeftDiagonal(int motorSpeed, int verDir){
    analogWrite(motorRSpeed, motorSpeed*verDir);
    analogWrite(motorLSpeed, motorSpeed*!verDir);
    digitalWrite(motorRDir, !verDir);
    digitalWrite(motorLDir, verDir);
}

void RightDiagonal(int motorSpeed, int verDir){
    analogWrite(motorRSpeed, motorSpeed*!verDir);
    analogWrite(motorLSpeed, motorSpeed*verDir);
    digitalWrite(motorRDir, !verDir);
    digitalWrite(motorLDir, verDir);
}

void Stop(){
    analogWrite(motorRSpeed, 0);
    analogWrite(motorLSpeed, 0);
}
