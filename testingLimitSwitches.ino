// Limit switch pins
const int leftSwitch = 18, rightSwitch = 19, topSwitch = 21, bottomSwitch = 20;

// Motor 1 pins
const int motorRDir = 4, motorRSpeed = 5;
// Motor 2 pins
const int motorLDir = 7, motorLSpeed = 6;

volatile bool isRunning;
volatile bool foundHome = false;

// Declaring encoder a pins for left and right motors
const int encoderAL = 2, encoderAR = 3; 

// Declaring encoder b pins for left and right motors
const int encoderBL = 46, encoderBR = 44;

// Declaring position on board
volatile int xCoord, yCoord;

// Defining 

// For testing
volatile int posL = 0, posR = 0;

/*
 * RECTANGLE WIDTH AND HEIGHT, CIRCLE DIAMETER
 */
volatile const int rectWidth = 5;
volatile const int rectHeight = 10;

volatile const int circDiam = 5;



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
  
  Stop();

  delay(100);

  FindHome();
}

void loop() {
  // Polling to check if limits switches have been pressed
  CheckLimits();

  isRunning ? RightDiagonal(255, HIGH) : Stop();
  Serial.println(posR);

  // Testing left motor
  //isRunning ? LeftDiagonal(255, HIGH) : Stop();
  //Serial.println(posL);

  // Testing drawing 50mm straight line
  isRunning ? Horizontal(255, HIGH) : Stop();
  
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

  // Allowing encoder interrupts to occur
  // Encoder A motorL/R interrupts
  attachInterrupt(digitalPinToInterrupt(encoderAL), EncoderRInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderBL), EncoderLInterrupt, RISING);

  // Declaring origin
  xCoord = 0;
  yCoord = 0;
}

void CheckLimits(){
  if (digitalRead(topSwitch) || digitalRead(bottomSwitch) || digitalRead(rightSwitch) || digitalRead(leftSwitch)){
    isRunning = false; 
  }
}

void EncoderRInterrupt() {
  // Check to see if encoder is working
  // Right diagonal -> right motor counter clockwise
  int encoderBState = digitalRead(encoderBR);

  encoderBState > 0 ? posR++ : posR--;

  /*
   * Test drawing 50mm line
   */
//  if(posR == 8212) {
//    Stop();
//  } else {
//    posR++;
//  } 
}

void EncoderLInterrupt() {
  // Check to see if encoder is working
  int encoderBState = digitalRead(encoderBL);

  encoderBState > 0 ? posL++ : posL--;
  
}

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
