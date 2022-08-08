// Limit switch pins
const int leftSwitch = 18, rightSwitch = 19, topSwitch = 21, bottomSwitch = 20;

// Motor 1 pins
const int motorRDirPin = 4, motorRSpeedPin = 5;
// Motor 2 pins
const int motorLDirPin = 7, motorLSpeedPin = 6;

volatile bool isRunning;
volatile bool foundHome = false;

// Declaring encoder a pins for left and right motors
const int encoderAL = 2, encoderAR = 3; 

// Declaring encoder b pins for left and right motors
const int encoderBL = 46, encoderBR = 44;

// Declaring position on board
volatile int xCoord, yCoord;

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
  pinMode(motorRDirPin, OUTPUT);
  pinMode(motorLDirPin, OUTPUT);

  // Encoder pin setup
  pinMode(encoderAL, INPUT_PULLUP);
  pinMode(encoderAR, INPUT_PULLUP);
  pinMode(encoderBL, INPUT_PULLUP);
  pinMode(encoderBR, INPUT_PULLUP);
  
  Stop();

  delay(100);

  FindHome();
  //RightDiagonal(255, HIGH);
  //delay(10000);

  // Allowing encoder interrupts to occur
  // Encoder A motorL/R interrupts
  cli();
  attachInterrupt(digitalPinToInterrupt(encoderAR), EncoderRInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderAL), EncoderLInterrupt, RISING);
  sei();
}

bool testPID = false;

void loop() {
  // Polling to check if limits switches have been pressed
  CheckLimits();

  /*
   * PID Controller test
   */
   while(!testPID) {
    CheckLimits();

    // Defining PID constants
    float Kp = 1;
    float Ki = 0;
    float Kd = 0; 

    float targetDist = 50; // 50mm

    // Computing error based of current count
    float error = targetDist - CountToDistance(posR);

    // Calculating control signal using PID constants
    float u = Kp*error; 

    // Calculting required pwm signal based off motor speed
    float pwmSignal = abs(u);

    // Moving motor based off the pwm signal
    Horizontal(pwmSignal, HIGH);

    // If error is within 0.1mm stop motors and exit loop
    if(abs(u) < 0.1) {
      Stop();
      testPID = true;
    }
   }

  //isRunning ? RightDiagonal(255, HIGH) : Stop();
  Serial.print(posR);
  Serial.print("---");
  Serial.print(posL);
  Serial.print("\n");

  // Testing left motor
  //isRunning ? LeftDiagonal(255, HIGH) : Stop();
  //Serial.println(posL);

  // Testing drawing 50mm straight line
  /*if(isRunning) {
    int i=0;
    while(posR<=2120) {
      CheckLimits();
      Horizontal(255, HIGH);
      Serial.println("1");
      //i<=255 ? i+=5 : i=255;
    }
    Stop();
    delay(500);
    posR = 0;
    while(posR<=2120) {
      CheckLimits();
      Vertical(255, HIGH);
      Serial.println("2");
    }
    Stop();
    delay(500);
    posR = 0;
    while(posR<=2120) {
      CheckLimits();
      Horizontal(255, LOW);
    }
    Stop();
    delay(500);
    posR = 0;
    while(posR<=2120) {
      CheckLimits();
      Vertical(255, LOW);
    }
    Stop();
  }
  isRunning = false;*/
  
  
  
  //isRunning  ? Horizontal(255, HIGH) : Stop();
  
  delay(10);
}

void FindHome(){
  Serial.println("Entering home");
  bool foundLeft = false, foundBottom = false;

  // Finding the left limit switch
  while(!foundLeft){
    Horizontal(150, LOW);
    //Serial.println(digitalRead(leftSwitch));
    if (digitalRead(leftSwitch)){
      foundLeft = true;
      Stop();
    }
  }
  delay(100);
  while(digitalRead(leftSwitch)){
    Horizontal(150, HIGH);
  }
  Stop();
  delay(100);
  
  // Finding the bottom limit switch
  while(!foundBottom){
    Vertical(150, LOW);
    Serial.println(digitalRead(bottomSwitch));
    if (digitalRead(bottomSwitch)){
      foundBottom = true;
      Stop();
    }
  }  
  delay(100);
  while(digitalRead(bottomSwitch)){
    Vertical(150, HIGH);
  }
  Stop();
  delay(10000);
  foundHome = true;
  Serial.println("Exiting home");

  // Allowing encoder interrupts to occur
  // Encoder A motorL/R interrupts
  //attachInterrupt(digitalPinToInterrupt(encoderAL), EncoderRInterrupt, RISING);
  //attachInterrupt(digitalPinToInterrupt(encoderBL), EncoderLInterrupt, RISING);

  // Declaring origin
  xCoord = 0;
  yCoord = 0;
  
}

void CheckLimits(){
  if (digitalRead(topSwitch) || digitalRead(bottomSwitch) || digitalRead(rightSwitch) || digitalRead(leftSwitch)){
    isRunning = false; 
  }
}

float CountToDistance(int count) {
  // 2064 counts/rev from the encoder
  // Calibration test: 2120 counts = 50mm
  int calCount = 2120;
  int calDist = 50; // distance in mm
  float pulleyDiam = (float)calDist / (PI*(calCount/2064));

  return (PI*pulleyDiam*(float)count)/(float)(2064); // corresponding distance in mm
}

void EncoderRInterrupt() {
  /*
   * Test drawing 50mm line
   */
  /*if(posR == 2120) {
    Stop();
    posR = 3000;
    //isRunning = false;
  } else {
    posR++;
  } */

  if(testPID) posR++;
}

void EncoderLInterrupt() {
  // Check to see if encoder is working
  //Serial.println("Entering interrupt left motor");
  /*int encoderBState = digitalRead(encoderBL);

  encoderBState > 0 ? posL++ : posL--;*/
  if(testPID) posL++;
  
}

void Horizontal(int motorSpeed, int horDir){
    analogWrite(motorRSpeedPin, motorSpeed);
    analogWrite(motorLSpeedPin, motorSpeed);
    digitalWrite(motorRDirPin, horDir);
    digitalWrite(motorLDirPin, horDir);
}

void Vertical(int motorSpeed, int verDir){
    analogWrite(motorRSpeedPin, motorSpeed);
    analogWrite(motorLSpeedPin, motorSpeed);
    digitalWrite(motorRDirPin, !verDir);
    digitalWrite(motorLDirPin, verDir);
}

void LeftDiagonal(int motorSpeed, int verDir){
    analogWrite(motorRSpeedPin, motorSpeed*verDir);
    analogWrite(motorLSpeedPin, motorSpeed*!verDir);
    digitalWrite(motorRDirPin, !verDir);
    digitalWrite(motorLDirPin, verDir);
}

void RightDiagonal(int motorSpeed, int verDir){
    analogWrite(motorRSpeedPin, motorSpeed*!verDir);
    analogWrite(motorLSpeedPin, motorSpeed*verDir);
    digitalWrite(motorRDirPin, !verDir);
    digitalWrite(motorLDirPin, verDir);
}

void Stop(){
    analogWrite(motorRSpeedPin, 0);
    analogWrite(motorLSpeedPin, 0);
}
