#define ACCELERATION_TIME 100

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

// PID Controller Class
// Can be instantiated with PID values and its CalculateEffort can be called
// by passing the current error, the saturation limit for the P effort, and dt
// dt --> is the time between the last CalculateEffort call, and the current time
class PIDController{
  private:
    // Variables
    float Kp;
    float Ki;
    float Kd;
    int sumError;
    int prevError;
  public:
    // Contructor
    PIDController(float Kp, float Ki, float Kd){
      this->Kp = Kp;
      this->Ki = Ki;
      this->Kd = Kd;
      this->sumError = 0;
      this->prevError = 0;
    }

    // Function to calculate the control effort
    float CalculateEffort(float error, int saturationLimit, float dt){
      // Proportional Control Effort
      float P = Kp * error;
      int sign = (P > 0) - (P < 0);
      // Saturating the Proportional Control Effort
      P = abs(P) > saturationLimit ? saturationLimit * sign : P;

      // Anti Integral windup
      if (abs(P) < saturationLimit){
        sumError += error * dt;  
      }
      // Integral Control Effort
      float I = Ki * sumError;

      // Derivative Control Effort
      float D = Kd * ((error - prevError) / dt);

      // Updating the previous Error
      prevError = error;

      float u = P + I + D;

      u = abs(u) > 230 ? 230 * ((u > 0) - (u < 0)) : u;
      // Returning the total control effort
      return u;
    }

    // Function to reset sum and previous error
    void ResetPID(){
      this->sumError = 0;
      this->prevError = 0;
    }
};

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

//  while(1){
//    RightDiagonal(100, HIGH);
//    delay(10);
//  }

  FindHome();

  
  
  
  
  // Allowing encoder interrupts to occur
  // Encoder A motorL/R interrupts
  cli();
  attachInterrupt(digitalPinToInterrupt(encoderAR), EncoderRInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderAL), EncoderLInterrupt, RISING);
  sei();

  //Serial.println(CountToDistance(2120));
}

bool testPID = false;

void loop() {
  //CheckLimits();
  
  posR = 0;
  posL = 0;

  for(int i=0; i<300; i++) {
    RightDiagonal(100, HIGH);
    delay(10);
    //CheckLimits();
  }
  Stop();

  delay(10000);

  HorizontalDistance(40,1,0,0);

  /*
   * PID Controller test
   
   while(!testPID) {
    CheckLimits();

    // Defining PID constants
    float Kp = 2;
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
    if(abs(error) < 0.1) {
      Stop();
      testPID = true;
    }
   }

   Stop();
   */  
  delay(10);
}

void FindHome(){
  //Serial.println("Entering home");
  bool foundLeft = false, foundBottom = false;

  // Finding the left limit switch
  while(!foundLeft){
    Horizontal(100, LOW);
    Serial.println(digitalRead(leftSwitch));
    if (digitalRead(leftSwitch)){
      foundLeft = true;
      Stop();
    }
    delay(10);
  }
  delay(500);
  while(digitalRead(leftSwitch)){
    Horizontal(100, HIGH);
    delay(10);
  }
  Stop();
  delay(500);
  
  // Finding the bottom limit switch
  while(!foundBottom){
    Vertical(100, LOW);
    Serial.println(digitalRead(bottomSwitch));
    if (digitalRead(bottomSwitch)){
      foundBottom = true;
      Stop();
    }
    delay(10);
  }  
  delay(500);
  while(digitalRead(bottomSwitch)){
    Vertical(100, HIGH);
    delay(10);
  }
  Stop();
  delay(1000);
  foundHome = true;
  Serial.println("Exiting home");

  // Declaring origin
  xCoord = 0;
  yCoord = 0;
  
}

void CheckLimits(){
  if (digitalRead(topSwitch) || digitalRead(bottomSwitch) || digitalRead(rightSwitch) || digitalRead(leftSwitch)){
    isRunning = false; 
    testPID = false;

    while(1) Stop();
  }
}

float CountToDistance(int count) {
  // 2064 counts/rev from the encoder
  // Calibration test: 2120 counts = 50mm
  float calCount = 2140;
  float calDist = 50; // distance in mm
  float pulleyDiam = (float)calDist / (PI*(calCount/2064.0));

  return (PI*pulleyDiam*(float)count)/2064.0; // corresponding distance in mm
}

float DistanceToCount(int distance) {
  // 2064 counts/rev from the encoder
  // Calibration test: 2120 counts = 50mm
  float calCount = 2300;
  float calDist = 50; // distance in mm
  float pulleyDiam = (float)calDist / (PI*(calCount/2064.0));

  return (distance * 2064.0) / (PI * pulleyDiam);
}

void EncoderRInterrupt() {
   int encoderBState = digitalRead(encoderBR);
   encoderBState > 0 ? posR++ : posR--;
}

void EncoderLInterrupt() {
  int encoderBState = digitalRead(encoderBL);
  encoderBState > 0 ? posL++ : posL--;
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

float saturate(float value, float max){
  return abs(value) > max ? max * sign(value) : value;
}

int sign(float value){
  return ((value > 0) - (value < 0));
}

void HorizontalDistance(float distance, float Kp, float Ki, float Kd){
   
    // Create the PID Controllers with the assigned values
    PIDController motorPID(Kp, Ki, Kd);
    PIDController motorDiffPID(0.5,0,0);
   
    float controlEffortL, controlEffortR, controlEffortDiff, dt, prevTime = millis();
    float initialTime = millis();
    float exitTime = 0;
    float motorLDistance = DistanceToCount(distance);

    posL = 0;
    posR = 0;
   
    while(isRunning){
      CheckLimits();
      dt = millis() - prevTime;

      // Calculating PID control efforts for the motor base powers and the encoder variation motor powers
       //CONVERSION FACTOR FOR HORIZONTAL HERE
      float errorDistance = motorLDistance - posL;
      float errorEnc = posL - posR;
      controlEffortL = motorPID.CalculateEffort(errorDistance, 235, dt);
      controlEffortDiff = motorDiffPID.CalculateEffort(errorEnc, 40 , dt);

      // Outputs for debugging;
//      Serial.print("The distance error is: ");
//      Serial.println(errorDistance);
//
//      Serial.print("Base control effort is: ");
//      Serial.println(controlEffortL);
////
//      Serial.print("The enconder error is: ");
//      Serial.println(errorEnc);
//
//      Serial.print("Encoder difference control effort is: ");
//      Serial.println(controlEffortDiff);
//      Serial.println();

//      Serial.print("Pos R: ");
//      Serial.println(posR);
//
//      Serial.print("Pos L: ");
//      Serial.println(posL);

      // Applying linear accelearation;
      controlEffortL = ((millis() - initialTime) < ACCELERATION_TIME) ? controlEffortL * ((millis() - initialTime) / ACCELERATION_TIME) : controlEffortL;

      // Alternative acceleration code not using a turnery operator:
      /*
      float saturatedTime = saturate(millis() - initialTime, ACCELERATION_TIME);
      controlEffortL *= (saturatedTime / ACCELERATION_TIME);
      */

      // Applying the control effort to both motors
      controlEffortR = controlEffortL;

      // Saturating the control efforts
      controlEffortL = saturate(controlEffortL - (controlEffortDiff / 2.0), 255);
      
      controlEffortR = saturate(controlEffortR + (controlEffortDiff / 2.0), 255);


      // Setting the motor speeds;
      analogWrite(motorLSpeedPin, abs(controlEffortL));
      analogWrite(motorRSpeedPin, abs(controlEffortR));

      // Setting the motor powers;
      digitalWrite(motorLDirPin, sign(controlEffortL) == -1 ? 0 : 1);
      digitalWrite(motorRDirPin, sign(controlEffortR) == -1 ? 0 : 1);
     
      prevTime = millis();

      if (errorDistance < 0.1){
        isRunning = false;
      }
      delay(1);

     

    }
    Stop();
    delay(10000);
}



/*
 * PID controller class
 * /
 */
