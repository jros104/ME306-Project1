// ######################################################### PID CLASS #########################################################
// Can be instantiated with PID values and its CalculateEffort can be called
// by passing the current error, the saturation limit for the P effort, and dt
// dt --> is the time between the last CalculateEffort call, and the current time
class PIDController{
  private:
    // Variables
    float Kp;
    float Ki;
    float Kd;
    
    float sumError;
    float prevError;
    float P, I, D;
  public:
    // Contructor
    PIDController(float Kp, float Ki, float Kd){
      this->Kp = Kp;
      this->Ki = Ki;
      this->Kd = Kd;

      this->sumError = 0;
      this->prevError = 0;
      this->P = 0;
      this->I = 0;
      this->D = 0;
    }

    // Function to calculate the control effort
    float CalculateEffort(float error, int saturationLimit, float dt){
      // Proportional Control Effort
      P = Kp * error;
      int sign = (P > 0) - (P < 0);
      // Saturating the Proportional Control Effort
      P = P > saturationLimit ? saturationLimit : P < -saturationLimit ? -saturationLimit : P;

      // Anti Integral windup
      if (abs(P) < saturationLimit){
        sumError += error * dt;  
      }
      // Integral Control Effort
      I = Ki * sumError;

      // Derivative Control Effort
      if (dt != 0){
        D = Kd * ((error - prevError) / dt);
      }

      //Serial.println(sumError);
      
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


// ######################################################### ARDUINO CODE SETUP #########################################################

#define ACCELERATION_TIME 300
#define EXIT_THRESHOLD 5

#define CALCOUNT 2300.0
#define CALDIST 50.0
#define PULLEYDIAM (CALDIST / (PI * (CALCOUNT / 2064.0)))

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

// ######################################################### SETUP #########################################################

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

  //FindHome();
  
  
  // Allowing encoder interrupts to occur
  // Encoder A motorL/R interrupts
  cli();
  attachInterrupt(digitalPinToInterrupt(encoderAR), EncoderRInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderAL), EncoderLInterrupt, RISING);
  sei();

  FindHomeV2();
  MoveDistance(100,0,0.5,0.01,0, 100, true);
  MoveDistance(0,60,0.5,0.01,0, 100, true);
  isRunning = true;
  Circle(30,16);
}

// ######################################################### LOOP #########################################################

void loop() {
  //CheckLimits();
  isRunning = true; 
  posR = 0;
  posL = 0;

  Stop();

  delay(2000);
  // CircleV2(20, 4);
  // Drawing a square
//  MoveDistance(0,60,0.5,0.01,0, 100, true);
//  MoveDistance(60,0,0.5,0.01,0, 100, true);
//  MoveDistance(0,-60,0.5,0.01,0, 100, true);
//  MoveDistance(-60,0,0.5,0.01,0, 100, true);

  delay(1000);
}

// ######################################################### FUNCTIONS #########################################################
// =========================== FIND HOME ===========================
void FindHomeV2(){
  Serial.println("Finding Bottom Switch");
  MoveDistance(0, -1000, 0.05, 0, 0, 100, true);
  delay(100);
  isRunning = true;
  
  Serial.println("Finding Left Switch");
  MoveDistance(-1000, 0, 0.05, 0, 0, 100, true);
  delay(100);
  isRunning = true;

  delay(1000);
  // Declaring origin
  xCoord = 0;
  yCoord = 0;
}

// =========================== CHECK LIMITS ===========================
void CheckLimits(){
  if (digitalRead(topSwitch) || digitalRead(bottomSwitch) || digitalRead(rightSwitch) || digitalRead(leftSwitch)){

    // Move the head away from the switch
    if(digitalRead(topSwitch)) { 
      Serial.println("Top Switch");
      MoveDistance(0, -13, 0.2, 0, 0, 100, false); 
      
    }
    if(digitalRead(bottomSwitch)) { 
      Serial.println("Bottom Switch");
      MoveDistance(0, 13, 0.2, 0, 0, 100, false); 
      
    }
    if(digitalRead(rightSwitch)) { 
      Serial.println("Right Switch");
      MoveDistance(-13, 0, 0.2, 0, 0, 100, false); 
    }
    if(digitalRead(leftSwitch)) { 
      Serial.println("Left Switch");
      MoveDistance(13, 0, 0.2, 0, 0, 100, false); 
    }
    
    
    isRunning = false; 
  }
}

// =========================== COUNT TO DISTANCE ===========================
float CountToDistance(float count) {
  return (PI * PULLEYDIAM * count)/2064.0; 
}

// =========================== DISTANCE TO COUNT ===========================
float DistanceToCount(float distance) {
  return (distance * 2064.0) / (PI * PULLEYDIAM);
}

void EncoderRInterrupt() {
   int encoderBState = digitalRead(encoderBR);
   encoderBState > 0 ? posR-- : posR++;
}

void EncoderLInterrupt() {
  int encoderBState = digitalRead(encoderBL);
  encoderBState > 0 ? posL--: posL++;
}

// =========================== SATURATE ===========================
float saturate(float value, float min, float max){
  return value > max ? max : value < min ? min : value;
}

// =========================== SIGN ===========================
// Returns the sign of a value:
// -1 for -ve ; 0 for 0 ; 1 for +ve
int sign(float value){
  return ((value > 0) - (value < 0));
}

// =========================== MOVE DISTANCE ===========================
// Requires the horizontal and vertical distance you want the head to move
// Requires Kp, Ki, Kd
// Checklimits bool decides whether the function will check to see if it hits the limit switch
// (used for the FindHomeV2)
void MoveDistance(float xDist, float yDist, float Kp, float Ki, float Kd, float saturationLimit, bool checkLimits){
   
    // Create the PID Controllers with the assigned values
    PIDController motorPID(Kp, Ki, Kd);         // Motor base power PID controller
    PIDController motorDiffPID(2,0,0);        // Encoder difference PID controller
   
    float controlEffortL, controlEffortR, controlEffortDiff, dt, motorLError, motorRError, encError;
    float initialTime = millis();
    float prevTime = millis();
    float exitTime = 0;

    posL = 0;       // Reset left encoder
    posR = 0;       // Reset right encoder

    float motorLEncTarget = DistanceToCount(xDist + yDist);
    float motorREncTarget = DistanceToCount(xDist - yDist);

    float encRatio = motorLEncTarget == 0 || motorREncTarget == 0 ? 0 : motorLEncTarget / motorREncTarget;
    bool canExit = false;
    bool checkingExit = false;

    float prevErrorL = 0;

    Serial.print("X: ");
    Serial.print(xDist);
    Serial.print(" Y: ");
    Serial.println(yDist);
   
    do{
      if (checkLimits) CheckLimits();
      dt = millis() - prevTime;

      // Calculating PID control efforts for the motor base powers and the encoder variation motor powers
      motorLError = motorLEncTarget - posL;
      encError = posL - (posR * encRatio);         // Applying the encoder ratio to the error calc

      controlEffortL = motorPID.CalculateEffort(motorLError, saturationLimit, dt);        // Left motor control effort calculation
      controlEffortDiff = motorDiffPID.CalculateEffort(encError, 30, dt);     // Encoder error control effort calculation

      //controlEffortL = ((millis() - initialTime) < ACCELERATION_TIME) ? controlEffortL * ((millis() - initialTime) / ACCELERATION_TIME) : controlEffortL;
      
      controlEffortR = controlEffortL / encRatio;
      controlEffortR = controlEffortR + (controlEffortDiff) * sign(encRatio);

      analogWrite(motorLSpeedPin, abs(controlEffortL));
      analogWrite(motorRSpeedPin, abs(controlEffortR));

      digitalWrite(motorLDirPin, sign(controlEffortL) == -1 ? 0 : 1);         // Setting the motor direction to either 0 or 1 depending on the sign
      digitalWrite(motorRDirPin, sign(controlEffortR) == -1 ? 0 : 1);
     
      float currentError = abs(motorLError - prevErrorL);
      
      if (checkingExit == false && (abs(motorLError) <= EXIT_THRESHOLD || currentError == 0) && (abs(posL) > 0 || abs(posR) > 0)){
        checkingExit = true;
        exitTime = millis();

        if (abs(motorLError) <= EXIT_THRESHOLD){
          Serial.println("within threshold");
        }

        if (currentError == 0){
          Serial.println("No changing error");
        }
      }

      if (checkingExit == true && (abs(motorLError) > 20 && currentError != 0)){
        checkingExit = false;
      }

      if (checkingExit == true && (millis() - exitTime > 200)){
         Serial.println("Can Exit");
        canExit = true;
      }

      
      
      delay(10);
      prevTime = millis();
      prevErrorL = motorLError;
      if (checkLimits) CheckLimits();
    } while(!canExit && isRunning);
    Stop();
}

// =========================== BASIC MOTOR FUNCTIONS ===========================
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

void Circle(float radius, int divisions){
  Serial.println("Draw Circle");
  // Finding angle of each triangle segment
  float Angle = (PI * 2.0) / (float)divisions;
  Serial.println(Angle);
  // Creating an empty 2D array of xy disitances
  float xyDistances[divisions][2] = {0};

  // Determining the initial x and y distances
  float deltaX = radius * (1 - cos(Angle));
  float deltaY = radius * (sin(Angle));

  Serial.print("deltaX 1 = ");
  Serial.println(deltaX);
  Serial.print("deltaY 1 = ");
  Serial.println(deltaY);

  // Assigining them
  xyDistances[0][0] = -deltaX;
  xyDistances[0][1] = deltaY;

  // Rotating each xy vector by the interior angle
  for (int i = 1 ; i < divisions; i++){
    xyDistances[i][0] = (cos(Angle) * xyDistances[i-1][0] - sin(Angle) * xyDistances[i-1][1]);
    xyDistances[i][1] = (sin(Angle )* xyDistances[i-1][0] + cos(Angle)* xyDistances[i-1][1]);
  }

//  for (int i = 0; i < divisions; i++){
//    Serial.print("x: ");
//    Serial.print(xyDistances[i][0]);
//    Serial.print("      y: ");
//    Serial.println(xyDistances[i][1]);
//  }

  // A distances and B distances for the motor to move
  float L[divisions] = {0};
  float R[divisions] = {0};

  for (int i = 0; i < divisions; i++){
    L[i] = DistanceToCount(xyDistances[i][0] + xyDistances[i][1]);
    R[i] = DistanceToCount(xyDistances[i][0] - xyDistances[i][1]);
  }

//  Serial.print(" L = ");
//  for (int i = 0; i < divisions; i++){
//    Serial.print(L[i]);
//    Serial.print(" ");
//  }
//  Serial.println();
//
//  Serial.print(" R = ");
//  for (int i = 0; i < divisions; i++){
//    Serial.print(R[i]);
//    Serial.print(" ");
//  }
//  Serial.println();

  PIDController motorDiffPID(0.5,0,0);
  float controlEffortDiff;
  
  float SPEED = 200;
  float LSpeed, RSpeed;
  for (int i = 0; i < divisions; i++){
    float encRatio;
    Serial.print("Loop: ");
    Serial.println(i);
    if (L[i] != 0 && R[i] != 0){
      encRatio = abs(L[i] / R[i]);

      if(L[i] >= R[i]){
         LSpeed = SPEED;
          RSpeed = SPEED / abs(encRatio);
      }else{
        RSpeed = SPEED;
          LSpeed = SPEED *  abs(encRatio);
      }
     
  
      float totalSpeed = abs(LSpeed) + abs(RSpeed);
      float ratio = SPEED / totalSpeed;
    
      LSpeed *= ratio;
      RSpeed *= ratio;

      //LSpeed = LSpeed < 0 ? saturate(LSpeed,-255,-60) : saturate(LSpeed,60,255);
      //RSpeed = RSpeed < 0 ? saturate(RSpeed,-255,-60) : saturate(RSpeed,60,255);
      
    }else{
      if (L[i] == 0){
        LSpeed = 0;
        RSpeed = SPEED;
      }else{
        RSpeed = 0;
        LSpeed = SPEED;
      }
    }
    
    posL = 0;
    posR = 0;

    bool canExit = false;

    Serial.print("Left Speed = ");
    Serial.print(LSpeed);
    Serial.print("Right Speed = ");
    Serial.println(RSpeed);

    float dt, prevTime = millis();

    do{
      dt = millis() - prevTime;
      prevTime = millis();
      if (L[i] != 0 && R[i] != 0){
        int encError = abs(posL) - abs(L[i] >= R[i] ? posR / encRatio : posR * encRatio);
        encError *= sign(R[i]);
        controlEffortDiff = motorDiffPID.CalculateEffort(encError, 30, dt);
        Serial.print("enc effort: ");
        Serial.println(controlEffortDiff);
      }
      
      
      CheckLimits();
      if (abs(L[i]) != 0){
//        Serial.print("abs(posL) = ");
//        Serial.print(abs(posL));
//        Serial.print("     abs(L[i]) = ");
//        Serial.println(abs(L[i]));
      }

      if (abs(R[i]) != 0){
//        Serial.print("abs(posR) = ");
//        Serial.print(abs(posR));
//        Serial.print("     abs(R[i]) = ");
//        Serial.println(abs(R[i]));
      }
      
      analogWrite(motorLSpeedPin, LSpeed);
      analogWrite(motorRSpeedPin, RSpeed - controlEffortDiff);

      digitalWrite(motorLDirPin, sign(L[i]) == -1 ? 0 : 1);       
      digitalWrite(motorRDirPin, sign(R[i]) == -1 ? 0 : 1);

      if (abs(posL) >= abs(L[i]) && L[i] != 0){
        canExit = true;
        Serial.println("Left motor reached Pos");
      }

      if (abs(posR) >= abs(R[i]) && R[i] != 0){
        canExit = true;
        Serial.println("Right motor reached Pos");
      }
      
      delay(10);
      
    }while(!canExit && isRunning);
    Serial.print("End of Loop: ");
    Serial.println(i);
    
  }
  Stop();

}

// Ignore for now
void CircleV2(float radius, int divisions){
  float xyDistsFromCentre[divisions][2] = {0};
  float angle = (PI * 2.0) / (float)divisions;

  for (int i = 0; i < divisions; i++){
    xyDistsFromCentre[i][0] = (radius * cos(angle*i));
    xyDistsFromCentre[i][1] = (radius * sin(angle*i));
  }

  for (int i = 0; i < divisions; i++){
    Serial.print("x: ");
    Serial.print(xyDistsFromCentre[i][0]);
    Serial.print("      y: ");
    Serial.println(xyDistsFromCentre[i][1]);
  }

  float xyDistsToNext[divisions][2] = {0};

  for (int i = 0; i < divisions; i++){
    xyDistsToNext[i][0] = xyDistsFromCentre[(i + 1 >= divisions ? i - divisions + 1: i + 1)][0] - xyDistsFromCentre[i][0];
    xyDistsToNext[i][1] = xyDistsFromCentre[(i + 1 >= divisions ? i - divisions + 1: i + 1)][1] - xyDistsFromCentre[i][1];
  }

  for (int i = 0; i < divisions; i++){
    Serial.print("x from last: ");
    Serial.print(xyDistsToNext[i][0]);
    Serial.print("      y from last: ");
    Serial.println(xyDistsToNext[i][1]);
  }

  

}
