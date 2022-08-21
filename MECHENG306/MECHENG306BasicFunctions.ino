// ####################################################################################################################
// ||         ____           _____ _____ _____   ______ _    _ _   _  _____ _______ _____ ____  _   _  _____         ||
// ||        |  _ \   /\    / ____|_   _/ ____| |  ____| |  | | \ | |/ ____|__   __|_   _/ __ \| \ | |/ ____|        ||
// ||        | |_) | /  \  | (___   | || |      | |__  | |  | |  \| | |       | |    | || |  | |  \| | (___          ||
// ||        |  _ < / /\ \  \___ \  | || |      |  __| | |  | | . ` | |       | |    | || |  | | . ` |\___ \         ||
// ||        | |_) / ____ \ ____) |_| || |____  | |    | |__| | |\  | |____   | |   _| || |__| | |\  |____) |        ||
// ||        |____/_/    \_\_____/|_____\_____| |_|     \____/|_| \_|\_____|  |_|  |_____\____/|_| \_|_____/         ||
// ||                                                                                                                ||
// ####################################################################################################################

// Motor 1 pins
const int motorRDirPin = 4, motorRSpeedPin = 5;
// Motor 2 pins
const int motorLDirPin = 7, motorLSpeedPin = 6;

// Declaring encoder a pins for left and right motors
const int encoderAL = 2, encoderAR = 3; 

// Declaring encoder b pins for left and right motors
const int encoderBL = 46, encoderBR = 44;

volatile int posL = 0, posR = 0;

#define CALCOUNT 2220.0
#define CALDIST 50.0
#define PULLEYDIAM (CALDIST / (PI * (CALCOUNT / 2064.0)))

// +--------------------------------+
// |       COUNT TO DISTANCE        |
// +--------------------------------+
float CountToDistance(float count) {
  return (PI * PULLEYDIAM * count)/2064.0; 
}

// +--------------------------------+
// |       DISTANCE TO COUNT        |
// +--------------------------------+
float DistanceToCount(float distance) {
  return distance != 0 ? (distance * 2064.0) / (PI * PULLEYDIAM): 0;
}

// +--------------------------------+
// |            SATURATE            |
// +--------------------------------+
float saturate(float value, float min, float max){
  return value > max ? max : value < min ? min : value;
}

// +--------------------------------+
// |             SIGN               |
// +--------------------------------+
// Returns the sign of a value:
// -1 for -ve ; 0 for 0 ; 1 for +ve
int sign(float value){
  return ((value > 0) - (value < 0));
}

// +--------------------------------+
// |            PRINT               |
// +--------------------------------+
void Print(String string, float var){
  Serial.print(string);
  Serial.println(var);
}

// +--------------------------------+
// |         RIGHT ENCODER          |
// +--------------------------------+
void EncoderRInterrupt() {
   int encoderBState = digitalRead(encoderBR);
   encoderBState > 0 ? posR-- : posR++;
}

// +--------------------------------+
// |          LEFT ENCODER          |
// +--------------------------------+
void EncoderLInterrupt() {
  int encoderBState = digitalRead(encoderBL);
  encoderBState > 0 ? posL--: posL++;
}

// +--------------------------------+
// |              STOP              |
// +--------------------------------+
void Stop(){
  analogWrite(motorRSpeedPin, 0);
  analogWrite(motorLSpeedPin, 0);
    
}
