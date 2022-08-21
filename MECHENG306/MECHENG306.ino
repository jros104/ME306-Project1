// ####################################################################################################################
// ||               __  __            _____  _   _        _____   _____  _____   _____  _____  _______               ||
// ||              |  \/  |    /\    |_   _|| \ | |      / ____| / ____||  __ \ |_   _||  __ \|__   __|              ||
// ||              | \  / |   /  \     | |  |  \| |     | (___  | |     | |__) |  | |  | |__) |  | |                 ||
// ||              | |\/| |  / /\ \    | |  | . ` |      \___ \ | |     |  _  /   | |  |  ___/   | |                 ||
// ||              | |  | | / ____ \  _| |_ | |\  |      ____) || |____ | | \ \  _| |_ | |       | |                 ||
// ||              |_|  |_|/_/    \_\|_____||_| \_|     |_____/  \_____||_|  \_\|_____||_|       |_|                 ||
// ||                                                                                                                ||
// ####################################################################################################################

#include "MECHENG306BasicFunctions.h"
#include "MECHENG306AdvancedFunctions.h"
#include "PID.h"

// ######################################################### ARDUINO CODE SETUP #########################################################

volatile bool foundHome = false;

// Declaring position on board
volatile int xCoord, yCoord;

/*
   RECTANGLE WIDTH AND HEIGHT, CIRCLE DIAMETER
*/
volatile const int rectWidth = 5;
volatile const int rectHeight = 10;

volatile const int circDiam = 5;

const int StartPin = 52;

// +--------------------------------+
// |             SETUP              |
// +--------------------------------+
void setup() {
  isRunning = true;

  Serial.begin(9600);
  // Limit Switch setups
  pinMode(leftSwitch, INPUT_PULLUP);
  pinMode(rightSwitch, INPUT_PULLUP);
  pinMode(topSwitch, INPUT_PULLUP);
  pinMode(bottomSwitch, INPUT_PULLUP);
  pinMode(StartPin, INPUT_PULLUP);

  // Motor pin setup
  pinMode(motorRDirPin, OUTPUT);
  pinMode(motorLDirPin, OUTPUT);

  // Encoder pin setup
  pinMode(encoderAL, INPUT_PULLUP);
  pinMode(encoderAR, INPUT_PULLUP);
  pinMode(encoderBL, INPUT_PULLUP);
  pinMode(encoderBR, INPUT_PULLUP);

  // Allowing encoder interrupts to occur
  // Encoder A motorL/R interrupts
  cli();
  attachInterrupt(digitalPinToInterrupt(encoderAR), EncoderRInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderAL), EncoderLInterrupt, RISING);

  sei();

  FindHomeV2();
  delay(1000);
  isRunning = true;

}

// +--------------------------------+
// |             LOOP               |
// +--------------------------------+

bool canStart = false;

void loop() {

  if (canStart == false && digitalRead(rightSwitch)){
    canStart = true;
  }

  if (canStart){
    MoveDistanceV2(7, 7, 0.9, 0.005, 0.01, 190, false);
    delay(300);  
      
    DrawRectangle(60 * (60.0/58.6), 90 * (90.0/88), 0.9, 0.005, 0.1, 170, 300);
    delay(300);
    
    MoveDistanceV2(100, 50, 0.9, 0.005, 0, 190, false);
    delay(300);   

    CircleV3(40,11);
    delay(300);   
    canStart = false;
  }
  delay(10);
}

void DrawRectangle(float hor, float ver, float Kp, float Ki, float Kd, int sat, int delayAmount){
  MoveDistanceV2(hor, 0, Kp, Ki, Kd, sat, true);
  delay(delayAmount);
  MoveDistanceV2(0, ver, Kp, Ki, Kd, sat, true);
  delay(delayAmount);
  MoveDistanceV2(-hor, 0, Kp, Ki, Kd, sat, true);
  delay(delayAmount);
  MoveDistanceV2(0, -ver, Kp, Ki, Kd, sat, true);
}

// +--------------------------------+
// |           FIND HOME            |
// +--------------------------------+

void FindHomeV2() {
  //Serial.println("Finding Bottom Switch");
  MoveDistanceV2(0, -250, 0.7, 0.005, 0, 50, true);
  delay(300);
  isRunning = true;

  //Serial.println("Finding Left Switch");
  MoveDistanceV2(-250, 0, 0.7, 0.005, 0, 50, true);
  delay(300);
  isRunning = true;

  delay(1000);
  // Declaring origin
  xCoord = 0;
  yCoord = 0;
}
