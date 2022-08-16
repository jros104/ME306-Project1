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
 * RECTANGLE WIDTH AND HEIGHT, CIRCLE DIAMETER
 */
volatile const int rectWidth = 5;
volatile const int rectHeight = 10;

volatile const int circDiam = 5;

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

  // Motor pin setup
  pinMode(motorRDirPin, OUTPUT);
  pinMode(motorLDirPin, OUTPUT);

  // Encoder pin setup
  pinMode(encoderAL, INPUT_PULLUP);
  pinMode(encoderAR, INPUT_PULLUP);
  pinMode(encoderBL, INPUT_PULLUP);
  pinMode(encoderBR, INPUT_PULLUP);
  
  Stop();

  // Allowing encoder interrupts to occur
  // Encoder A motorL/R interrupts
  cli();
  attachInterrupt(digitalPinToInterrupt(encoderAR), EncoderRInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderAL), EncoderLInterrupt, RISING);
 
  sei();

  FindHomeV2();
  MoveDistanceV2(100,0,0.5,0.01,0, 50, true);
  MoveDistanceV2(0,60,0.5,0.01,0, 50, true);
  isRunning = true;
//
//  delay(1000);
//  //MoveDistanceV2(30, 10, 1, 0.1, 0, 100, 200, true);
//
  CircleV2(40,32,1,0.1,0,50);
}

// +--------------------------------+
// |             LOOP               |
// +--------------------------------+
void loop() {
  
  //CheckLimits();
  isRunning = true; 
  posR = 0;
  posL = 0;

  Stop();

  // CircleV2(20, 4);
  // Drawing a square
//  MoveDistance(0,60,0.5,0.01,0, 100, true);
//  MoveDistance(60,0,0.5,0.01,0, 100, true);
//  MoveDistance(0,-60,0.5,0.01,0, 100, true);
//  MoveDistance(-60,0,0.5,0.01,0, 100, true);
  delay(1000);
}

// +--------------------------------+
// |           FIND HOME            |
// +--------------------------------+

void FindHomeV2(){
  Serial.println("Finding Bottom Switch");
  MoveDistanceV2(0, -100, 0.05, 0, 0, 50, true);
  delay(100);
  isRunning = true;
  
  Serial.println("Finding Left Switch");
  MoveDistanceV2(-100, 0, 0.05, 0, 0, 50, true);
  delay(100);
  isRunning = true;

  delay(1000);
  // Declaring origin
  xCoord = 0;
  yCoord = 0;
}
