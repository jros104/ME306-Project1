// ################################################################################################################################################
// ||            _____ __      __       _   _   _____  ______  _____         ______  _    _  _   _   _____  _______  _____  ____   _   _   _____ ||
// ||     /\    |  __ \\ \    / //\    | \ | | / ____||  ____||  __ \       |  ____|| |  | || \ | | / ____||__   __||_   _|/ __ \ | \ | | / ____|||
// ||    /  \   | |  | |\ \  / //  \   |  \| || |     | |__   | |  | |      | |__   | |  | ||  \| || |        | |     | | | |  | ||  \| || (___  ||
// ||   / /\ \  | |  | | \ \/ // /\ \  | . ` || |     |  __|  | |  | |      |  __|  | |  | || . ` || |        | |     | | | |  | || . ` | \___ \ ||
// ||  / ____ \ | |__| |  \  // ____ \ | |\  || |____ | |____ | |__| |      | |     | |__| || |\  || |____    | |    _| |_| |__| || |\  | ____) |||
// || /_/    \_\|_____/    \//_/    \_\|_| \_| \_____||______||_____/       |_|      \____/ |_| \_| \_____|   |_|   |_____|\____/ |_| \_||_____/ ||
// ||                                                                                                                                            ||
// ################################################################################################################################################

#include "MECHENG306BasicFunctions.h"
#include "PID.h"

#define ACCELERATION_TIME 300
#define EXIT_THRESHOLD 5


volatile bool isRunning;

// Limit switch pins
const int leftSwitch = 18, rightSwitch = 19, topSwitch = 21, bottomSwitch = 20;

// +--------------------------------+
// |          CHECK LIMITS          |
// +--------------------------------+
void CheckLimits(){
  if (digitalRead(topSwitch) || digitalRead(bottomSwitch) || digitalRead(rightSwitch) || digitalRead(leftSwitch)){
    Stop();
    delay(100);

    // Move the head away from the switch
    if(digitalRead(topSwitch)) { 
      Serial.println("Top Switch");
      MoveDistanceV2(0, -13, 0.2, 0, 0, 50, false); 
      
    }
    if(digitalRead(bottomSwitch)) { 
      Serial.println("Bottom Switch");
      MoveDistanceV2(0, 13, 0.2, 0, 0, 50, false); 
      
    }
    if(digitalRead(rightSwitch)) { 
      Serial.println("Right Switch");
      MoveDistanceV2(-13, 0, 0.2, 0, 0, 50, false); 
    }
    if(digitalRead(leftSwitch)) { 
      Serial.println("Left Switch");
      MoveDistanceV2(13, 0, 0.2, 0, 0, 50, false); 
    }
    isRunning = false; 
  }
}


// +--------------------------------+
// |        MOVE DISTANCE V2        |
// +--------------------------------+
void MoveDistanceV2(float xDist, float yDist, float Kp, float Ki, float Kd, float saturationLimit, bool checkLimits){
   
    // Create the PID Controllers with the assigned values
    PIDController motorPID(Kp, Ki, Kd);         // Motor base power PID controller
    PIDController motorDiffPID(1,0,0);        // Encoder difference PID controller
   
    posL = 0;
    posR = 0;

    bool canExit = false;

    float distL = xDist + yDist;
    int countTargetL = DistanceToCount(distL);

    Print("Left target: ", countTargetL);
    

    float distR = xDist - yDist;
    int countTargetR = DistanceToCount(distR);

    Print("Right target: ", countTargetR);

    float prevTime = millis();

    float controlEffortL, controlEffortR;
    float controlEffortDiff;
    
    float encRatio;

    float errorL, errorR;
    float errorEnc;

    bool rightDrive = false;


    do{
       if(checkLimits) CheckLimits();
       float dt = (millis() - prevTime) / 1000;
       prevTime = millis();

       if (abs(countTargetL) > abs(countTargetR)){
          //Serial.println("Left Drive");
          rightDrive = false;
          errorL = countTargetL - posL;
          controlEffortL = motorPID.CalculateEffort(errorL, saturationLimit, dt);

          if (countTargetR != 0){
            encRatio =  (float)countTargetL / (float)countTargetR;
            controlEffortR = (abs(controlEffortL) / abs(encRatio)) * sign(countTargetR);
  
            errorEnc = (posL / encRatio) - posR; 
            controlEffortDiff = motorDiffPID.CalculateEffort(errorEnc, 50, dt);
  
            controlEffortR += controlEffortL != 0 ? controlEffortDiff : 0;
          }else{
            controlEffortR = 0;
          }
          
       }else{
          //Serial.println("Right Drive");
          rightDrive = true;
          errorR = countTargetR - posR;
          controlEffortR = motorPID.CalculateEffort(errorR, saturationLimit, dt);

          if (countTargetL != 0){
            encRatio =  (float)countTargetR / (float)countTargetL;
            controlEffortL = (abs(controlEffortR) / abs(encRatio)) * sign(countTargetL);
  
            errorEnc = (posR / encRatio) - posL; 
            controlEffortDiff = motorDiffPID.CalculateEffort(errorEnc, 50, dt);
  
            controlEffortL += controlEffortR != 0 ? controlEffortDiff : 0;
          }else{
            controlEffortL = 0;
          }
       }

       controlEffortR += 60 * sign(controlEffortR);
       controlEffortL += 60 * sign(controlEffortL);

//      rightDrive ? Print("Right error: ", errorR) : Print("Left error: ", errorL);
//      Print("Right effort: ", controlEffortR);
//      Print("Left effort: ", controlEffortL);
//      Print("Encoder Ratio: ", encRatio);
//      Print("Encoder Error: ", errorEnc);
//      Print("Diff Effort: ", controlEffortDiff);

       analogWrite(motorLSpeedPin, saturate(abs(controlEffortL), 60, 255));
       analogWrite(motorRSpeedPin, saturate(abs(controlEffortR), 60, 255));

       digitalWrite(motorLDirPin, sign(controlEffortL) == -1 ? 0 : 1);
       digitalWrite(motorRDirPin, sign(controlEffortR) == -1 ? 0 : 1);

              
       if ((!rightDrive && abs(errorL) <= 5) || (rightDrive && abs(errorR) <= 5)){
        Print("Error on exit: ", rightDrive ? errorR : errorL);
        canExit = true;
       }
       delay(1);
    }while(!canExit && isRunning);
    Stop();
}

// +--------------------------------+
// |          CIRCLE V2             |
// +--------------------------------+
void CircleV2(float radius, int divisions, float kp, float ki, float kd, float saturationLimit){
  float xyDistsFromCentre[divisions][2] = {0};
  float angle = (PI * 2.0) / (float)divisions;

  for (int i = 0; i < divisions; i++){
    xyDistsFromCentre[i][0] = (radius * cos(angle*i));
    xyDistsFromCentre[i][1] = (radius * sin(angle*i));
  }

  float xyDistsToNext[divisions][2] = {0};

  for (int i = 0; i < divisions; i++){
    xyDistsToNext[i][0] = xyDistsFromCentre[(i + 1 >= divisions ? i - divisions + 1: i + 1)][0] - xyDistsFromCentre[i][0];
    xyDistsToNext[i][1] = xyDistsFromCentre[(i + 1 >= divisions ? i - divisions + 1: i + 1)][1] - xyDistsFromCentre[i][1];
  }

  for (int i = 0; i < divisions; i++){
    isRunning = true;
    MoveDistanceV2(xyDistsToNext[i][0],xyDistsToNext[i][1], kp, ki, kd, saturationLimit, true);
  }

}
