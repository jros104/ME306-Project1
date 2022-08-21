#include "PID.h"
#include "MECHENG306BasicFunctions.h"

#define SIZE 2

void CircleV3(float radius, float period){
  PIDController motorPIDL(2,0.3,0);
  PIDController motorPIDR(2,0.3,0);

  float dt;
  float prevTime = millis();
  float T = 0;

  float initTargetH = DistanceToCount(radius * sin(-PI/2));
  float initTargetV = DistanceToCount(radius * cos(-PI/2));

  posL = initTargetH + initTargetV;
  posR = initTargetH - initTargetV;


  float rawValues[SIZE] = {0};
  float filteredValues[SIZE] = {0};
  
  do{
    CheckLimits();
    dt = (millis() - prevTime)/1000.0;
    prevTime = millis();
    T += dt;
    
    float targetH = DistanceToCount(radius * sin(((2*PI*T)/period)-PI/2));    
    float targetV = DistanceToCount(radius * cos(((2*PI*T)/period)-PI/2));

    float targetL = targetH + targetV;
    float targetR = targetH - targetV;
    
    float errorL = targetL - posL;
    float errorR = targetR - posR;
   
    float controlEffortL = motorPIDL.CalculateEffort(errorL,190,dt) + 60;
    float controlEffortR = motorPIDR.CalculateEffort(errorR,190,dt) + 60;

    rawValues[0] = controlEffortL;
    rawValues[1] = controlEffortR;
    Filter(dt, rawValues, filteredValues);


    analogWrite(motorLSpeedPin, abs(controlEffortL));
    analogWrite(motorRSpeedPin, abs(controlEffortR));
    
//    analogWrite(motorLSpeedPin, abs(filteredValues[0]));
//    analogWrite(motorRSpeedPin, abs(filteredValues[1]));

    digitalWrite(motorLDirPin, sign(controlEffortL) == -1 ? 0 : 1);
    digitalWrite(motorRDirPin, sign(controlEffortR) == -1 ? 0 : 1);
   
    delay(10);
    

  }while(T < period);

}

#define TAU 0.05

void Filter(float dt, float* rawValues, float* filteredValues){
  float weight1 = dt / (dt + TAU);
  float weight2 = 1 - weight1;

  for (int i = 0; i < SIZE; i++){
    filteredValues[i] = rawValues[i] * weight1 + filteredValues[i] * weight2;
  }
}
