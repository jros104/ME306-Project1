#include "PID.h"
#include "MECHENG306BasicFunctions.h"

#define SIZE 2


#define scale 10

#define RATIO 80.0/78.0

void CircleV3(float radius, float period){
  PIDController motorPIDL(2,0,0.1);
  PIDController motorPIDR(2,0,0.1);

  float dt;
  float prevTime = millis();
  float T = 0;

  float initTargetH = DistanceToCount(radius * sin(-PI/2)) * RATIO;
  float initTargetV = DistanceToCount(radius * cos(-PI/2)) * RATIO;

  posL = initTargetH + initTargetV;
  posR = initTargetH - initTargetV;


  float rawValues[SIZE] = {0};
  float filteredValues[SIZE] = {0};

  analogWrite(motorLSpeedPin, 65);
  analogWrite(motorRSpeedPin, 65);

  digitalWrite(motorLDirPin, HIGH);
  digitalWrite(motorRDirPin, LOW);
  float controlEffortL, controlEffortR;
  float errorR, errorL;
  float targetL, targetR;
  float targetH, targetV;
  
  
  do{
    CheckLimits();
    dt = (millis() - prevTime)/1000.0;
    prevTime = millis();
    T += dt;

    targetH = DistanceToCount(radius * sin(((2*PI*T)/period)-PI/2)) * RATIO;    
    targetV = DistanceToCount(radius * cos(((2*PI*T)/period)-PI/2)) * RATIO;

    targetL = targetH + targetV;
    targetR = targetH - targetV;
    
    errorL = targetL - posL;
    errorR = targetR - posR;
   
    controlEffortL = motorPIDL.CalculateEffort(errorL,190,dt);
    controlEffortR = motorPIDR.CalculateEffort(errorR,190,dt);

    controlEffortL += 65 * sign(controlEffortL);
    controlEffortR += 65 * sign(controlEffortR);

    analogWrite(motorLSpeedPin, abs(controlEffortL));
    analogWrite(motorRSpeedPin, abs(controlEffortR));
    

    digitalWrite(motorLDirPin, sign(controlEffortL) == -1 ? 0 : 1);
    digitalWrite(motorRDirPin, sign(controlEffortR) == -1 ? 0 : 1);

    Serial.print(targetL);
    Serial.print("\t");
    Serial.print(controlEffortL);
    Serial.print("\t");
    Serial.println(posL);
    

  }while(T < period * 0.99);
  Stop();
}

#define TAU 0.05

void Filter(float dt, float* rawValues, float* filteredValues){
  float weight1 = dt / (dt + TAU);
  float weight2 = 1 - weight1;

  for (int i = 0; i < SIZE; i++){
    filteredValues[i] = rawValues[i] * weight1 + filteredValues[i] * weight2;
  }
}
