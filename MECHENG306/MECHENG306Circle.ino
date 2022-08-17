#include "PID.h"
#include "MECHENG306BasicFunctions.h"

void CircleV3(float radius, float period){
  PIDController motorPIDL(1,0,0);
  PIDController motorPIDR(1,0,0);

  float dt;
  float prevTime = millis();
  float T = 0;

  posR = 0;
  posL = 0;
  do{
    CheckLimits();
    dt = (millis() - prevTime)/1000.0;
    prevTime = millis();
    T += dt;
    
    float targetH = DistanceToCount(radius * sin((2*PI*T)/period));    
    float targetV = DistanceToCount(radius * cos((2*PI*T)/period));

    float targetL = targetH + targetV;
    float targetR = targetH - targetV;
    
    float errorL = targetL - posL;
    float errorR = targetR - posR;
   
    float controlEffortL = motorPIDL.CalculateEffort(errorL,190,dt) + 60;
    float controlEffortR = motorPIDR.CalculateEffort(errorR,190,dt) + 60;

    analogWrite(motorLSpeedPin, abs(controlEffortL));
    analogWrite(motorRSpeedPin, abs(controlEffortR));

    digitalWrite(motorLDirPin, sign(controlEffortL) == -1 ? 0 : 1);
    digitalWrite(motorRDirPin, sign(controlEffortR) == -1 ? 0 : 1);

    Serial.print(targetL);
    Serial.print("\t");

    Serial.print(targetR);
    Serial.print("\t");

    Serial.print(posL);
    Serial.print("\t");

    Serial.println(posR);
   
  
    
    delay(10);
    

  }while(1);

}
