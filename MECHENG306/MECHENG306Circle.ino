#include "PID.h"
#include "MECHENG306BasicFunctions.h"

void CircleV3(float radius, float period){
  PIDController motorPID(2,0.1,0.1);

  float dt;
  float prevTime = millis();
  float T = 0;

  posR = 0;
  posL = 0;
  do{
    dt = (millis() - prevTime)/1000.0;
    prevTime = millis();
    T += dt;
    float target = DistanceToCount(radius * sin((2*PI*T)/period));
    Serial.print(target);
    Serial.print(" \t ");
    float error = target - posR;
    Serial.print(posR);
    Serial.print(" \t ");
    float controlEffort = motorPID.CalculateEffort(error,190,dt);
    Serial.println(controlEffort);
    controlEffort += 60;
    analogWrite(motorRSpeedPin, abs(controlEffort));
    digitalWrite(motorRDirPin, sign(controlEffort) == -1 ? 1 : 0);
    delay(10);

  }while(1);

}
