extern volatile bool isRunning;
extern const int leftSwitch, rightSwitch, topSwitch, bottomSwitch;

void CheckLimits();
void MoveDistanceV2(float xDist, float yDist, float Kp, float Ki, float Kd, float saturationLimit, bool checkLimits);
void TestMotorPowers();
