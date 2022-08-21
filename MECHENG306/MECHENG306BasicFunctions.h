extern const int motorRDirPin, motorRSpeedPin, motorLDirPin, motorLSpeedPin;
extern const int encoderAL, encoderAR, encoderBL, encoderBR;
extern volatile int posR, posL;


float CountToDistance(float count);
float DistanceToCount(float distance);

float saturate(float value, float min, float max);
int sign(float value);
void Print(String string, float var);

void EncoderRInterrupt();
void EncoderLInterrupt();

void Horizontal(int motorSpeed, int horDir);
void Vertical(int motorSpeed, int verDir);
void LeftDiagonal(int motorSpeed, int verDir);
void RightDiagonal(int motorSpeed, int verDir);
void Stop(float uL, float uR);
