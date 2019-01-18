/*----------------------------------------------------------------------------------*/

//Use this to tune the auto vaules, especially for the PID loop

/*----------------------------------------------------------------------------------*/

double targetOffsetAngle_Horizontal;
double targetOffsetAngle_Vertical;
double targetArea;
double targetSkew;

double leftPower = 0.0;
double rightPower = 0.0;
double adjust = 0.0;

double kP = 0.018;
double minCommmand = 0.23;