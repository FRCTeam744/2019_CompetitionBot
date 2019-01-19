/*----------------------------------------------------------------------------------*/

//Use this to tune the auto vaules, especially for the PID loop

/*----------------------------------------------------------------------------------*/

#include <math.h>

double targetOffsetAngle_Horizontal;
double targetOffsetAngle_Vertical;
double targetArea;
double targetSkew;

const double RADIUS_INCHES = 3.0;
const double NU_PER_REV = 4096.0;
const double CIRCUMFERENCE_INCHES = RADIUS_INCHES * 2 * M_PI;
const double INCHES_PER_REV = CIRCUMFERENCE_INCHES;
const double NU_TO_FEET = (1.0/NU_PER_REV) * INCHES_PER_REV * (1.0/12.0);
const double FEET_TO_NU = 1.0/NU_TO_FEET;
const double NOF_SECOND_PER_100MS = .1;
const double NOF_100MS_PER_SECOND = 10;

double leftPower = 0.0;
double rightPower = 0.0;
double adjust = 0.0;

double kP = 0.018;
double minCommmand = 0.23;