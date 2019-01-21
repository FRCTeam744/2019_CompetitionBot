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
const double SECONDS_TO_100MS = 10;
const double CONVERT_100MS_TO_SECONDS = 0.1;
const double TEST_PERCENT_OUTPUT = 0.25; //this is the percent output we used to test the feed forward gain
const double MAX_TALON_OUTPUT = 1023.0;
const double MEASURED_SPEED_NU = 800.0;

double leftPower = 0.0;
double rightPower = 0.0;
double adjust = 0.0;

double talonTimeout = 10; //number of ms before the talon stops trying to configure a specific value

double minCommmand = 0.23;
const double kP_ANGLE = 0.018; //FOR ANGLE CORRECTION
const double kP_SPEED = 0.95; //FOR SPEED CONTROL
const double kD_SPEED_RIGHT = kP_SPEED * 20.0 * 3.0; //USE 1.0 VALUE TO CALIBRATE
const double kD_SPEED_LEFT = kP_SPEED * 20.0 * 3.0; //FOR SPEED CONTROL
const double kI_SPEED = kP_SPEED / 100.0;
const double kI_ZONE = (0.4 * 2.0) * FEET_TO_NU * CONVERT_100MS_TO_SECONDS;
double kFeedForwardGain = (TEST_PERCENT_OUTPUT * MAX_TALON_OUTPUT)/MEASURED_SPEED_NU;
