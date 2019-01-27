/*----------------------------------------------------------------------------------*/

//Use this to tune the auto vaules, especially for the PID loop

/*----------------------------------------------------------------------------------*/
#include "Tunables.h"

const double CIRCUMFERENCE_INCHES = RADIUS_INCHES * 2 * M_PI;
const double INCHES_PER_REV = CIRCUMFERENCE_INCHES;
const double NU_TO_FEET = (1.0/NU_PER_REV) * INCHES_PER_REV * (1.0/12.0);
const double FEET_TO_NU = 1.0/NU_TO_FEET;
const double SECONDS_TO_100MS = .10;
const double CONVERT_100MS_TO_SECONDS = 0.1;

const double MAX_TALON_OUTPUT = 1023.0; //instead of 0-100% power it is now 0-1023'%' where 1023 is the new 100%

//Field Measurements
const double TARGET_LOW_HEIGHT_INCHES = 28.84;
const double TARGET_HIGH_HEIGHT_INCHES = 36.465;

