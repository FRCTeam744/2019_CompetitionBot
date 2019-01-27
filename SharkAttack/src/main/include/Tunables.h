/*----------------------------------------------------------------------------------*/

//Use this to tune the auto vaules, especially for the PID loop

/*----------------------------------------------------------------------------------*/

#pragma once

//Robot mechanical specifications & drivetrain variables
const double RADIUS_INCHES = 3.0;
const double NU_PER_REV = 4096.0;
const double TEST_PERCENT_OUTPUT = 0.25; //this is the percent output we used to test the feed forward gain
const double MEASURED_SPEED_NU = 800.0;  //

//PID control limelight
const double DESIRED_DISTANCE_INCHES = 65; //desired distance from target
const double kP_DIST = -.05 * FEET_TO_NU * SECONDS_TO_100MS;
const double LIMELIGHT_HEIGHT_INCHES = 20.625;
const double LIMELIGHT_ANGLE = 0.0;
const double kP_ANGLE = 0.02 * 13.0 * FEET_TO_NU * SECONDS_TO_100MS; //FOR ANGLE CORRECTION TODO

const double MIN_COMMAND = 0.23;

//Constants for the PID of talon
const double kP_SPEED = 0.1; //FOR SPEED CONTROL

const double kD_SPEED_RIGHT = kP_SPEED * 20.0 * 1.0; //USE 1.0 VALUE TO CALIBRATE
const double kD_SPEED_LEFT = kP_SPEED * 20.0 * 1.0;  //FOR SPEED CONTROL

const double kI_SPEED = kP_SPEED / 100.0;
const double kI_ZONE = (0.4 * 2.0) * FEET_TO_NU * CONVERT_100MS_TO_SECONDS;

double kFeedForwardGain = (TEST_PERCENT_OUTPUT * MAX_TALON_OUTPUT) / MEASURED_SPEED_NU;

const double talonTimeout = 10; //number of ms before the talon stops trying to configure a specific value
