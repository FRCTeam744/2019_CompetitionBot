/*-------------------------------------------------------------------*/

//Will be used for dividing constants and other tuners in the future.

/*-------------------------------------------------------------------*/

#pragma once
#include "math.h"

struct DriveControl {

    //CONSTANTS FOR DRIVE
    /*struct{
        const double NU_PER_REV = 4096.0;
        const double CIRCUMFERENCE_INCHES = RADIUS_INCHES * 2 * M_PI;
        const double INCHES_PER_REV = CIRCUMFERENCE_INCHES;
        const double NU_TO_FEET = (1.0 / NU_PER_REV) * INCHES_PER_REV * (1.0 / 12.0);
        const double FEET_TO_NU = 1.0 / NU_TO_FEET;
        const double SECONDS_TO_100MS = .10;
        const double CONVERT_100MS_TO_SECONDS = 0.1;

        const double MAX_TALON_OUTPUT = 1023.0; //instead of 0-100% power it is now 0-1023'%' where 1023 is the new 100%

        //Field Measurements
        const double TARGET_LOW_HEIGHT_INCHES = 28.84;
        const double TARGET_HIGH_HEIGHT_INCHES = 36.465;
    } consts;

    //TUNABLES FOR DRIVE
    struct{
        //Robot mechanical specifications & drivetrain variables
        const double RADIUS_INCHES = 3.0;
        const double TEST_PERCENT_OUTPUT = 0.25; //this is the percent output we used to test the feed forward gain
        const double MEASURED_SPEED_NU = 800.0;  //this is the result of the test above in

        //PID control limelight
        const double DESIRED_DISTANCE_INCHES = 65; //desired distance from target
        const double kP_DIST_FPS = -.05; //Estimate this value by seeing at what percent of the distance you want the speed to be in FPS
        const double kP_NU_PER_100MS = kP_DIST_FPS * FEET_TO_NU * SECONDS_TO_100ms; //Converted from FPS estimate above to NU/100ms that the talon can use
        const double LIMELIGHT_HEIGHT_INCHES = 20.625;
        const double LIMELIGHT_ANGLE = 0.0;
        const double kP_ANGLE = 0.02 * 13.0 * FEET_TO_NU * SECONDS_TO_100MS; //FOR ANGLE CORRECTION TODO

        const double MIN_COMMAND = 0.23;
    } tuners;
	*/
} drive;

struct ArmControl {
    struct{

    } consts;
	struct{

	} tuners;
} arm;

struct ClimbControl {
    struct{
        
    } consts;
    struct{

    } tuners;                        
} climber;