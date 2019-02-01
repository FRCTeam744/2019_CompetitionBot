#pragma once

#include "math.h"
#include <ctre/Phoenix.h>

class Drivetrain
{
  public:
	void DrivetrainInit();
	void Periodic();
	void Limelight();

  private:
	//Private Instance Objects
	TalonSRX *leftFront;
	TalonSRX *leftMid;
	TalonSRX *leftBack;
	TalonSRX *rightFront;
	TalonSRX *rightMid;
	TalonSRX *rightBack;

	

	//CAN Talon IDs for each of the drivetrain motors
	const int rightFrontID = 22;
	const int rightMidID = 24;
	const int rightBackID = 26;
	const int leftFrontID = 23;
	const int leftMidID = 25;
	const int leftBackID = 27;


	double leftDashboardSpeed = 0.0;
	double rightDashboardSpeed = 0.0;
	double leftPower = 0.0;
	double rightPower = 0.0;
	double desiredRightFPS = 0.0;
	double desiredLeftFPS = 0.0;

	double realRightSpeedNUPer100ms = 0.0;
	double realLeftSpeedNUPer100ms = 0.0;

	double targetOffsetAngle_Horizontal;
	double targetOffsetAngle_Vertical;
	double targetArea;
	double targetSkew;

	double adjust = 0.0;

	double currentDistanceInches = 0.0;

	//CONSTANTS FOR DRIVE
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

	//TUNABLES FOR DRIVE
	//Robot mechanical specifications & drivetrain variables
	const double RADIUS_INCHES = 3.0;
	const double TEST_PERCENT_OUTPUT = 0.25; //this is the percent output we used to test the feed forward gain
	const double MEASURED_SPEED_NU = 800.0;  //this is the result of the test above in

	//PID control limelight
	const double DESIRED_DISTANCE_INCHES = 65;									//desired distance from target
	const double kP_DIST_FPS = -.05;											//Estimate this value by seeing at what percent of the distance you want the speed to be in FPS
	const double kP_NU_PER_100MS = kP_DIST_FPS * FEET_TO_NU * SECONDS_TO_100MS; //Converted from FPS estimate above to NU/100ms that the talon can use
	const double LIMELIGHT_HEIGHT_INCHES = 20.625;
	const double LIMELIGHT_ANGLE = 0.0;
	const double kP_ANGLE = 0.02 * 13.0 * FEET_TO_NU * SECONDS_TO_100MS; //FOR ANGLE CORRECTION TODO

	const double MIN_COMMAND = 0.23;
}