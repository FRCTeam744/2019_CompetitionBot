

#pragma once

#include "math.h"
#include <string>
#include "networktables/NetworkTableInstance.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include <ctre/Phoenix.h>
#include "frc/DoubleSolenoid.h"
#include "frc/Solenoid.h"
#include <iostream>
//#include <frc/shuffleboard/Shuffleboard.h>

#include "ShuffleManager.h"

class Drivetrain
{

  public:
	static Drivetrain *GetInstance();

	void AutoDrive(bool wantLimelight, double leftTank, double rightTank, bool isBallMode, bool wantToNotMove);
	void PrintDriveShuffleInfo();
	void TankDrive(double leftValue, double rightValue);
	void LimelightSet(std::tuple<bool, std::string, double>);
	double LimelightGet(std::string key);
	void CheckSwitchGears(bool isHighGear);
	void AutoDriveForward(bool isBut, bool isVelocityControl);
	void SetDesiredLLDistances(double xDesiredInches, double zDesiredInches);
	void SetIsFrontLL(bool isFront);
	void AutoDriveLL(bool wantLimelight, double leftTank, double rightTank);
	// void PutOnShuffleboard();
	void SetSlopeInterceptForAngleCalc(double slope, double intercept);
	void SetCrosshairAngle(double crosshairAngle);
	void SetPipelineNumber(int pipelineNumber);

	//measured crosshair angles
	//FRONT
	const double CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_FRONT = 0.5; 
	const double CROSSHAIR_TY_ANGLE_MIDDLE_HATCH_FRONT   = -3.6; 
	const double CROSSHAIR_TY_ANGLE_BALL_LOW_HIGH_FRONT = 13.55; 
	const double CROSSHAIR_TY_ANGLE_BALL_MIDDLE_FRONT = 0;

	//BACK
	const double CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_BACK = 9.10; 
	const double CROSSHAIR_TY_ANGLE_MIDDLE_HATCH_BACK   = 4.04; 
	const double CROSSHAIR_TY_ANGLE_BALL_LOW_HIGH_BACK = 0; 
	const double CROSSHAIR_TY_ANGLE_BALL_MIDDLE_BACK = 0;

	//HATCH FRONT
	const double SLOPE_LOW_HIGH_HATCH_FRONT = 0.2608; 
	const double INTERCEPT_LOW_HIGH_HATCH_FRONT = -8.206;
	const double SLOPE_MIDDLE_HATCH_FRONT = 0.15; 
	const double INTERCEPT_MIDDLE_HATCH_FRONT = -3.31;

	//HATCH BACK
	const double SLOPE_LOW_HIGH_HATCH_BACK = -0.35; 
	const double INTERCEPT_LOW_HIGH_HATCH_BACK = 7.51;
	const double SLOPE_MIDDLE_HATCH_BACK = -0.344; 
	const double INTERCEPT_MIDDLE_HATCH_BACK = 14.86;

	//BALL FRONT
	const double SLOPE_LOW_HIGH_BALL_FRONT = 2.6758579; 
	const double INTERCEPT_LOW_HIGH_BALL_FRONT = -26.96;
	const double SLOPE_MID_BALL_FRONT = 0.15; 
	const double INTERCEPT_MID_BALL_FRONT = -3.31;

	//BALL BACK
	const double SLOPE_LOW_HIGH_BALL_BACK = -0.55; 
	const double INTERCEPT_LOW_HIGH_BALL_BACK = 4.7;
	const double SLOPE_MID_BALL_BACK = -0.344; 
	const double INTERCEPT_MID_BALL_BACK = 14.86;

  private:
	static Drivetrain *s_instance;

	Drivetrain();
	void IsTargetNotAcquired(double leftTank, double rightTank);

	//Private Instance Objects

	bool isInitialized = false;

	//left back and right back encoder used for drivetrain
	//left front and right front encoder used for arm and wrist respectively
	TalonSRX *leftFront;
	TalonSRX *leftMid;
	TalonSRX *leftBack;
	TalonSRX *rightFront;
	TalonSRX *rightMid;
	TalonSRX *rightBack;

	frc::Solenoid *gearShifter;

	std::shared_ptr<NetworkTable> limelightFront;
	std::shared_ptr<NetworkTable> limelightBack;

	//CAN Talon IDs for each of the drivetrain motors
	const int RIGHT_FRONT_ID = 23;
	const int RIGHT_MID_ID = 25;
	const int RIGHT_BACK_ID = 27;
	const int LEFT_FRONT_ID = 22;
	const int LEFT_MID_ID = 24;
	const int LEFT_BACK_ID = 26;

	//DoubleSolenoid gearShirter forward and reverse channels
	const int LOW_GEAR = 0;
	const int HIGH_GEAR = 1;

	double leftDashboardSpeed = 0.0;
	double rightDashboardSpeed = 0.0;
	double leftPower = 0.0;
	double rightPower = 0.0;
	double desiredRightFPS = 3.0;
	double desiredLeftFPS = 3.0;

	double realRightSpeedNUPer100ms = 0.0;
	double realLeftSpeedNUPer100ms = 0.0;

	double targetOffsetAngle_Horizontal;
	double targetOffsetAngle_Vertical;
	double targetArea;
	double targetSkew;
	std::vector<double> limelightPose;
	std::vector<double> rawXBuffer;
	std::vector<double> rawYBuffer;
	std::vector<double> rawZBuffer;
	std::vector<double> rawRollBuffer;
	std::vector<double> rawPitchBuffer;
	std::vector<double> rawYawBuffer;
	double prevX;
	double prevY;
	double prevZ;
	double prevRoll;
	double prevPitch;
	double prevYaw;
	const double alpha = 0.02 / (0.06 + 0.02);

	double adjust = 0.0;

	double currentDistanceInches = 0.0;

	double xDesiredInches;
	double zDesiredInches;
	bool isInAutoDrive;
	bool isInLLDrive;
	bool isFront;
	double slopeForAngleCalc;
    double interceptForAngleCalc;
    double crosshairAngle;
	int pipelineNumber;

	//CONSTANTS FOR DRIVE
	const double NU_PER_REV = 4096.0;
	const double CIRCUMFERENCE_INCHES = 4.08 * M_PI;
	const double INCHES_PER_REV = CIRCUMFERENCE_INCHES;
	const double NU_TO_FEET = (1.0 / NU_PER_REV) * INCHES_PER_REV * (1.0 / 12.0);
	const double FEET_TO_NU = 1.0 / NU_TO_FEET;
	const double SECONDS_TO_100MS = 10;
	const double CONVERT_100MS_TO_SECONDS = 0.1;

	const double MAX_TALON_OUTPUT = 1023.0; //instead of 0-100% power it is now 0-1023'%' where 1023 is the new 100%

	//Field Measurements
	const double TARGET_LOW_HEIGHT_INCHES = 28.84;
	const double TARGET_HIGH_HEIGHT_INCHES = 36.465;

	//TUNABLES FOR DRIVE
	//Robot mechanical specifications & drivetrain variables
	//const double RADIUS_INCHES = 3.0;
	const double TEST_PERCENT_OUTPUT = 0.5;  //this is the percent output we used to test the feed forward gain
	const double MEASURED_SPEED_NU = 2300.0; //this is the result of the test above in NU/100ms

	//PID control limelight
	const double kP_THETA_DESIRED = -2;
	const double LL_FRONT_THETA_OFFSET = 15;
	const double LL_FRONT_X_OFFSET = 12.5;
	const double kP_FORWARD = 0; //.2 / 10;
	const double kP_THETA = 0; //.2 / 30;
	const double START_FILTERING_JUMPS = 15;
	bool isTargetAcquired;
	
    int counter = 0;


	const double DESIRED_DISTANCE_INCHES = 22;									//desired distance from target
	const double kP_DIST_FPS = -.20;											//Estimate this value by seeing at what percent of the distance you want the speed to be in FPS
	const double kP_NU_PER_100MS = kP_DIST_FPS * FEET_TO_NU * SECONDS_TO_100MS; //Converted from FPS estimate above to NU/100ms that the talon can use
	const double LL_DISTANCE_PER_5FEET_FRONT = 100.0;
	const double LL_DISTANCE_PER_5FEET_BACK = 100.0;
	const double kP_DIST_FPS_BACK = -kP_DIST_FPS * (LL_DISTANCE_PER_5FEET_FRONT/LL_DISTANCE_PER_5FEET_BACK);
	const double LIMELIGHT_HEIGHT_INCHES = 45;
	const double LIMELIGHT_ANGLE_FRONT = 26.0;
	const double LIMELIGHT_ANGLE_BACK = 25.0;
	// const double CROSSHAIR_ANGLE = 6.5; //17.3

	const double kP_ANGLE = 0.08; //FOR ANGLE CORRECTION TODO

	const double MIN_COMMAND = 0.23;

	//Constants for the PID of talon
	const double kP_SPEED = 0.2; //FOR SPEED CONTROL
	const double kD_SPEED_RIGHT = kP_SPEED * 20.0 * 1.0; //USE 1.0 VALUE TO CALIBRATE
	const double kD_SPEED_LEFT = kP_SPEED * 20.0 * 1.0;  //FOR SPEED CONTROL

	const double kI_SPEED = kP_SPEED / 100.0;
	const double kI_ZONE = (.5 * 2.0) * FEET_TO_NU * CONVERT_100MS_TO_SECONDS;

	double kFeedForwardGain = (TEST_PERCENT_OUTPUT * MAX_TALON_OUTPUT) / MEASURED_SPEED_NU;

	const double talonTimeout = 10; //number of ms before the talon stops trying to configure a specific value

	//Talon Loop Ramp Rates in seconds
	const double talonRampRate = 0.25;

	const double LL_MAX_FEET_PER_SEC = 3.5;
};
