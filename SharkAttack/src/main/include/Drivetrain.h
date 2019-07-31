

#pragma once

#include "math.h"
#include <string>
#include "networktables/NetworkTableInstance.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include <ctre/Phoenix.h>
#include "frc/DoubleSolenoid.h"
#include "frc/Solenoid.h"
#include <iostream>
#include "frc/Timer.h"
//#include <frc/shuffleboard/Shuffleboard.h>

#include "ShuffleManager.h"

// #include "frc/Notifier.h"
#include <pathfinder-frc.h>
#include "AHRS.h"

class Drivetrain
{

  public:
	static Drivetrain *GetInstance();

	/**
    @brief Appends the path to the file name to get the full path to the trajectory file.
    @param name The file name of the trajectory file.
	@return The full path of the trajectory file

    Appends the path to the file name to get the full path to the trajectory file.
	*/
	std::string get_trajectory_file(std::string name);
	
	/**
    @brief Takes the trajectory in the file with the name "name" and sets it to the traj_out variable.
    @param name The file name of the trajectory file.
	@param trajout The buffer that stores the current trajectory.
	@return len Returns the length of the trajectory, in discrete timesteps.

	Takes the trajectory in the file with the name "name" and sets it to the traj_out variable.	
	*/
	int get_trajectory(std::string name, Segment *traj_out);
	
	/**
	 * @brief Initialization of certain variables for the Drivetrain.
	 * 
	 * Initializes the DGain timer for LL tracking, and the prevT value.
	 */
	void RobotInit();

	/**
	 * @brief Initialization of variables for autonomous.
	 * 
	 * Zeros the yaw angle for auto driving purposes.
	 */
	void AutonomousInit();

	
	/**
	 * @brief Initialize the trajectory buffers for the left and right sides.
	 * @param pathName the name of the path file to initialize.
	 * 
	 * Takes the trajectories from the files and sets them to the correct buffers.
	 */
	void FollowPathInit(std::string pathName);

	/**
	 * @brief Executes the following of the path loaded into the trajectory buffers
	 * @param isReverse if true, follow the path in reverse, if false, follow forward
	 * @return returns true when finished with the trajectory.
	 * 
	 * Trajectory following, flips and reverses the sign of the velocity inputs if reverse.
	 * Angle adjustment based on NavX gyroscope feedback.
	 * Handles cutting off the trajectory early to save time in sandstorm.
	 */
	bool FollowPath(bool isReverse);

	/**
	 * @brief Drive by LL to track the vision target.
	 * @param wantLimelight whehter or not limelight tracking is desired right now
	 * @param leftTank the value of the left drive joystick, for driving when no target is in view
	 * @param rightTank the value of the right drive joystick, for driving when no target is in view
	 * @param isBallMode whether or not the arm is in ball mode
	 * @param wantToNotMove true if you want to run the limelight code, without moving the drivetrain.
	 * @return returns true when finished with tracking the vision target, false otherwise
	 * 
	 * Drives to the correct spot relative to the vision target.
	 */
	bool AutoDrive(bool wantLimelight, double leftTank, double rightTank, bool isBallMode, bool wantToNotMove);
	
	/**
	 * @brief Print various Drivetrain info to shuffle board
	 * 
	 * Print various Drivetrain info to shuffle board
	 */
	void PrintDriveShuffleInfo();

	/**
	 * @brief Method for executing tank drive.
	 * @param leftValue the value to input to the left side of the drivetrian
	 * @param rightValue the value to input to the right side of the dirvetrain
	 * 
	 * Set the drivetrain to drive by tankdrive when it is not auto driving or driving via limelight. 
	 */ 
	void TankDrive(double leftValue, double rightValue);

	/**
	 * @brief Sets values for properties of the front limelight
	 * @param data tuple containing: whether or not to set the property, the key of the property being set, and the value to set
	 * 
	 * Sets values for properties of the front limelight.
	 */ 
	void LimelightSet(std::tuple<bool, std::string, double>);

	/**
	 * @brief Gets the value of a certain property for the front limelight.
	 * @param key the key of the property to get the current value of.
	 * 
	 * Gets the value of a certain property for the front limelight.
	 */ 
	double LimelightGet(std::string key);

	/**
	 * @brief Sets the gear shifter solenoid to high or low gear.
	 * @param isHighGear true if setting to high gear, false if setting to low gear
	 * 
	 * Sets the gear shifter solenoid to high or low gear.
	 */ 
	void SetGearShifter(bool isHighGear);

	/**
	 * @brief Either drives at a given percent output or at a given velocity in the direction of the arm.
	 * @param isTestPercentOut true if wanting to drive with a constant percent output.
	 * @param isVelocityControl true if wanting to drive with a certain velocity in the same direction as the arm side.
	 * 
	 * Either drives at a given percent output or at a given velocity in the direction of the arm.
	 * TEST_PERCENT_OUT contains the percent output to drive at when in that mode.
	 * AUTO_VELOCITY_CONTROL_DRIVE_SPEED contains the speed to drive at when in velocity mode. 
	 */ 
	void AutoDriveForward(bool isTestPercentOut, bool isVelocityControl);
	
	/**
	 * @brief Either drives at a given percent output or at a given velocity in the opposite direction of the arm.
	 * @param isTestPercentOut true if wanting to drive with a constant percent output.
	 * @param isVelocityControl true if wanting to drive with a certain velocity in the opposite direction as the arm side.
	 * 
	 * Either drives at a given percent output or at a given velocity in the opposite direction of the arm.
	 * TEST_PERCENT_OUT contains the percent output to drive at when in that mode.
	 * AUTO_VELOCITY_CONTROL_DRIVE_SPEED contains the speed to drive at when in velocity mode. 
	 */ 
	void AutoDriveBackwards(bool isTestPercentOut, bool isVelocityControl);
	
	//The following are setter methods for LL tracking parameters. These change based on the position of the arm.
	/**
	 * @brief Set desired x and z distance setpoints in calculated "inches" for LL tracking. Changes based on arm position.
	 * @param xDesiredInches the desired x inches (or side to side) inches to end at when LL tracking
	 * @param zDesiredInches the desired z inches (or how far away) inches to end at when LL tracking
	 * 
	 * Set desired x and z distance setpoints in calculated "inches." These change based on the arm position.
	 * In actuality, the xDesiredInches is more of an angle measurement, but this value is always 0 anyways.
	 */ 
	void SetDesiredLLDistances(double xDesiredInches, double zDesiredInches);
	
	/**
	 * @brief Set whether or not the arm is in the front, used for LL tracking. Changes based on arm position.
	 * @param isFront true if the arm is in the front of the robot, false if it's in the back
	 * 
	 * whether or not the arm is in the front, used for LL tracking. Changes based on arm position.
	 * A position of 0 defaults to the front.
	 */ 
	void SetIsFrontLL(bool isFront);

	/**
	 * @brief Set slope and intercept for line to follow when LL tracking. Changes based on arm position.
	 * @param slope the calculated slope of the line, dy/dx where dy and dx are in units of angle from the crosshair
	 * @param intercept the dy value when dx=0
	 * 
	 * Because the Limelights are mounted offset from the arm, and at an obscure angle, 
	 * the tracking of the target moves left to right as the robot approaches the target.
	 * The slope and intercept are used to describe the line that the target follows, 
	 * as it moves closer to the target while lined up to score.
	 * While the robot drives forward, the setpoitn angle is being constantly adjusted to 
	 * follow this line. 
	 * 
	 * While written in a way that line (slope/intercept) could be set for each individual arm position,
	 * it ended up only needing a different slope/intercept for the front and back limelight.
	 */ 
	void SetSlopeInterceptForAngleCalc(double slope, double intercept);

	/**
	 * @brief Set crosshair angle for use in calculating distance from the target. Changes based on arm position.
	 * @param crosshairAngle the vertical deviation of the corsshair, in units of angle, from the center of the image 
	 * 
	 * The vertical (dy) deviation of the crosshair from the center point of the image for the pipeline in use. 
	 * This is used in the calculation of the horizontal distance of the robot from the target.
	 *  
	 * While written in a way that line crosshair angle could be set for each individual arm position,
	 * it ended up only needing a different crosshair angle for the front and back limelight, as there was 
	 * only 1 pipeline in use for tracking per side. 
	 */ 
	void SetCrosshairAngle(double crosshairAngle);
	
	/**
	 * @brief Sets the pipeline for use in LL tracking. Changes based on arm position.
	 * @param pipelineNumber the number assigned to the pipeline in the LL GUI. 
	 * 
	 * Sets the pipeline for use in LL tracking.
	 * 
	 * Only 1 pipeline ended up being used, and due to a strange lag in intaking light when switching pipelines,
	 * eventually we set the LLs to only stay in their tracking pipelines. And only 1 pipeline was needed on each side. 
	 */
	void SetPipelineNumber(int pipelineNumber);

	/**
	 * @brief Stops drivetrain motors.
	 * 
	 * Sets the percent output of all drivetrain motors to 0.
	 */ 
	void StopMotors();

	//measured crosshair angles
	//FRONT
	const double CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_FRONT = -13.11; // 10.9;
	const double CROSSHAIR_TY_ANGLE_MIDDLE_HATCH_FRONT = CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_FRONT;
	const double CROSSHAIR_TY_ANGLE_BALL_CARGO_SHIP_FRONT = CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_FRONT;
	// const double CROSSHAIR_TY_ANGLE_BALL_LOW_HIGH_FRONT = 13.55;
	// const double CROSSHAIR_TY_ANGLE_BALL_MIDDLE_FRONT = 0;

	//BACK
	const double CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_BACK = -5.95;
	const double CROSSHAIR_TY_ANGLE_MIDDLE_HATCH_BACK = CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_BACK;
	const double CROSSHAIR_TY_ANGLE_BALL_CARGO_SHIP_BACK = CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_BACK;
	// const double CROSSHAIR_TY_ANGLE_BALL_LOW_HIGH_BACK = 0;
	// const double CROSSHAIR_TY_ANGLE_BALL_MIDDLE_BACK = 0;

	//FRONT
	const double SLOPE_LOW_HIGH_HATCH_FRONT = 0.69842;//0.6786874594; //0.790512334;
	const double INTERCEPT_LOW_HIGH_HATCH_FRONT = 0;
	const double SLOPE_MIDDLE_HATCH_FRONT = SLOPE_LOW_HIGH_HATCH_FRONT;
	const double INTERCEPT_MIDDLE_HATCH_FRONT = INTERCEPT_LOW_HIGH_HATCH_FRONT;
	const double SLOPE_CARGO_SHIP_BALL_FRONT = SLOPE_LOW_HIGH_HATCH_FRONT;
	const double INTERCEPT_CARGO_SHIP_BALL_FRONT = INTERCEPT_LOW_HIGH_HATCH_FRONT;

	//BACK
	const double SLOPE_LOW_HIGH_HATCH_BACK = -0.640856672;
	const double INTERCEPT_LOW_HIGH_HATCH_BACK = 0;
	const double SLOPE_MIDDLE_HATCH_BACK = SLOPE_LOW_HIGH_HATCH_BACK;
	const double INTERCEPT_MIDDLE_HATCH_BACK = INTERCEPT_LOW_HIGH_HATCH_BACK;
	const double SLOPE_CARGO_SHIP_BALL_BACK = SLOPE_LOW_HIGH_HATCH_BACK;
	const double INTERCEPT_CARGO_SHIP_BALL_BACK = INTERCEPT_LOW_HIGH_HATCH_BACK;

	//BALL FRONT
	// const double SLOPE_LOW_HIGH_BALL_FRONT = 2.6758579;
	// const double INTERCEPT_LOW_HIGH_BALL_FRONT = -26.96;
	// const double SLOPE_MID_BALL_FRONT = 0.15;
	// const double INTERCEPT_MID_BALL_FRONT = -3.31;

	// //BALL BACK
	// const double SLOPE_LOW_HIGH_BALL_BACK = -0.55;
	// const double INTERCEPT_LOW_HIGH_BALL_BACK = 4.7;
	// const double SLOPE_MID_BALL_BACK = -0.344;
	// const double INTERCEPT_MID_BALL_BACK = 14.86;

	//Pipelines
	const double HATCH_LOW_HIGH_FINAL_PIPELINE = 1; //vision	  //was 4
	const double HATCH_MID_FINAL_PIPELINE = HATCH_LOW_HIGH_FINAL_PIPELINE; //was 2
	const double BALL_CARGO_SHIP_PIPELINE = HATCH_LOW_HIGH_FINAL_PIPELINE; //6;
	const double DRIVER_PIPELINE = 4; //was 1

	//Distances
	//Front
	const double HATCH_LOW_HIGH_FRONT_DESIRED_INCHES = 25.6; //18.25 HIGH
	const double HATCH_MID_FRONT_DESIRED_INCHES = 38.8;
	const double CARGO_SHIP_FRONT_DESIRED_INCHES = 35.3;
	const double FRONT_INIT_PIPELINE_DESIRED_INCHES = 20;

	//Back
	const double HATCH_LOW_HIGH_BACK_DESIRED_INCHES = 28.4; //27.6 HIGH
	const double HATCH_MID_BACK_DESIRED_INCHES = 52.4;
	const double CARGO_SHIP_BACK_DESIRED_INCHES = 38.9;
	const double BACK_INIT_PIPELINE_DESIRED_INCHES = 20;

	//Path Following
	const bool REVERSE = true;
	const bool FORWARD = false;

  private:
	static Drivetrain *s_instance;

	Drivetrain();
	/**
	 * @brief What to do in LL tracking if the target is not acquired.
	 * @param leftTank the value for the left side of the drivetrain
	 * @param rightTank the value for the right side of the drivetrain
	 * 
	 * Temporarily turn of the isInLLDrive flag, and runs the TankDrive method.
	 * Sets isTargetAcquired to false.
	 */ 
	void IsTargetNotAcquired(double leftTank, double rightTank);

	//Private Instance Objects
	//left back and right back encoder used for drivetrain
	//left front and right front encoder used for arm and wrist respectively
	TalonSRX *leftFront;
	TalonSRX *leftMid;
	TalonSRX *leftBack;
	TalonSRX *rightFront;
	TalonSRX *rightMid;
	TalonSRX *rightBack;

	frc::Solenoid *gearShifter;

	//Various Timers
	frc::Timer *hatchPlaceTimer;
	frc::Timer *angleDGainTimer;

	//Limelights
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
	const int GEAR_SHIFTER_ID = 0;

	double leftDashboardSpeed = 0.0;
	double rightDashboardSpeed = 0.0;
	double desiredRightFPS = 3.0;
	double desiredLeftFPS = 3.0;
	bool driveTrainGearShuffle = true;

	//LL data variables
	double targetOffsetAngle_Horizontal;
	double targetOffsetAngle_Vertical;
	double currentArea;
	double targetSkew;
	double tv;
	double actualPipeline;
	
	double angleErrorAdjustmentValue_LLTracking = 0.0;

	double currentDistanceInches = 0.0;

	//LL tracking variables
	double xDesiredInches;
	double zDesiredInches;
	bool isInAutoDrive;
	bool isInLLDrive;
	bool isFront;
	double slopeForAngleCalc;
	double interceptForAngleCalc;
	double crosshairAngle;
	int pipelineNumber;
	double prevDistance;
	double prevAngle;
	double accumAngleError;
	double prevDistanceError;
	double prevAngleError;
	double prevT;
	bool overrideEnabled;
	bool wantToDriveHatchInPlace;

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
	const double AUTO_VELOCITY_CONTROL_DRIVE_SPEED = 2.5;

	//PID control limelight
	bool isTargetAcquired;

	const double kP_DIST_FPS = -.2;											//Estimate this value by seeing at what percent of the distance you want the speed to be in FPS
	const double kP_NU_PER_100MS = kP_DIST_FPS * FEET_TO_NU * SECONDS_TO_100MS; //Converted from FPS estimate above to NU/100ms that the talon can use
	const double LL_DISTANCE_PER_5FEET_FRONT = 100.0;
	const double LL_DISTANCE_PER_5FEET_BACK = 80.0;
	const double kP_DIST_FPS_BACK = -kP_DIST_FPS * (LL_DISTANCE_PER_5FEET_FRONT / LL_DISTANCE_PER_5FEET_BACK);
	const double LIMELIGHT_HEIGHT_INCHES = 45;
	const double LIMELIGHT_ANGLE_FRONT = 26.0;
	const double LIMELIGHT_ANGLE_BACK = 25.0;
	// const double CROSSHAIR_ANGLE = 6.5; //17.3
	const double PATH_CUTOFF_TIME = 0.25; //adjustable

	const double kP_ANGLE = 0.23;   //FOR ANGLE CORRECTION TODO
	const double kI_ANGLE = 0;//0.0030; //FOR ANGLE CORRECTION TODO
	const double kD_ANGLE = -0.03; 		//FOR ANGLE CORRECTION TODO
	const double I_ZONE_ANGLE = 3;  //degrees

	const double ALLOWED_ANGLE_ERROR_LL = 2; //degrees
	const double ALLOWED_DISTANCE_ERROR_LL = 2; //calculated inches

	const double ANGLE_WITHIN_NO_JUMPS_LL = 5; //degrees
	const double DISTANCE_WITHIN_NO_JUMPS_LL = 10; //calculated inches
	const double MAX_JUMP_ANGLE_ALLOWED = 5; //degrees
	const double MAX_JUMP_DISTANCE_ALLOWED = 5; //calculated inches

	//Constants for the PID speed control built in to the Talon
	const double kP_SPEED = 0.2;						 //FOR SPEED CONTROL
	const double kD_SPEED_RIGHT = kP_SPEED * 20.0 * 1.0; //USE 1.0 VALUE TO CALIBRATE
	const double kD_SPEED_LEFT = kP_SPEED * 20.0 * 1.0;  //FOR SPEED CONTROL

	const double kI_SPEED = kP_SPEED / 100.0;
	const double kI_ZONE = (.5 * 2.0) * FEET_TO_NU * CONVERT_100MS_TO_SECONDS;

	const double kFF_SPEED = (TEST_PERCENT_OUTPUT * MAX_TALON_OUTPUT) / MEASURED_SPEED_NU;

	const double TALON_TIMEOUT = 10; //number of ms before the talon stops trying to configure a specific value

	//Talon Loop Ramp Rates in seconds
	const double TALON_RAMP_RATE = 0.0;

	const double LL_MAX_FEET_PER_SEC = 3.5;

	//Pathfinding
	Segment leftTrajectory[750];
	Segment rightTrajectory[750];

	int left_trajectory_length = 0;
	int right_trajectory_length = 0;

	int follow_path_counter = 0;

	//Gyro
	AHRS *ahrs;

	const double LL_HATCH_PLACE_TIMER = 0.15; //Was 0.4, suggested change by Sebastian, chanegd by Robert
};
