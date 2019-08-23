/*----------------------------------------------------------------------------------*/

//Header file for OI.cpp

/*----------------------------------------------------------------------------------*/

#pragma once

//Inlcude all libraries that are necessary for operator interface, delete any that aren't used
#include <string>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/Preferences.h>
#include <cscore.h>
#include <cameraserver/CameraServer.h>
#include <frc/smartdashboard/Smartdashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <iostream>

#include "ShuffleManager.h"

/**
 @brief Class to handle all assignment and reading of buttons and joysticks from the driver station.
 */
class OI
{
	//TODO: Go through each button and assign variable in cpp file in functions
  public:
	static OI *GetInstance();
	/**
    @brief Gets button input. Either drive forward at test speed or drive whichever side the arm is facing.

    Either drive forward at test speed or drive whichever side the arm is facing.
  	*/
	bool GetAutoDriveForward();
	/**
	 @brief Gets the button input for the pickup of hatches by the limelight
	 
	 Gets the button input for the pickup of hatches by the limelight
	 */
	bool GetDriveByLimelightPickup();
	/**
	 @brief Gets the button input for the placement of hatches by the limelight
	 
	 Gets the button input for the placement of hatches by the limelight
	 */
	bool GetDriveByLimelightPlace();

	/**
	 @brief Switches gear for transition
	 @return Either switch gears or don't

	 Switches gear if button that is pressed is for the opposite gear.
	 */
	bool SwitchGears();
	
	/**
	  @brief Returns if the transmition is in high gear.

	  Returns if the transmition is in high gear.
	*/
	bool GetIsHighGear();

	/**
    @brief Gets button input. Either drive forward at test speed or drive whichever side the arm is facing.

    Either drive forward at test speed or drive whichever side the arm is facing.
  	*/
	bool GetVelocityTest();

	/**
    @brief Switches the gripper mode to either open or closed (gripped or released respectively)
	@returns Returns switched value of true when gripper is closed, false when open.

    Switches the gripper mode to either open or closed (gripped or released respectively)
  	*/
	bool SwitchGripper();

	/**
    @brief Get method that returns whether the gripper is closed or not
	@returns The gripper state (closed/open or gripped/released respectively)

    Get method that returns whether the gripper is closed or not
  	*/
	bool GetIsGripperClosed();

	/**
    @brief Switches arm control to manual when joystick is moved significantly
	@returns Power given to arm based on joystick movements.

	Switches arm control to manual when joystick is moved significantly
  	*/
	double GetArmInput();

	/**
    @brief Switches wrist control to manual when joystick is moved significantly
	@returns Power given to arm based on joystick movements.

	Switches arm control to manual when joystick is moved significantly. Otherwise
	the arm is in auto
  	*/
	double GetWristInput();

	/**
    @brief Return an intake power on trigger pull
	@returns Return a double of intake power (left trigger intakes, right pushes out)

	Return an intake power on trigger pull on xbox controller
  	*/
	double GetIntakeInput();

	/**
    @brief Left joystick pressed, hatch mode. Right joystick pressed, ball mode
	@returns If the robot is in ball mode

	If left joystick pressed, switch into hatch mode. If right joystick is pressed, switch into ball mode.
  	*/
	bool GetPlacingMode();

	/**
    @brief Set whether the arm is in ball mode or hatch mode

	Set whether the arm is in ball mode or hatch mode
  	*/
	void SetPlacingMode(bool isInBallMode);

	/**
    @brief Returns if arm is in ball pickup mode
	@returns Whether arm is in ball pickup mode

	Returns if arm is in ball pickup mode
  	*/
	bool GetIsInBallPickup();

	/**
    @brief Returns if arm is in manual control mode
	@returns Returns if arm is in manual control mode

	Returns if arm is in manual control mode
  	*/
	bool GetIsArmInManual();

	/**
    @brief Returns if wrist is in manual control mode
	@returns Returns if wrist is in manual control mode

	Returns if wrist is in manual control mode
  	*/
	bool GetIsWristInManual();

	/**
    @brief Inverts left stick value and returns the output
	@returns Returns the inverted value of the stick input

	Joysticks natively give out negative values when going forward, so adding the negative corrects it
  	*/
	double GetLeftDriveInput();

	/**
    @brief Inverts right stick value and returns the output
	@returns Returns the inverted value of the stick input

	Joysticks natively give out negative values when going forward, so adding the negative corrects it
  	*/
	double GetRightDriveInput();

	/**
    @brief Returns if the Fourbar extend button is pressed
	@returns Returns whether the Fourbar extend button is pressed

	Returns whether the Fourbar extend button is pressed
  	*/
	bool GetFourbarExtend();

	/**
    @brief Returns if the Fourbar extend button is pressed
	@returns Returns whether the Fourbar extend button is pressed

	Returns whether the Fourbar extend button is pressed
  	*/
	bool GetFourbarRetract();
	/**
    @brief Returns if the Fourbar retract button is pressed
	@returns Returns whether the Fourbar retract button is pressed

	Returns whether the Fourbar retract button is pressed
  	*/
	bool GetFourbarHome();

	/**
    @brief State machine that checks all buttons and changes the current state of the arm
	@returns The target arm position that the arm will move towards

	State machine that checks all buttons and changes the current state of the arm
  	*/
	double GetTargetArmPosition();

	/**
    @brief Sets a target arm position for the arm to move towards

	Sets a target arm position for the arm to move towards
  	*/
	void SetTargetArmPosition(double targetArmPosition);

	/**
    @brief Returns if the robot is in cargo ship mode
	@returns If Xbox B Button is pressed or xbox DPad right is pressed

	Returns if the robot is in cargo ship mode
  	*/
	bool IsInCargoShipMode();

	/**
    @brief State machine that checks all buttons and changes the current state of the wrist
	@returns The target wrist position that the wrist will move towards

	State machine that checks all buttons and changes the current state of the wrist
  	*/
	double GetTargetWristPosition();

	/**
    @brief checks left joystick's button press to stop the limelight
	@returns left stick button pressed to stop the limelights auto

	checks left joystick's button press to stop the limelight
  	*/
	bool GetStopLLMove();

	/**
    @brief returns the voltage for the FeedForward
	@returns Feed Forward Voltage

	returns the voltage for the FeedForward
  	*/
	double GetArmFFVoltage();

	/**
    @brief Sets Arm and Wrist manual mode

	Sets Arm and Wrist manual mode
  	*/
	void SetArmWristInManual(bool isArmManual, bool isWristManual);

	std::tuple<bool, std::string, double> SetLimelight();

	// void SwitchLED_Mode(Drivetrain drivetrain);
	//void PutOnShuffleboardInOI();


	//TARGET HEIGHTS IN INCHES (USE THESE FOR CALCULATION NOT FOR SETTING)
	const double HIGH_HATCH_HEIGHT = 67.0;
	const double MID_HATCH_HEIGHT = 44.0; //Was 43.0, changed by Robert
	const double LOW_HATCH_HEIGHT = 19.0;
	const double HIGH_BALL_HEIGHT = 71.5;
	const double MID_BALL_HEIGHT = 47;
	const double LOW_BALL_HEIGHT = 23.5;
	const double CARGOSHIP_BALL_HEIGHT = 43; //Don't know the height yet, needs to be measured
	const double BALL_PICKUP_HEIGHT = 19.0;	//Don't know the height yet, needs to be measured

	//ARM MEASUREMENTS
	const double PIVOT_HEIGHT = 46.15;
	const double ARM_LENGTH = 33.5;

	const double RADIANS_TO_DEGREES = (180.0 / M_PI);

	//add to back and front to compensate for the deadzone in the bands
	const double DEADZONE_ADJUSTOR = -2.5;

	//ARM POSITIONS IN DEGREES (USE FOR SETTING ENCODER)
	const double FRONT_HIGH_BALL_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - HIGH_BALL_HEIGHT) / (ARM_LENGTH)));
	const double FRONT_HIGH_HATCH_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - HIGH_HATCH_HEIGHT) / (ARM_LENGTH)));
	const double FRONT_MID_BALL_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - MID_BALL_HEIGHT) / (ARM_LENGTH)));
	const double FRONT_MID_HATCH_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - MID_HATCH_HEIGHT) / (ARM_LENGTH)));
	const double FRONT_LOW_BALL_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - LOW_BALL_HEIGHT) / (ARM_LENGTH)));
	const double FRONT_LOW_HATCH_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - LOW_HATCH_HEIGHT) / (ARM_LENGTH)));
	const double FRONT_BALL_PICKUP_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - BALL_PICKUP_HEIGHT) / (ARM_LENGTH)));
	const double FRONT_CARGOSHIP_BALL_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - CARGOSHIP_BALL_HEIGHT) / (ARM_LENGTH)));
	const double BACK_HIGH_BALL_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - HIGH_BALL_HEIGHT) / (ARM_LENGTH)));
	const double BACK_HIGH_HATCH_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - HIGH_HATCH_HEIGHT) / (ARM_LENGTH)));
	const double BACK_MID_BALL_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - MID_BALL_HEIGHT) / (ARM_LENGTH)));
	const double BACK_MID_HATCH_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - MID_HATCH_HEIGHT) / (ARM_LENGTH)));
	const double BACK_LOW_BALL_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - LOW_BALL_HEIGHT) / (ARM_LENGTH)));
	const double BACK_LOW_HATCH_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - LOW_HATCH_HEIGHT) / (ARM_LENGTH)));
	const double BACK_BALL_PICKUP_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - BALL_PICKUP_HEIGHT) / (ARM_LENGTH)));
	const double BACK_CARGOSHIP_BALL_POSITION = DEADZONE_ADJUSTOR + (RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - CARGOSHIP_BALL_HEIGHT) / (ARM_LENGTH)));

	const double NEUTRAL_ARM_POSITION = 0.0;

  private:
	static OI *s_instance;

	OI();

	//Auto Methods (will probably not be used in favor of teleop control during sandstorm)
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoDrive1 = "Drive Off Level 1";
	const std::string kAutoDrive2 = "Drive Off Level 2";
	std::string m_autoSelected;

	//CONTROLLER DEADZONES
	const double ARM_DEADZONE = 0.5;
	const double WRIST_DEADZONE = 0.5;

	//Use LEFT_HAND and RIGHT_HAND constants to designate left and right sides of XboxController in OI.cpp
	const frc::XboxController::JoystickHand LEFT_HAND = frc::XboxController::kLeftHand;
	const frc::XboxController::JoystickHand RIGHT_HAND = frc::XboxController::kRightHand;

	//Use D-Pad constants to designate the button direction
	const int DPAD_UP = 0;
	const int DPAD_RIGHT = 90;
	const int DPAD_DOWN = 180;
	const int DPAD_LEFT = 270;

	//Camera Constants (change these to change camera quality in SmartDashboard)
	const int resolutionWidth = 160;
	const int resolutionHeight = 120;
	const int framerate = 10;

	//Wrist Constants //Work in Progress
	const double WRIST_BALL_PICKUP_FRONT_CARGO_PRESET_DEG = -90; //A
	const double WRIST_BALL_PICKUP_FRONT_LOW_DEG = 0;
	const double WRIST_BALL_PICKUP_FRONT_MID_DEG = -90;		   //B
	const double WRIST_BALL_PICKUP_FRONT_HIGH_DEG = -90;	   //Y
	const double WRIST_BALL_PICKUP_BACK_CARGO_PRESET_DEG = 90; //DOWN
	const double WRIST_BALL_PICKUP_BACK_LOW_DEG = 0;
	const double WRIST_BALL_PICKUP_BACK_MID_DEG = 90;  //RIGHT
	const double WRIST_BALL_PICKUP_BACK_HIGH_DEG = 90; //UP

	const double WRIST_NEUTRAL_DEG = 0; //X and LEFT

	const double WRIST_HATCH_PICKUP_FRONT_CARGO_PRESET_DEG = 90; //A
	const double WRIST_HATCH_PICKUP_FRONT_LOW_DEG = 0;
	const double WRIST_HATCH_PICKUP_FRONT_MID_DEG = 90;			 //B
	const double WRIST_HATCH_PICKUP_FRONT_HIGH_DEG = 90;		 //Y
	const double WRIST_HATCH_PICKUP_BACK_CARGO_PRESET_DEG = -90; //DOWN
	const double WRIST_HATCH_PICKUP_BACK_LOW_DEG = 0;
	const double WRIST_HATCH_PICKUP_BACK_MID_DEG = -90;  //RIGHT
	const double WRIST_HATCH_PICKUP_BACK_HIGH_DEG = -90; //UP

	double targetArmPosition;
	double targetWristPosition;

	//Private Objects in OI.cpp
	frc::Joystick *rightStick;
	frc::Joystick *leftStick;
	frc::XboxController *xbox;
	//bool isInitialized = false;

	frc::Preferences *preferences;

	cs::UsbCamera camera; //Untested to see if usb camera class works for limelight

	//Private Instance Variables
	bool isHighGear;
	bool isGripperClosed;
	double armPowerOutput = 0.0;
	double wristPowerOutput = 0.0;

	bool isInBallMode;
	bool isInBallPickup;
	bool isArmInManual;
	bool isWristInManual;
	bool isInCargoShipMode;

	double armFFVoltage;

	bool wasDPADRightPressed;

};
