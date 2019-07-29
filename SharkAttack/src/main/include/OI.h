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

	bool SwitchGripper();
	bool GetIsGripperClosed();

	double GetArmInput();
	double GetWristInput();
	double GetIntakeInput();

	bool GetPlacingMode();
	void SetPlacingMode(bool isInBallMode);

	bool GetIsInBallPickup();
	bool GetIsArmInManual();
	bool GetIsWristInManual();

	double GetLeftDriveInput();
	double GetRightDriveInput();

	void PrintToSmartDashboard(double encoderValue);

	bool GetFourbarExtend();
	bool GetFourbarRetract();
	bool GetFourbarHome();

	double GetTargetArmPosition();
	void SetTargetArmPosition(double targetArmPosition);

	bool IsInCargoShipMode();

	double GetTargetWristPosition();

	std::tuple<bool, std::string, double> SetLimelight();

	bool GetStopLLMove();
	double GetArmFFVoltage();

	void SetArmWristInManual(bool isArmManual, bool isWristManual);

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
