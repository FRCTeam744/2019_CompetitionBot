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

#include "ShuffleManager.h"

class OI
{
	//TODO: Go through each button and assign variable in cpp file in functions
  public:
	static OI *GetInstance();

	void SwitchDriveMode();
	bool SwitchGears();
	bool GetIsHighGear();

	bool GetVelocityTest();

	bool SwitchGripper();
	bool GetIsGripperClosed();
	void PrintToSmartDashboard(double encoderValue);

	double GetLeftDriveInput();
	double GetRightDriveInput();
	bool GetAutoDriveForward();
	double GetArmInput();
	double GetWristInput();
	double GetArmEncoder();
	double GetWristEncoder();
	double GetIntakeOut();
	double GetIntakeIn();
	double GetTargetPosition();
	// double ArmPresetPickupCargo();
	// double ArmPresetLow();
	// double ArmPresetMid();
	// double ArmPresetHigh();
	// double ArmPresetNegPickupCargo();
	// double ArmPresetNegLow();
	// double ArmPresetNegMid();
	// double ArmPresetNegHigh();

	bool GetFourbarExtend();
	bool GetFourbarRetract();
	bool GetFourbarHome();

	bool SetArmFrontHigh();
	bool SetArmFrontMid();
	bool SetArmFrontLow();
	bool SetArmFrontBallPickup();
	bool SetArmBackHigh();
	bool SetArmBackMid();
	bool SetArmBackLow();
	bool SetArmBackBallPickup();

	// void SwitchLED_Mode(Drivetrain drivetrain);
	//void PutOnShuffleboardInOI();

	std::tuple<bool, std::string, double> SetLimelight();

	bool LEDButtonPressed();
	bool AlsoLEDButtonPressed();

  private:
	static OI *s_instance;

	OI();

	//Auto Methods (will probably not be used in favor of teleop control during sandstorm)
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoDrive1 = "Drive Off Level 1";
	const std::string kAutoDrive2 = "Drive Off Level 2";
	std::string m_autoSelected;

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

	//Arm Constants
	const double ARM_PICKUP_CARGO_PRESET_DEG = 35;		//A
	const double ARM_PICKUP_LOW_DEG = 45;				//X
	const double ARM_PICKUP_MID_DEG = 90;				//B
	const double ARM_PICKUP_HIGH_DEG = 135;				//Y
	const double ARM_PICKUP_NEG_CARGO_PRESET_DEG = -35; //DOWN
	const double ARM_PICKUP_NEG_LOW_DEG = -45;			//RIGHT
	const double ARM_PICKUP_NEG_MID_DEG = 4.4;			//LEFT // -90
	const double ARM_PICKUP_NEG_HIGH_DEG = 0;		//UP // -135

	double targetArmPosition;

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



    //TARGET HEIGHTS IN INCHES (USE THESE FOR CALCULATION NOT FOR SETTING)
    const double HIGH_HATCH_HEIGHT = 67.0;
    const double MID_HATCH_HEIGHT = 43.0;
    const double LOW_HATCH_HEIGHT = 19.0;
    const double HIGH_BALL_HEIGHT = 71.5;
    const double MID_BALL_HEIGHT = 47.5;
    const double LOW_BALL_HEIGHT = 23.5;
    const double CARGOSHIP_BALL_HEIGHT = 47.5; //Don't know the height yet, needs to be measured
    const double BALL_PICKUP_HEIGHT = 23.5; //Don't know the height yet, needs to be measured

    //ARM MEASUREMENTS
    const double PIVOT_HEIGHT = 46.15;
    const double ARM_LENGTH = 33.5;

	const double RADIANS_TO_DEGREES = (180.0/M_PI);

    //ARM POSITIONS IN DEGREES (USE FOR SETTING ENCODER)
    const double FRONT_HIGH_BALL_POSITION = RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - HIGH_BALL_HEIGHT) / (ARM_LENGTH));
    const double FRONT_HIGH_HATCH_POSITION = RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - HIGH_HATCH_HEIGHT) / (ARM_LENGTH));
    const double FRONT_MID_BALL_POSITION = RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - MID_BALL_HEIGHT) / (ARM_LENGTH));
    const double FRONT_MID_HATCH_POSITION = RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - MID_HATCH_HEIGHT) / (ARM_LENGTH));
    const double FRONT_LOW_BALL_POSITION = RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - LOW_BALL_HEIGHT) / (ARM_LENGTH));
    const double FRONT_LOW_HATCH_POSITION = RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - LOW_HATCH_HEIGHT) / (ARM_LENGTH));
    const double FRONT_BALL_PICKUP_POSITION = RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - BALL_PICKUP_HEIGHT) / (ARM_LENGTH));
    const double FRONT_CARGOSHIP_BALL_POSITION = RADIANS_TO_DEGREES * acos((PIVOT_HEIGHT - CARGOSHIP_BALL_HEIGHT) / (ARM_LENGTH));
    const double BACK_HIGH_BALL_POSITION = RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - HIGH_BALL_HEIGHT) / (ARM_LENGTH));
    const double BACK_HIGH_HATCH_POSITION = RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - HIGH_HATCH_HEIGHT) / (ARM_LENGTH));
    const double BACK_MID_BALL_POSITION = RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - MID_BALL_HEIGHT) / (ARM_LENGTH));
    const double BACK_MID_HATCH_POSITION = RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - MID_HATCH_HEIGHT) / (ARM_LENGTH));
    const double BACK_LOW_BALL_POSITION = RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - LOW_BALL_HEIGHT) / (ARM_LENGTH));
    const double BACK_LOW_HATCH_POSITION = RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - LOW_HATCH_HEIGHT) / (ARM_LENGTH));
    const double BACK_BALL_PICKUP_POSITION = RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - BALL_PICKUP_HEIGHT) / (ARM_LENGTH));
    const double BACK_CARGOSHIP_BALL_POSITION = RADIANS_TO_DEGREES * -acos((PIVOT_HEIGHT - CARGOSHIP_BALL_HEIGHT) / (ARM_LENGTH));

    const double NEUTRAL_ARM_POSITION = 0.0;
};
