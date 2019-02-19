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

	bool SwitchGripper();
	bool GetIsGripperClosed();
	void PrintToSmartDashboard(double encoderValue);

	double GetLeftDriveInput();
	double GetRightDriveInput();
	double GetArmInput();
	double GetWristInput();
	double GetArmEncoder();
	double GetWristEncoder();
	double GetIntakeOut();
	double GetIntakeIn();

	double ArmPresetPickupCargo();
	double ArmPresetLow();
	double ArmPresetMid();
	double ArmPresetHigh();
	double ArmPresetNegPickupCargo();
	double ArmPresetNegLow();
	double ArmPresetNegMid();
	double ArmPresetNegHigh();

	bool GetFourbarExtend();
	bool GetFourbarRetract();
	bool GetFourbarHome();

	bool SetPresetToAButton();
	bool SetPresetToBButton();
	bool SetPresetToYButton();
	bool SetPresetToXButton();
	bool SetPresetToDPadUp();
	bool SetPresetToDPadRight();
	bool SetPresetToDPadDown();
	bool SetPresetToDPadLeft();

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

	//Use leftHand and rightHand constants to designate left and right sides of XboxController in OI.cpp
	const frc::XboxController::JoystickHand LEFT_HAND = frc::XboxController::kLeftHand;
	const frc::XboxController::JoystickHand RIGHT_HAND = frc::XboxController::kRightHand;

	//Camera Constants (change these to change camera quality in SmartDashboard)
	const int resolutionWidth = 160;
	const int resolutionHeight = 120;
	const int framerate = 10;

	//Arm Constants
	const double ARM_POWER_SCALE = 0.2;
	const double WRIST_POWER_SCALE = 0.2;
	const double ARM_PICKUP_CARGO_PRESET_DEG = 35; //Y
	const double ARM_PICKUP_LOW_DEG = 45; //A
	const double ARM_PICKUP_MID_DEG = 90; //B
	const double ARM_PICKUP_HIGH_DEG = 135; //X
	const double ARM_PICKUP_NEG_CARGO_PRESET_DEG = -35; //UP
	const double ARM_PICKUP_NEG_LOW_DEG = -45; //RIGHT
	const double ARM_PICKUP_NEG_MID_DEG = -90; //DOWN
	const double ARM_PICKUP_NEG_HIGH_DEG = -135; //LEFT

	//Private Objects in OI.cpp
	frc::Joystick *rightStick;
	frc::Joystick *leftStick;
	frc::XboxController *xbox;
	bool isInitialized = false;

	frc::Preferences *preferences;

	cs::UsbCamera camera; //Untested to see if usb camera class works for limelight

	//Private Instance Variables
	bool isHighGear;
	bool isGripperClosed;
	double armPowerOutput = 0.0;
	double wristPowerOutput = 0.0;
};
