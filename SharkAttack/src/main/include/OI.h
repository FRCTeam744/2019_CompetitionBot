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
#include "Drivetrain.h"
// #include "Drivetrain.h"

class OI {
	//TODO: Go through each button and assign variable in cpp file in functions
	public:

		static OI* GetInstance();

		void SwitchDriveMode();
		void SwitchGears();
		void PrintToSmartDashboard(double encoderValue);
		double GetLeftDriveInput();
		double GetRightDriveInput();
		double GetArmInput();
		double GetWristInput();
		bool GetFourbarExtend();
		bool GetFourbarRetract();
		// void SwitchLED_Mode(Drivetrain drivetrain);
		bool lowGear, highGear = 0;

	private:

		static OI* s_instance;

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

		//Private Objects in OI.cpp
		frc::Joystick *rightStick;
    	frc::Joystick *leftStick;
    	frc::XboxController *xbox;

		frc::Preferences *preferences;
		
		cs::UsbCamera camera; //Untested to see if usb camera class works for limelight

		//Private Instance Variables
		bool driveWithXbox = false;
		bool isInLowGear;
		double armPowerOutput = 0.0;
		double wristPowerOutput = 0.0;
};

