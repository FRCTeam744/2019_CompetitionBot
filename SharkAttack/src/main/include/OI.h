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
#include <CameraServer.h>

class OI
{
	//TODO: Go through each button and assign variable in cpp file in functions
	public:
		void OI_Init();
		void TurnOffLEDs();

	private:

		//Auto Methods (will probably not be used in favor of teleop control during sandstorm)
		frc::SendableChooser<std::string> m_chooser;
		const std::string kAutoDrive1 = "Drive Off Level 1";
		const std::string kAutoDrive2 = "Drive Off Level 2";
		std::string m_autoSelected;

		//Use leftHand and rightHand constants to designate left and right sides of XboxController in OI.cpp
		const frc::XboxController::JoystickHand leftHand = frc::XboxController::kLeftHand;
		const frc::XboxController::JoystickHand rightHand = frc::XboxController::kRightHand;

		//Private Objects in OI.cpp
		frc::Joystick *rightStick;
    	frc::Joystick *leftStick;
    	frc::XboxController *xbox;

		frc::Preferences *preferences;
		
		cs::UsbCamera *camera; //Untested to see if usb camera works for limelight

		//Private Instance Variables
		bool driveWithXbox = false;
};

