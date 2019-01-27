/*----------------------------------------------------------------------------------*/

//Header file for OI.cpp

/*----------------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/Preferences.h>

const frc::XboxController::JoystickHand leftHand = frc::XboxController::kLeftHand;
const frc::XboxController::JoystickHand rightHand = frc::XboxController::kRightHand;

class OI
{
	public:
		void SelectRobotDrive();

	private:
		frc::SendableChooser<std::string> m_chooser;
		const std::string kAutoDrive1 = "Drive Off Level 1";
		const std::string kAutoDrive2 = "Drive Off Level 2";
		std::string m_autoSelected;
  
};

