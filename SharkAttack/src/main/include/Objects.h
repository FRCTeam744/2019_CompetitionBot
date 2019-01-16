/*----------------------------------------------------------------------------------*/

//All new objects are pointed to here, then the objects are instantiated in RobotInit

/*----------------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <ctre/Phoenix.h>
#include <frc/RobotDrive.h>
#include <frc/Preferences.h>

frc::Joystick *rightStick, *leftStick;

frc::XboxController *xbox;

WPI_TalonSRX *leftFront, *leftBack, *rightFront, *rightBack;

frc::RobotDrive *driveTrain;

frc::Preferences *preferences;