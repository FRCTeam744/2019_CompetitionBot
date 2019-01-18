/*----------------------------------------------------------------------------------*/

//All new objects are pointed to here, then the objects are instantiated in Robot.cpp

/*----------------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <ctre/Phoenix.h>
#include <frc/RobotDrive.h>
#include <frc/Preferences.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTable.h"

frc::Joystick *rightStick, *leftStick;

frc::XboxController *xbox;

WPI_TalonSRX *leftFront, *leftBack, *rightFront, *rightBack;

frc::RobotDrive *driveTrain;

frc::Preferences *preferences;

std::shared_ptr<NetworkTable> table;