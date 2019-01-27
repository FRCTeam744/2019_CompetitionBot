/*----------------------------------------------------------------------------------*/

//All new objects are pointed to here, then the objects are instantiated in Robot.cpp

/*----------------------------------------------------------------------------------*/
#pragma once
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <ctre/Phoenix.h>
#include <frc/RobotDrive.h>
#include <frc/Preferences.h>
#include <cscore_oo.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTable.h"

frc::Joystick *rightStick, *leftStick;

frc::XboxController *xbox;

TalonSRX *leftFront, *leftBack, *rightFront, *rightBack;

frc::RobotDrive *driveTrain;

frc::Preferences *preferences;

std::shared_ptr<NetworkTable> table;

cs::HttpCamera *camera;