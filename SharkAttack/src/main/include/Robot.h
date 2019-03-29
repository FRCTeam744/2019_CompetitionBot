/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/WPILib.h"

#include <string>
#include <math.h>
#include <iostream>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <frc/DriverStation.h>

#include "Drivetrain.h"
#include "OI.h"
#include "Arm.h"
#include "Fourbar.h"
#include "LED.h"
#include "ShuffleManager.h"
#include <frc/Encoder.h>

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestPeriodic() override;
  void GetDesiredLLDistances(double armTargetPosition);
  void AutoStateMachine();

  double xDesiredInches;
  double zDesiredInches;

  bool isShufflePopulated = false;

private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoHatchRightCargo = "Place 2 Hatches on the Right Cargo Ship";
  const std::string kAutoHatchLeftCargo = "Place 2 Hatches on the Left Cargo Ship";
  const std::string kAutoHatchRightRocket = "Place 2 Hatches on the Right Rocket";
  const std::string kAutoHatchLeftRocket = "Place 2 Hatches on the Left Rocket";
  const std::string kAutoRunTeleop = "Run Teleop Rather Than Autonomous";
  std::string m_autoSelected;

  Drivetrain *drivetrain;
  OI *oi;
  Arm *arm;
  Fourbar *fourbar;
  LED *led;
  ShuffleManager *shufflemanager;

  //LED CODE
  frc::DriverStation::Alliance alliance;
  bool isBeforeMatch = true;

  const frc::DriverStation::Alliance blue = frc::DriverStation::Alliance::kBlue;
  const frc::DriverStation::Alliance red = frc::DriverStation::Alliance::kRed;
  bool hasSetUpForMatch = false;

  frc::Encoder *sampleEncoder; //Testing

  //Auto Paths
  int path_count = 0;
  enum auto_states {FOLLOW_PATH_STATE, DRIVE_BY_LL_STATE, DELAY_STATE, TELEOP_STATE};
  enum auto_states auto_state;

  bool autoIsGripperClosed = true;

  frc::Timer *hatchDelayTimer;
  const double TOGGLE_HATCH_DELAY = 0.5;

  frc::Timer *armMoveDelayTimer;
  const double ARM_MOVE_DELAY = 1.0;

  frc::Timer *intakeEjectDelayTimer;
  const double INTAKE_EJECT_DELAY = 0.5;

  // std::vector<std::string> autoPathNames;
  // std::vector<bool> autoPathDirections;
  // std::vector<double> autoArmPresets;

  //TEST AUTO
  std::vector<std::string> autoPathNames;
	std::vector<bool> autoPathDirections;
              // = {drivetrain->FORWARD, drivetrain->REVERSE};
	std::vector<double> autoArmPresets;
              // = {oi->FRONT_LOW_HATCH_POSITION, oi->BACK_LOW_HATCH_POSITION};

  //CARGO SHIP RIGHT
  // const int CARGO_SHIP_RIGHT_AUTO_STEPS = 4;
  // std::string cargoShipRightPaths[CARGO_SHIP_RIGHT_AUTO_STEPS] 
  //             = {"TestPath", "TestPath", "TestPath", "TestPath"};
	// bool   cargoShipRightPathDirections[CARGO_SHIP_RIGHT_AUTO_STEPS] 
  //             = {drivetrain->FORWARD, drivetrain->REVERSE, drivetrain->FORWARD, drivetrain->REVERSE};
	// double cargoShipRightArmPresets[CARGO_SHIP_RIGHT_AUTO_STEPS] 
  //             = {oi->FRONT_LOW_HATCH_POSITION, oi->BACK_LOW_HATCH_POSITION, oi->FRONT_LOW_HATCH_POSITION, oi->BACK_LOW_HATCH_POSITION};

  bool prevIsLLDriveFinished = false;
};
