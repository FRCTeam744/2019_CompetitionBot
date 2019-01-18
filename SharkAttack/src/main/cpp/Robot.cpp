/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "Objects.h"
#include "AutoTune.h"

#include <iostream>
#include <RobotDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>


bool driveWithXbox;
bool arcadeDrive;
const frc::XboxController::JoystickHand leftHand = frc::XboxController::kLeftHand;
const frc::XboxController::JoystickHand rightHand = frc::XboxController::kRightHand;


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoDrive1, kAutoDrive1);
  m_chooser.AddOption(kAutoDrive2, kAutoDrive2);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  leftStick = new frc::Joystick(0);
  rightStick = new frc::Joystick(1);

  xbox = new frc::XboxController(2);

  leftFront = new WPI_TalonSRX(23);
  leftBack = new WPI_TalonSRX(27);
  rightFront = new WPI_TalonSRX(22);
  rightBack = new WPI_TalonSRX(26);

  leftFront->SetInverted(true);
  leftBack->SetInverted(true);
  rightFront->SetInverted(true);
  rightBack->SetInverted(true);

  driveTrain = new frc::RobotDrive(leftFront, leftBack, rightFront, rightBack);

  preferences = frc::Preferences::GetInstance();
  driveWithXbox = preferences->GetBoolean("drive with xbox", false);
  arcadeDrive = preferences->GetBoolean("arcade drive", false);

  table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

  targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
  targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  targetArea = table->GetNumber("ta",0.0);
  targetSkew = table->GetNumber("ts",0.0);

  frc::SmartDashboard::PutNumber("Heading", targetOffsetAngle_Horizontal);
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoDrive2) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoDrive2) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

  if (xbox->GetStartButton()){
    
    // if (targetOffsetAngle_Horizontal <= 0.8 && targetOffsetAngle_Horizontal >= -0.8){
    //   adjust = 0;
    // }
    //Target is to the left of the Robot
    if (targetOffsetAngle_Horizontal < -1.0){
      adjust = kP*targetOffsetAngle_Horizontal - minCommmand;
    }
    //Target is to the right of the Robot
    else if (targetOffsetAngle_Horizontal > 1.0){
      adjust = kP*targetOffsetAngle_Horizontal + minCommmand;
    }
    
    leftPower = -adjust;
    rightPower = adjust;
    
    frc::SmartDashboard::PutNumber("Left Power", leftPower);
    frc::SmartDashboard::PutNumber("Right Power", rightPower);

    driveTrain->TankDrive(leftPower, rightPower, false);
    
  }
  else if (!arcadeDrive) {
    if (driveWithXbox) {
      driveTrain->TankDrive(xbox->GetY(leftHand), xbox->GetY(rightHand), false);
    }
    else {
      driveTrain->TankDrive(leftStick->GetY(), rightStick->GetY(), false);
    }
  }

  else {
    if (driveWithXbox) {
      driveTrain->ArcadeDrive(xbox->GetY(leftHand), xbox->GetX(rightHand), false);
    }
    else {
      driveTrain->ArcadeDrive(leftStick->GetY(), rightStick->GetX(), false);
    }
  }

  if(xbox->GetAButtonPressed()){
    driveWithXbox = true;
    preferences->PutBoolean("drive with xbox", true);
  }
  if(xbox->GetBButtonPressed()){
    driveWithXbox = false;
    preferences->PutBoolean("drive with xbox", false);
  }
  if(xbox->GetXButtonPressed()){
    arcadeDrive = true;
    preferences->PutBoolean("arcade drive", true);
  }
  if(xbox->GetYButtonPressed()){
    arcadeDrive = false;
    preferences->PutBoolean("arcade drive", false);
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
