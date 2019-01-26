/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "Objects.h"
#include "AutoTune.h"
#include "Functions.h"
#include "Autonomous.h"

#include <iostream>
#include <frc/Timer.h>
#include <RobotDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>


bool driveWithXbox;
bool arcadeDrive;
const frc::XboxController::JoystickHand leftHand = frc::XboxController::kLeftHand;
const frc::XboxController::JoystickHand rightHand = frc::XboxController::kRightHand;

double rightSpeed = 0.0;
double leftSpeed = 0.0;

double desiredRightFPS = 0.0;
double desiredLeftFPS = 0.0;

double realRightSpeedNUPer100ms = 0.0;
double realLeftSpeedNUPer100ms = 0.0;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoDrive1, kAutoDrive1);
  m_chooser.AddOption(kAutoDrive2, kAutoDrive2);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  leftStick = new frc::Joystick(0);
  rightStick = new frc::Joystick(1);

  xbox = new frc::XboxController(2);

  leftFront = new TalonSRX(23);
  leftBack = new TalonSRX(27);
  rightFront = new TalonSRX(22);
  rightBack = new TalonSRX(26);

  leftFront->SetInverted(false);
  leftBack->SetInverted(false);
  rightFront->SetInverted(true);
  rightBack->SetInverted(true);

  leftBack->SetSensorPhase(true);
  rightBack->SetSensorPhase(true);

  leftBack->Config_kF(0, kFeedForwardGain, talonTimeout);
  rightBack->Config_kF(0, kFeedForwardGain, talonTimeout);
  leftBack->Config_kP(0, kP_SPEED, talonTimeout);
  rightBack->Config_kP(0, kP_SPEED, talonTimeout);
  leftBack->Config_kD(0, kD_SPEED_LEFT, talonTimeout);
  rightBack->Config_kD(0, kD_SPEED_RIGHT, talonTimeout);
  leftBack->Config_kI(0, kI_SPEED, talonTimeout);
  rightBack->Config_kI(0, kI_SPEED, talonTimeout);
  leftBack->Config_IntegralZone(0, kI_ZONE, talonTimeout);
  rightBack->Config_IntegralZone(0, kI_ZONE, talonTimeout);

  // driveTrain = new frc::RobotDrive(leftFront, leftBack, rightFront, rightBack);

  preferences = frc::Preferences::GetInstance();
  driveWithXbox = preferences->GetBoolean("drive with xbox", false);
  arcadeDrive = preferences->GetBoolean("arcade drive", false);

  table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  // wpi::Twine cameraName = wpi::Twine::Twine("http://10.7.44.11:5800");
  // camera = new cs::HttpCamera::HttpCamera("limelight", "10.7.44.11:5800", cs::HttpCamera::kMJPGStreamer);

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
  frc::SmartDashboard::PutNumber("Skew", targetSkew);

  rightSpeed = leftBack->GetSelectedSensorVelocity(0) * NU_TO_FEET * SECONDS_TO_100MS;
  leftSpeed = rightBack->GetSelectedSensorVelocity(0) * NU_TO_FEET * SECONDS_TO_100MS;

  frc::SmartDashboard::PutNumber("Speed Error Right", desiredRightFPS - rightSpeed);
  frc::SmartDashboard::PutNumber("Speed Error Left", desiredLeftFPS - leftSpeed);

  frc::SmartDashboard::PutNumber("Ft-Sec-Right", rightSpeed);
  frc::SmartDashboard::PutNumber("Ft-Sec-Left", leftSpeed);
  frc::SmartDashboard::PutNumber("NU-100ms Left", leftBack->GetSelectedSensorVelocity(0));
  frc::SmartDashboard::PutNumber("NU-100ms Right", rightBack->GetSelectedSensorVelocity(0));

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
      adjust = kP_ANGLE*targetOffsetAngle_Horizontal - minCommmand;
    }
    //Target is to the right of the Robot
    else if (targetOffsetAngle_Horizontal > 1.0){
      adjust = kP_ANGLE*targetOffsetAngle_Horizontal + minCommmand;
    }
    
    leftPower = adjust;
    rightPower = -adjust;

    leftBack->Set(ControlMode::PercentOutput, leftPower);
    leftFront->Set(ControlMode::Follower, LEFT_TALON_MASTER);
    rightBack->Set(ControlMode::PercentOutput, rightPower);
    rightFront->Set(ControlMode::Follower, RIGHT_TALON_MASTER);
    // driveTrain->TankDrive(leftPower, rightPower, false);
    
  }

  if(xbox->GetBackButton()){

    desiredLeftFPS = desiredRightFPS = 6.0;
    
    leftBack->Set(ControlMode::Velocity, desiredLeftFPS * FEET_TO_NU * CONVERT_100MS_TO_SECONDS);
    leftFront->Set(ControlMode::Follower, LEFT_TALON_MASTER);
    rightBack->Set(ControlMode::Velocity, desiredRightFPS * FEET_TO_NU * CONVERT_100MS_TO_SECONDS);
    rightFront->Set(ControlMode::Follower, RIGHT_TALON_MASTER);
  }

  else if (!arcadeDrive) {
    if (driveWithXbox) {
      
      leftPower = -xbox->GetY(leftHand);
      rightPower = -xbox->GetY(rightHand);

      // speedTankDrive(xbox->GetY(leftHand), xbox->GetY(rightHand));
      leftBack->Set(ControlMode::PercentOutput, leftPower);
      leftFront->Set(ControlMode::Follower, LEFT_TALON_MASTER);
      rightBack->Set(ControlMode::PercentOutput, rightPower);
      rightFront->Set(ControlMode::Follower, RIGHT_TALON_MASTER);
    }
    else {

      leftPower = -leftStick->GetY();
      rightPower = -rightStick->GetY();

      // speedTankDrive(leftStick->GetY(), rightStick->GetY());
      leftBack->Set(ControlMode::PercentOutput, leftPower);
      leftFront->Set(ControlMode::Follower, LEFT_TALON_MASTER);
      rightBack->Set(ControlMode::PercentOutput, rightPower);
      rightFront->Set(ControlMode::Follower, RIGHT_TALON_MASTER);
    }
  }

  else {
    if (driveWithXbox) {
      // driveTrain->ArcadeDrive(xbox->GetY(leftHand), xbox->GetX(rightHand), false);
    }
    else {
      // driveTrain->ArcadeDrive(leftStick->GetY(), rightStick->GetX(), false);
    }
  }

  frc::SmartDashboard::PutNumber("Left Power", leftPower);
  frc::SmartDashboard::PutNumber("Right Power", rightPower);

  if(xbox->GetAButtonPressed()){
    driveWithXbox = true;
    preferences->PutBoolean("drive with xbox", true);
    // table->PutNumber("camMode", 1.0);
  }
  if(xbox->GetBButtonPressed()){
    driveWithXbox = false;
    preferences->PutBoolean("drive with xbox", false);
    // table->PutNumber("camMode", 0.0);
  }
  if(xbox->GetXButtonPressed()){
    // arcadeDrive = true;
    // preferences->PutBoolean("arcade drive", true);
    table->PutNumber("ledMode", 1);
  }
  if(xbox->GetYButtonPressed()){
    // arcadeDrive = false;
    // preferences->PutBoolean("arcade drive", false);
    table->PutNumber("ledMode", 0);
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

// void speedTankDrive(double leftSpeed, double rightSpeed, bool squared){
  
//   if(!squared){
//     leftFront->Set(ControlMode::Velocity, leftSpeed);
//     leftBack->Set(ControlMode::Follower, 23);

//     rightFront->Set(ControlMode::Velocity, rightSpeed);
//     rightBack->Set(ControlMode::Follower, 22);
//   }
//   else {
//     leftFront->Set(ControlMode::Velocity, (leftSpeed*leftSpeed));
//     leftBack->Set(ControlMode::Follower, 23);

//     rightFront->Set(ControlMode::Velocity, (rightSpeed*rightSpeed));
//     rightBack->Set(ControlMode::Follower, 22);
//   }
// }