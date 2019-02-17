/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"


static void VisionThread(){
  // cs::UsbCamera USBCam = frc::CameraServer::GetInstance()->StartAutomaticCapture();
  // USBCam.SetResolution(640,480);
  // USBCam.SetFPS(12);
}

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoDrive1, kAutoDrive1);
  m_chooser.AddOption(kAutoDrive2, kAutoDrive2);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  drivetrain = Drivetrain::GetInstance();
  oi = OI::GetInstance();
  arm = Arm::GetInstance();
  fourbar = Fourbar::GetInstance();
  led = LED::GetInstance();

  frc::SmartDashboard::PutNumber("fourbarSpeed", 0.1);
  // std::thread vision(VisionThread);
  // vision.detach();

  // printf("%s\n","Hi, if you see this, the SD/SB method is working apparently!");
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
  fourbar->UpdateFourbarSpeed();
  oi->PutOnShuffleboard();

  drivetrain->LimelightSet(oi->SetLimelight());

  fourbar->PrintClimberRPM();
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
void Robot::AutonomousInit()
{
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoDrive2)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic()
{
  if (m_autoSelected == kAutoDrive2)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  
  if (alliance == blue){
    led->StartUpBlue();
  }
  else if (alliance == red){
    led->StartUpRed();
  }
}

void Robot::TeleopPeriodic() {

  if (oi->LEDButtonPressed()) {
    led->LEDsOff();
  }
  if (oi->AlsoLEDButtonPressed()) {
    led->SwimmingShark();
  }
  
  //drivetrain->leftBack->Set(ControlMode::PercentOutput, oi->GetLeftDriveInput());
  //drivetrain->rightBack->Set(ControlMode::PercentOutput, oi->GetRightDriveInput());
  //drivetrain->leftMid->Set(ControlMode::PercentOutput, oi->GetLeftDriveInput());
  //drivetrain->rightMid->Set(ControlMode::PercentOutput, oi->GetRightDriveInput());
  //drivetrain->leftFront->Set(ControlMode::PercentOutput, oi->GetLeftDriveInput());
  //drivetrain->rightFront->Set(ControlMode::PercentOutput, oi->GetRightDriveInput());
  
  drivetrain->PutData();

  arm->ManualRotateArm(oi->GetArmInput());
  arm->ManualRotateWrist(oi->GetWristInput());
  
  fourbar->ExtendOrRetract(oi->GetFourbarExtend(), oi->GetFourbarRetract());

  oi->PrintToSmartDashboard(drivetrain->GetArmEncoderValue());
  drivetrain->TankDrive(oi->GetLeftDriveInput(), oi->GetRightDriveInput());

  if (oi->SwitchGears()){
    drivetrain->CheckSwitchGears(oi->GetIsHighGear());
  }

  if (oi->SwitchGripper()){
    arm->CheckHatchGripper(oi->GetIsGripperClosed());
  }
}

void Robot::DisabledInit() {
  
  led->StartUp();
  alliance = frc::DriverStation::GetInstance().GetAlliance();
}

void Robot::DisabledPeriodic() {

}

void Robot::TestPeriodic() {
  
}


#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif