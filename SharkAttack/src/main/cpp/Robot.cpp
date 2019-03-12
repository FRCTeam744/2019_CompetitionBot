/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

static void VisionThread()
{
  cs::UsbCamera USBCam = frc::CameraServer::GetInstance()->StartAutomaticCapture();
  USBCam.SetResolution(640,480);
  USBCam.SetFPS(12);
}

void Robot::RobotInit()
{
  drivetrain = Drivetrain::GetInstance();
  oi = OI::GetInstance();
  arm = Arm::GetInstance();
  fourbar = Fourbar::GetInstance();
  // led = LED::GetInstance();
  
  shufflemanager = ShuffleManager::GetInstance();
  shufflemanager->ShuffleInit();
  shufflemanager->VariableInit();

  frc::SmartDashboard::PutNumber("fourbarSpeed", 0.1);

  frc::SmartDashboard::PutNumber("wristEncoder", 0.1);

  std::thread vision(VisionThread);
  vision.detach();

  //Testing
  // frc::Encoder *sampleEncoder = new frc::Encoder(0, 1, false, frc::Encoder::EncodingType::k4X);
  // sampleEncoder->SetMaxPeriod(.1);
  // sampleEncoder->SetMinRate(10);
  // sampleEncoder->SetDistancePerPulse(5);
  // sampleEncoder->SetReverseDirection(true);
  // sampleEncoder->SetSamplesToAverage(7);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  fourbar->UpdateFourbarSpeed();

  drivetrain->LimelightSet(oi->SetLimelight());
  drivetrain->PutData();
  

  fourbar->PrintClimberRPM();
  arm->PrintArmInfo();

  // arm->SetMAX_FF_GAIN(oi->GetArmFFVoltage());
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
  isBeforeMatch = false;
  
}

void Robot::AutonomousPeriodic()
{
  TeleopPeriodic();
}

void Robot::TeleopInit()
{

  if (alliance == blue)
  {
    // led->StartUpBlue();
  }
  else if (alliance == red)
  {
    // led->StartUpRed();
  }
}

void Robot::TeleopPeriodic()
{
  arm->UpdateArmAndWristInManual(oi->GetIsArmInManual(), oi->GetIsWristInManual());

  arm->PrintArmInfotoConsole();
  // frc::SmartDashboard::PutNumber("sampleEncoder Value: ", sampleEncoder->GetRaw()); //Testing
  // std::cout << "sampleEncoder Value: " << sampleEncoder->GetRaw() << std::endl;

  if (oi->LEDButtonPressed())
  {
    // led->LEDsOff();
  }
  if (oi->AlsoLEDButtonPressed())
  {
    // led->SwimmingShark();
  }

  drivetrain->AutoDriveForward(oi->GetAutoDriveForward(), oi->GetVelocityTest());
  GetDesiredLLDistances(oi->GetTargetArmPosition());
  drivetrain->AutoDriveLL(oi->GetDriveByLimelight(), true, false, true, oi->GetLeftDriveInput(), oi->GetRightDriveInput());
//   arm->ManualRotateArm(oi->GetArmInput());
//   arm->ManualRotateWrist(oi->GetWristInput());
  arm->MoveArmToPosition(oi->GetTargetArmPosition(), oi->GetPlacingMode(), oi->GetIsInBallPickup());
  //arm->MoveWristToPosition(oi->GetTargetWristPosition());
  //std::cout << "Arm Position: " << arm->GetArmEncoderValue() << std::endl;

  //std::cout << "Target Position: " << oi->GetTargetPosition() << std::endl;
  arm->RunIntake(oi->GetIntakeInput());

  fourbar->ExtendOrRetract(oi->GetFourbarExtend(), oi->GetFourbarRetract());
  fourbar->FourbarHome(oi->GetFourbarHome());

  drivetrain->TankDrive(oi->GetLeftDriveInput(), oi->GetRightDriveInput());

  if (oi->SwitchGears())
  {
    drivetrain->CheckSwitchGears(oi->GetIsHighGear());
  }

  if (oi->SwitchGripper())
  {
    arm->CheckHatchGripper(oi->GetIsGripperClosed());
  }

}

void Robot::DisabledInit()
{
  alliance = frc::DriverStation::GetInstance().GetAlliance();
}

void Robot::DisabledPeriodic()
{
  // if (isBeforeMatch) {
  //   led->StartUp();
  // }
  
  // if (!isBeforeMatch) {
  //   led->ShutDown();
  // }
  
}

void Robot::TestPeriodic()
{
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif

void Robot::GetDesiredLLDistances(double armTargetPosition)
{
    if ((armTargetPosition == oi->FRONT_HIGH_BALL_POSITION) || (armTargetPosition == oi->BACK_HIGH_BALL_POSITION))
    {
        
    }
    if ((armTargetPosition == oi->FRONT_MID_BALL_POSITION) || (armTargetPosition == oi->BACK_MID_BALL_POSITION))
    {
        
    }
    if ((armTargetPosition == oi->FRONT_LOW_BALL_POSITION) || (armTargetPosition == oi->BACK_LOW_BALL_POSITION))
    {
        
    }
    if ((armTargetPosition == oi->FRONT_HIGH_HATCH_POSITION) || (armTargetPosition == oi->BACK_HIGH_HATCH_POSITION))
    {
        
    }
    if ((armTargetPosition == oi->FRONT_MID_HATCH_POSITION) || (armTargetPosition == oi->BACK_MID_HATCH_POSITION))
    {
        xDesiredInches = 0;
        zDesiredInches = -36;
    }
    if ((armTargetPosition == oi->FRONT_LOW_HATCH_POSITION) || (armTargetPosition == oi->BACK_LOW_HATCH_POSITION))
    {
        
    }
}