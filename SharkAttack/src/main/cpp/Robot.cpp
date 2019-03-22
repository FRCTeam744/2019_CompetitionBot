/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

static void VisionThread()
{
  // cs::UsbCamera USBCam = frc::CameraServer::GetInstance()->StartAutomaticCapture();
  // USBCam.SetResolution(640,480);
  // USBCam.SetFPS(12);
}

void Robot::RobotInit()
{
  drivetrain = Drivetrain::GetInstance();
  oi = OI::GetInstance();
  arm = Arm::GetInstance();
  fourbar = Fourbar::GetInstance();
  // led = LED::GetInstance();
  isBeforeMatch = true;
  
  shufflemanager = ShuffleManager::GetInstance();
 // shufflemanager->ShuffleInit();
 // shufflemanager->VariableInit();

  frc::SmartDashboard::PutNumber("fourbarSpeed", 0.1);

  frc::SmartDashboard::PutNumber("wristEncoder", 0.1);

  isBeforeMatch = true;

  // std::thread vision(VisionThread);
  // vision.detach();

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
  //oi->PutOnShuffleboard();

  drivetrain->LimelightSet(oi->SetLimelight());
  drivetrain->PrintDriveShuffleInfo();
  fourbar->PrintFourbarShuffleInfo();
  arm->PrintArmShuffleInfo();

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
  shufflemanager->ShuffleInit();
  shufflemanager->VariableInit();

}

void Robot::TeleopPeriodic()
{
  // led->HatchOrBallMode(oi->GetPlacingMode(), oi->GetDriveByLimelight());

  arm->UpdateArmAndWristInManual(oi->GetIsArmInManual(), oi->GetIsWristInManual());

  arm->PrintArmInfotoConsole();
  // frc::SmartDashboard::PutNumber("sampleEncoder Value: ", sampleEncoder->GetRaw()); //Testing
  // std::cout << "sampleEncoder Value: " << sampleEncoder->GetRaw() << std::endl;

  // if (oi->LEDButtonPressed())
  // {
  //   // led->LEDsOff();
  // }
  // if (oi->AlsoLEDButtonPressed())
  // {
  //   // led->SwimmingShark();
  // }

  drivetrain->AutoDriveForward(oi->GetAutoDriveForward(), oi->GetVelocityTest());
  GetDesiredLLDistances(oi->GetTargetArmPosition());
  drivetrain->SetDesiredLLDistances(xDesiredInches, zDesiredInches);
  // drivetrain->AutoDriveLL(oi->GetDriveByLimelight(), oi->GetLeftDriveInput(), oi->GetRightDriveInput());
  drivetrain->AutoDrive(oi->GetDriveByLimelight(), oi->GetLeftDriveInput(), oi->GetRightDriveInput(), oi->GetPlacingMode(), oi->GetStopLLMove());
  //std::cout << "zDesiredInches: " << zDesiredInches << std::endl;
  arm->ManualRotateArm(oi->GetArmInput());
  arm->ManualRotateWrist(oi->GetWristInput());
  arm->MoveArmToPosition(oi->GetTargetArmPosition(), oi->GetPlacingMode(), oi->GetIsInBallPickup(), oi->IsInCargoShipMode());
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
  // alliance = frc::DriverStation::GetInstance().GetAlliance();
}

void Robot::DisabledPeriodic()
{

  // if (isBeforeMatch)
  // {
  //   led->StartUp();
  // }

  // if (!isBeforeMatch)
  // {
  //   led->ShutDown();
  // }

  // std::cout << "DisabledPeriodic running" << std::endl;
  // std::cout << "hasSetUpForMatch: " << hasSetUpForMatch << std::endl;
  if (hasSetUpForMatch == false)
  {
    //Speed things up
    if ((oi->GetFakeFMSConnected() == true))
    {
      arm->SetToMatchMode();
      hasSetUpForMatch = true;
    }
  }
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
  if (armTargetPosition == oi->FRONT_HIGH_BALL_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = 10.52; // TODO CHANGE
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_BALL_FRONT, drivetrain->INTERCEPT_LOW_HIGH_BALL_FRONT);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_BALL_LOW_HIGH_FRONT);
    drivetrain->SetPipelineNumber(3);
  }
  if (armTargetPosition == oi->BACK_HIGH_BALL_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = 22; // TODO CHANGE
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_BALL_BACK, drivetrain->INTERCEPT_LOW_HIGH_BALL_BACK);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_BALL_LOW_HIGH_BACK);
    drivetrain->SetPipelineNumber(3);
  }
  if (armTargetPosition == oi->FRONT_MID_BALL_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = 22; // TODO CHANGE
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_MID_BALL_FRONT, drivetrain->INTERCEPT_MID_BALL_FRONT);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_BALL_MIDDLE_FRONT);
    drivetrain->SetPipelineNumber(4);
  }
  if (armTargetPosition == oi->BACK_MID_BALL_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = 22; // TODO CHANGE
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_MID_BALL_BACK, drivetrain->INTERCEPT_MID_BALL_BACK);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_BALL_MIDDLE_BACK);
    drivetrain->SetPipelineNumber(4);
  }
  if (armTargetPosition == oi->FRONT_LOW_BALL_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = 10.52; // TODO CHANGE
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_BALL_FRONT, drivetrain->INTERCEPT_LOW_HIGH_BALL_FRONT);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_BALL_LOW_HIGH_FRONT);
    drivetrain->SetPipelineNumber(3);
  }
  if (armTargetPosition == oi->BACK_LOW_BALL_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = 13.39; // TODO CHANGE
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_BALL_BACK, drivetrain->INTERCEPT_LOW_HIGH_BALL_BACK);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_BALL_LOW_HIGH_BACK);
    drivetrain->SetPipelineNumber(3);
  }
  if (armTargetPosition == oi->FRONT_HIGH_HATCH_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = 33; //26.4; //was 22.6
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_HATCH_FRONT, drivetrain->INTERCEPT_LOW_HIGH_HATCH_FRONT);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_FRONT);
    drivetrain->SetPipelineNumber(0);
  }
  if (armTargetPosition == oi->BACK_HIGH_HATCH_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = 24;
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_HATCH_BACK, drivetrain->INTERCEPT_LOW_HIGH_HATCH_BACK);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_BACK);
    drivetrain->SetPipelineNumber(0);
  }
  if (armTargetPosition == oi->FRONT_MID_HATCH_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = 36;
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_MIDDLE_HATCH_FRONT, drivetrain->INTERCEPT_MIDDLE_HATCH_FRONT);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_MIDDLE_HATCH_FRONT);
    drivetrain->SetPipelineNumber(2.0);
  }
  if (armTargetPosition == oi->BACK_MID_HATCH_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = 23;
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_MIDDLE_HATCH_BACK, drivetrain->INTERCEPT_MIDDLE_HATCH_BACK);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_MIDDLE_HATCH_BACK);
    drivetrain->SetPipelineNumber(2.0);
  }
  if (armTargetPosition == oi->FRONT_LOW_HATCH_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = 33; ///26.4;
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_HATCH_FRONT, drivetrain->INTERCEPT_LOW_HIGH_HATCH_FRONT);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_FRONT);
    drivetrain->SetPipelineNumber(0);
  }
  if (armTargetPosition == oi->BACK_LOW_HATCH_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = 24;
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_HATCH_BACK, drivetrain->INTERCEPT_LOW_HIGH_HATCH_BACK);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_BACK);
    drivetrain->SetPipelineNumber(0);
  }
  if (armTargetPosition > 0)
  {
    drivetrain->SetIsFrontLL(true);
  }
  else
  {
    drivetrain->SetIsFrontLL(false);
  }
}