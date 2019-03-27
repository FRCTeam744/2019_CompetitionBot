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
  hatchDelayTimer = new frc::Timer();
  armMoveDelayTimer = new frc::Timer();

  isBeforeMatch = true;
  shufflemanager = ShuffleManager::GetInstance();

    
  m_chooser.SetDefaultOption(kAutoRunTeleop, kAutoRunTeleop);
  m_chooser.AddOption("Right Cargo Autonomous", kAutoHatchRightCargo);
  m_chooser.AddOption("Left Cargo Autonomous", kAutoHatchLeftCargo);
  m_chooser.AddOption("Right Rocket Autonomous", kAutoHatchRightRocket);
  m_chooser.AddOption("Left Rocket Autonomous", kAutoHatchLeftRocket);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // m_pCanScale.AddOption("Yes", true);
  // m _pCanScale.SetDefaultOption("No", false);

  frc::Shuffleboard::GetTab("DriverView").Add("Auto Modes", m_chooser).WithWidget(frc::BuiltInWidgets::kComboBoxChooser);

  frc::SmartDashboard::PutNumber("fourbarSpeed", 0.1);

  frc::SmartDashboard::PutNumber("wristEncoder", 0.1);

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

  arm->ToggleDefenseMode(oi->GetIsArmInDefenseMode());
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
  if(isShufflePopulated == false){
    shufflemanager->ShuffleInit();
    shufflemanager->VariableInit();
    isShufflePopulated = true;
  }



  isBeforeMatch = false;
  // drivetrain->AutonomousInit();

  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;
  path_count = 0;
  autoIsGripperClosed = true;
  armMoveDelayTimer->Reset();
  armMoveDelayTimer->Start();

  autoPathNames.clear();
  autoPathDirections.clear();
  autoArmPresets.clear();
            

  if (m_autoSelected == kAutoHatchRightCargo) {
    // Custom Auto goes here
    // autoPathNames = testPaths;
    // autoPathDirections = testDirections;
    // autoArmPresets = testArmPresets;
    // autoPathNames.push_back("TestPath");
    autoPathNames.push_back("CenterPlatformToLeftShip");
    autoPathDirections.push_back(drivetrain->FORWARD);
    autoPathNames.push_back("LeftShipToLoadingStation");
    autoPathDirections.push_back(drivetrain->REVERSE);
    autoArmPresets.push_back(oi->FRONT_LOW_HATCH_POSITION);
    autoArmPresets.push_back(oi->BACK_LOW_HATCH_POSITION);
    
    drivetrain->FollowPathInit(autoPathNames.at(path_count));
    arm->MoveArmToPosition(autoArmPresets.at(path_count), false, false, false);
    auto_state = FOLLOW_PATH_STATE;
  } else if (m_autoSelected == kAutoHatchLeftCargo) {

  }
  else if(m_autoSelected == kAutoHatchRightRocket){
  
    // Another Custom Auto goes here
    // autoPathNames = testPaths;
    // autoPathDirections = testDirections;
    // autoArmPresets = testArmPresets;
    autoPathNames.push_back("TestPath");
    autoPathNames.push_back("TestPath");
    autoPathDirections.push_back(drivetrain->FORWARD);
    autoPathDirections.push_back(drivetrain->REVERSE);
    autoArmPresets.push_back(oi->FRONT_LOW_HATCH_POSITION);
    autoArmPresets.push_back(oi->BACK_LOW_HATCH_POSITION);
    
    drivetrain->FollowPathInit(autoPathNames.at(path_count));
    arm->MoveArmToPosition(autoArmPresets.at(path_count), false, false, false);
    auto_state = FOLLOW_PATH_STATE;
  } else if(m_autoSelected == kAutoHatchLeftRocket){

  }
  else  if(m_autoSelected == kAutoRunTeleop){ 
    TeleopPeriodic();
    // Default Auto goes here
  } else {
    TeleopPeriodic();
  }
}

void Robot::AutonomousPeriodic()
{
  if (m_autoSelected == kAutoHatchRightCargo) {
    AutoStateMachine();
    // drivetrain->FollowPath(drivetrain->FORWARD);
    // Custom Auto goes here

  } else if(m_autoSelected == kAutoHatchLeftCargo) {
    AutoStateMachine();
    // drivetrain->FollowPath(drivetrain->FORWARD);
    // Custom Auto goes here
  }
  else if(m_autoSelected == kAutoHatchRightRocket){
    AutoStateMachine();
    // drivetrain->FollowPath(drivetrain->FORWARD);
    // Another Custom Auto goes here

  } else if(m_autoSelected == kAutoHatchLeftRocket){
     AutoStateMachine();
    // drivetrain->FollowPath(drivetrain->FORWARD);
    // Another Custom Auto goes here    
  }
  else if(m_autoSelected == kAutoRunTeleop){ 
    TeleopPeriodic();
    // Default Auto goes here
  } else {
    TeleopPeriodic();
  }
  
}

void Robot::AutoStateMachine() {
  switch (auto_state) {
    case FOLLOW_PATH_STATE:
      {
        //if arm delay timer is finished, move arm
        if(armMoveDelayTimer->Get() > ARM_MOVE_DELAY) {
          arm->MoveArmToPosition(autoArmPresets.at(path_count), false, false, false);
        }

        //follow path until it's done, then switch to Drive by LL state
        bool isPathDone = drivetrain->FollowPath(autoPathDirections[path_count]);
        if(isPathDone) {
          auto_state = DRIVE_BY_LL_STATE;
        }
      }
      break;
    case DRIVE_BY_LL_STATE:
      {
        //keep active control of arm at current path preset
        arm->MoveArmToPosition(autoArmPresets.at(path_count), false, false, false);

        //drive by limelight
        GetDesiredLLDistances(autoArmPresets.at(path_count));
        drivetrain->SetDesiredLLDistances(xDesiredInches, zDesiredInches);
        bool isLLDriveDone = drivetrain->AutoDrive(true, oi->GetLeftDriveInput(), oi->GetRightDriveInput(), false, false);

        //if done with drive by ll
        if(isLLDriveDone) {
          //Toggle Hatch Panel Gripper
          autoIsGripperClosed = !autoIsGripperClosed;
          arm->CheckHatchGripper(autoIsGripperClosed);

          //start and reset timer to wait for hatch mechanism to grip/release 
          hatchDelayTimer->Reset();
          hatchDelayTimer->Start();

          //switch state
          auto_state = DELAY_STATE;
        }
      }
      break;
    case DELAY_STATE:
      {
        //if done waiting for hatch gripper to move
        if(hatchDelayTimer->Get() > TOGGLE_HATCH_DELAY) {
          path_count++;
          //if no more paths, go to teleop
          std::cout << "Paths Size: " << autoPathNames.size() << std::endl;
          if(path_count >= autoPathNames.size()) {
            auto_state = TELEOP_STATE;
          } 
          //otherwise, initialize next path, reset arm move timer, and go to path follow state
          else {
            drivetrain->FollowPathInit(autoPathNames.at(path_count));
            armMoveDelayTimer->Reset();
            armMoveDelayTimer->Start();
            auto_state = FOLLOW_PATH_STATE;
          }
          
        }
      }
      break;
    case TELEOP_STATE:
      {
        TeleopPeriodic();
      }
  }
}

void Robot::TeleopInit()
{
if(isShufflePopulated == false){
    shufflemanager->ShuffleInit();
    shufflemanager->VariableInit();
    isShufflePopulated = true;
  }
}

void Robot::TeleopPeriodic()
{
  // led->IsHatchOpen(arm->GetIsGripperGripped(), oi->GetDriveByLimelight());

  arm->UpdateArmAndWristInManual(oi->GetIsArmInManual(), oi->GetIsWristInManual());

  arm->PrintArmInfotoConsole();
  // frc::SmartDashboard::PutNumber("sampleEncoder Value: ", sampleEncoder->GetRaw()); //Testing
  // std::cout << "sampleEncoder Value: " << sampleEncoder->GetRaw() << std::endl;

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
  // if (hasSetUpForMatch == false)
  // {
  //   //Speed things up
  //   if ((oi->GetFakeFMSConnected() == true))
  //   {
  //     arm->SetToMatchMode();
  //     hasSetUpForMatch = true;
  //   }
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
  if (armTargetPosition == oi->FRONT_HIGH_BALL_POSITION)
  { //NOT USED NOW
    // xDesiredInches = 0;
    // zDesiredInches = 10.52; // TODO CHANGE
    // drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_BALL_FRONT, drivetrain->INTERCEPT_LOW_HIGH_BALL_FRONT);
    // drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_BALL_LOW_HIGH_FRONT);
    // drivetrain->SetPipelineNumber(3);
  }
  if (armTargetPosition == oi->BACK_HIGH_BALL_POSITION)
  { //NOT USED NOW
    // xDesiredInches = 0;
    // zDesiredInches = 22; // TODO CHANGE
    // drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_BALL_BACK, drivetrain->INTERCEPT_LOW_HIGH_BALL_BACK);
    // drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_BALL_LOW_HIGH_BACK);
    // drivetrain->SetPipelineNumber(3);
  }
  if (armTargetPosition == oi->FRONT_MID_BALL_POSITION || armTargetPosition == oi->FRONT_CARGOSHIP_BALL_POSITION)
  { //USED FOR CARGO BAY
    xDesiredInches = 0;
    zDesiredInches = drivetrain->CARGO_SHIP_FRONT_DESIRED_INCHES; // TODO CHANGE
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_CARGO_SHIP_BALL_FRONT, drivetrain->INTERCEPT_CARGO_SHIP_BALL_FRONT);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_BALL_CARGO_SHIP_FRONT);
    drivetrain->SetPipelineNumber(drivetrain->BALL_CARGO_SHIP_PIPELINE);
  }
  if (armTargetPosition == oi->BACK_MID_BALL_POSITION || armTargetPosition == oi->BACK_CARGOSHIP_BALL_POSITION)
  { //USED FOR CARGO BAY
    xDesiredInches = 0;
    zDesiredInches = drivetrain->CARGO_SHIP_BACK_DESIRED_INCHES; // TODO CHANGE
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_CARGO_SHIP_BALL_BACK, drivetrain->INTERCEPT_CARGO_SHIP_BALL_BACK);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_BALL_CARGO_SHIP_BACK);
    drivetrain->SetPipelineNumber(drivetrain->BALL_CARGO_SHIP_PIPELINE);
  }
  if (armTargetPosition == oi->FRONT_LOW_BALL_POSITION)
  { //NOT USED NOW
    // xDesiredInches = 0;
    // zDesiredInches = 10.52; // TODO CHANGE
    // drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_BALL_FRONT, drivetrain->INTERCEPT_LOW_HIGH_BALL_FRONT);
    // drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_BALL_LOW_HIGH_FRONT);
    // drivetrain->SetPipelineNumber(3);
  }
  if (armTargetPosition == oi->BACK_LOW_BALL_POSITION)
  { //NOT USED NOW
    // xDesiredInches = 0;
    // zDesiredInches = 13.39; // TODO CHANGE
    // drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_BALL_BACK, drivetrain->INTERCEPT_LOW_HIGH_BALL_BACK);
    // drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_BALL_LOW_HIGH_BACK);
    // drivetrain->SetPipelineNumber(3);
  }
  if (armTargetPosition == oi->FRONT_HIGH_HATCH_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = drivetrain->HATCH_LOW_HIGH_FRONT_DESIRED_INCHES; //26.4; //was 22.6
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_HATCH_FRONT, drivetrain->INTERCEPT_LOW_HIGH_HATCH_FRONT);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_FRONT);
    drivetrain->SetPipelineNumber(drivetrain->HATCH_LOW_HIGH_FINAL_PIPELINE);
  }
  if (armTargetPosition == oi->BACK_HIGH_HATCH_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = drivetrain->HATCH_LOW_HIGH_BACK_DESIRED_INCHES;
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_HATCH_BACK, drivetrain->INTERCEPT_LOW_HIGH_HATCH_BACK);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_BACK);
    drivetrain->SetPipelineNumber(drivetrain->HATCH_LOW_HIGH_FINAL_PIPELINE);
  }
  if (armTargetPosition == oi->FRONT_MID_HATCH_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = drivetrain->HATCH_MID_FRONT_DESIRED_INCHES;
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_MIDDLE_HATCH_FRONT, drivetrain->INTERCEPT_MIDDLE_HATCH_FRONT);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_MIDDLE_HATCH_FRONT);
    drivetrain->SetPipelineNumber(drivetrain->HATCH_MID_FINAL_PIPELINE);
  }
  if (armTargetPosition == oi->BACK_MID_HATCH_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = drivetrain->HATCH_MID_BACK_DESIRED_INCHES;
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_MIDDLE_HATCH_BACK, drivetrain->INTERCEPT_MIDDLE_HATCH_BACK);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_MIDDLE_HATCH_BACK);
    drivetrain->SetPipelineNumber(drivetrain->HATCH_MID_FINAL_PIPELINE);
  }
  if (armTargetPosition == oi->FRONT_LOW_HATCH_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = drivetrain->HATCH_LOW_HIGH_FRONT_DESIRED_INCHES; ///26.4;
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_HATCH_FRONT, drivetrain->INTERCEPT_LOW_HIGH_HATCH_FRONT);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_FRONT);
    drivetrain->SetPipelineNumber(drivetrain->HATCH_LOW_HIGH_FINAL_PIPELINE);
    // zDesiredInches = 33; ///26.4;
    // drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_HATCH_FRONT, drivetrain->INTERCEPT_LOW_HIGH_HATCH_FRONT);
    // drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_FRONT);
    // drivetrain->SetPipelineNumber(0);
  }
  if (armTargetPosition == oi->BACK_LOW_HATCH_POSITION)
  {
    xDesiredInches = 0;
    zDesiredInches = drivetrain->HATCH_LOW_HIGH_BACK_DESIRED_INCHES;
    drivetrain->SetSlopeInterceptForAngleCalc(drivetrain->SLOPE_LOW_HIGH_HATCH_BACK, drivetrain->INTERCEPT_LOW_HIGH_HATCH_BACK);
    drivetrain->SetCrosshairAngle(drivetrain->CROSSHAIR_TY_ANGLE_LOW_HIGH_HATCH_BACK);
    drivetrain->SetPipelineNumber(drivetrain->HATCH_LOW_HIGH_FINAL_PIPELINE);
  }
  if (armTargetPosition >= 0)
  {
    drivetrain->SetIsFrontLL(true);
  }
  else
  {
    drivetrain->SetIsFrontLL(false);
  }
}