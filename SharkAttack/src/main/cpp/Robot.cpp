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
  //set up instances of different robot subsystems
  drivetrain = Drivetrain::GetInstance();
  oi = OI::GetInstance();
  arm = Arm::GetInstance();
  fourbar = Fourbar::GetInstance();
  led = LED::GetInstance();
  shufflemanager = ShuffleManager::GetInstance();

  //initiate various timers
  hatchDelayTimer = new frc::Timer();
  armMoveDelayTimer = new frc::Timer();
  periodTimeRemaining = new frc::Timer();
  backupDelayTimer = new frc::Timer();

  isBeforeMatch = true;
  
  //Set up auto chooser
  m_chooser.SetDefaultOption(kAutoRunTeleop, kAutoRunTeleop);
  m_chooser.AddOption("Right Cargo Autonomous", kAutoHatchRightCargo);
  m_chooser.AddOption("Left Cargo Autonomous", kAutoHatchLeftCargo);
  m_chooser.AddOption("Low Right Rocket Autonomous", kAutoHatchLowRightRocket);
  m_chooser.AddOption("Low Left Rocket Autonomous", kAutoHatchLowLeftRocket);
  m_chooser.AddOption("High Right Rocket Autonomous", kAutoHatchHighRightRocket);
  m_chooser.AddOption("High Left Rocket Autonomous", kAutoHatchHighLeftRocket);
  m_chooser.AddOption("TEST Path", kAutoTest);
  frc::Shuffleboard::GetTab("DriverView").Add("Auto Modes", m_chooser).WithWidget(frc::BuiltInWidgets::kComboBoxChooser);

  //RoboInit calls done here
  drivetrain->RobotInit();
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
  //functions that are called regardless of the robot state
  fourbar->UpdateFourbarSpeed();
  drivetrain->LimelightSet(oi->SetLimelight());
  drivetrain->PrintDriveShuffleInfo();
  fourbar->PrintFourbarShuffleInfo();
  arm->PrintArmShuffleInfo();
  ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->periodMatchTimeFMS, periodTimeRemaining->GetMatchTime());

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
  led->LEDsOff();

  //populate the shuffle board once after startup
  if (isShufflePopulated == false)
  {
    shufflemanager->TabInit();
    shufflemanager->VariableInit();
    isShufflePopulated = true;
  }

  isBeforeMatch = false;
  drivetrain->AutonomousInit();
  arm->SetDesiredHatchGripperState(true);

  m_autoSelected = m_chooser.GetSelected();
  path_count = 0;
  autoIsGripperClosed = true;
  armMoveDelayTimer->Reset();
  armMoveDelayTimer->Start();
  backupDelayTimer->Reset();
  backupDelayTimer->Start();


  autoPathNames.clear();
  autoPathDirections.clear();
  autoArmPresets.clear();

  lowestArmAngle = oi->FRONT_HIGH_HATCH_POSITION + 10;

  //add the correct path names, directions and arm heights for the various autos
  if (m_autoSelected == kAutoHatchRightCargo)
  {
    oi->SetArmWristInManual(false, false);
    arm->UpdateArmAndWristInManual(false, false);

    autoPathNames.push_back("CenterPlatformToLeftFrontCargo");
    autoPathDirections.push_back(drivetrain->FORWARD);
    autoArmPresets.push_back(oi->FRONT_LOW_HATCH_POSITION);

    autoPathNames.push_back("RightShipToRightStation");
    autoPathDirections.push_back(drivetrain->REVERSE);
    autoArmPresets.push_back(oi->BACK_LOW_HATCH_POSITION);

    autoPathNames.push_back("RightStationToRightCargo1");
    autoPathDirections.push_back(drivetrain->FORWARD);
    autoArmPresets.push_back(oi->FRONT_LOW_HATCH_POSITION);

    drivetrain->FollowPathInit(autoPathNames.at(path_count));
    arm->MoveArmToPosition(autoArmPresets.at(path_count), false, false, false);
    auto_state = FOLLOW_PATH_STATE;
  }
  else if (m_autoSelected == kAutoHatchLeftCargo)
  {
    // Custom Auto goes here
    oi->SetArmWristInManual(false, false);
    arm->UpdateArmAndWristInManual(false, false);

    autoPathNames.push_back("CenterPlatformToLeftFrontCargo");
    autoPathDirections.push_back(drivetrain->FORWARD);
    autoArmPresets.push_back(oi->FRONT_LOW_HATCH_POSITION);

    autoPathNames.push_back("LeftShipToLeftStation");
    autoPathDirections.push_back(drivetrain->REVERSE);
    autoArmPresets.push_back(oi->BACK_LOW_HATCH_POSITION);

    autoPathNames.push_back("LeftStationToLeftCargo1");
    autoPathDirections.push_back(drivetrain->FORWARD);
    autoArmPresets.push_back(oi->FRONT_LOW_HATCH_POSITION);

    drivetrain->FollowPathInit(autoPathNames.at(path_count));
    arm->MoveArmToPosition(autoArmPresets.at(path_count), false, false, false);
    auto_state = FOLLOW_PATH_STATE;
  }
  else if (m_autoSelected == kAutoTest)
  {
    oi->SetArmWristInManual(false, false);
    arm->UpdateArmAndWristInManual(false, false);

    // Another Custom Auto goes here
    autoPathNames.push_back("TestPath");
    autoPathDirections.push_back(drivetrain->FORWARD);
    autoArmPresets.push_back(oi->FRONT_LOW_HATCH_POSITION);

    drivetrain->FollowPathInit(autoPathNames.at(path_count));
    arm->MoveArmToPosition(autoArmPresets.at(path_count), false, false, false);
    auto_state = FOLLOW_PATH_STATE;
  }
  else if (m_autoSelected == kAutoHatchHighLeftRocket)
  {
    oi->SetArmWristInManual(false, false);
    arm->UpdateArmAndWristInManual(false, false);

    autoPathNames.push_back("LeftPlatformToLeftFrontRocket");
    autoPathDirections.push_back(drivetrain->FORWARD);
    autoArmPresets.push_back(oi->FRONT_HIGH_HATCH_POSITION);

    autoPathNames.push_back("LeftFrontRocketToLeftStation");
    autoPathDirections.push_back(drivetrain->REVERSE);
    autoArmPresets.push_back(oi->BACK_LOW_HATCH_POSITION);

    autoPathNames.push_back("LeftStationToLeftBackRocket"); //high on opposite side
    autoPathDirections.push_back(drivetrain->FORWARD);
    autoArmPresets.push_back(oi->FRONT_HIGH_HATCH_POSITION);

    drivetrain->FollowPathInit(autoPathNames.at(path_count));
    arm->MoveArmToPosition(autoArmPresets.at(path_count), false, false, false);
    auto_state = FOLLOW_PATH_STATE;
  }
  else if (m_autoSelected == kAutoHatchLowLeftRocket)
  {
    oi->SetArmWristInManual(false, false);
    arm->UpdateArmAndWristInManual(false, false);

    autoPathNames.push_back("LeftPlatformToLeftFrontRocket");
    autoPathDirections.push_back(drivetrain->FORWARD);
    autoArmPresets.push_back(oi->FRONT_LOW_HATCH_POSITION);

    autoPathNames.push_back("LeftFrontRocketToLeftStation");
    autoPathDirections.push_back(drivetrain->REVERSE);
    autoArmPresets.push_back(oi->BACK_LOW_HATCH_POSITION);

    autoPathNames.push_back("LeftStationToLeftBackRocket"); //high on opposite side
    autoPathDirections.push_back(drivetrain->FORWARD);
    autoArmPresets.push_back(oi->FRONT_LOW_HATCH_POSITION);

    drivetrain->FollowPathInit(autoPathNames.at(path_count));
    arm->MoveArmToPosition(autoArmPresets.at(path_count), false, false, false);
    auto_state = FOLLOW_PATH_STATE;
  }
  else if (m_autoSelected == kAutoHatchHighRightRocket)
  {
    oi->SetArmWristInManual(false, false);
    arm->UpdateArmAndWristInManual(false, false);

    autoPathNames.push_back("RightPlatformToRightFrontRocket");
    autoPathDirections.push_back(drivetrain->FORWARD);
    autoArmPresets.push_back(oi->FRONT_HIGH_HATCH_POSITION);

    autoPathNames.push_back("RightFrontRocketToRightStation");
    autoPathDirections.push_back(drivetrain->REVERSE);
    autoArmPresets.push_back(oi->BACK_LOW_HATCH_POSITION);

    // autoPathNames.push_back("RightStationToRightBackRocket"); //high on opposite side
    // autoPathDirections.push_back(drivetrain->FORWARD);
    // autoArmPresets.push_back(oi->FRONT_HIGH_HATCH_POSITION);

    drivetrain->FollowPathInit(autoPathNames.at(path_count));
    arm->MoveArmToPosition(autoArmPresets.at(path_count), false, false, false);
    auto_state = FOLLOW_PATH_STATE;
  }
  else if (m_autoSelected == kAutoHatchLowRightRocket)
  {
    oi->SetArmWristInManual(false, false);
    arm->UpdateArmAndWristInManual(false, false);

    autoPathNames.push_back("RightPlatformToRightFrontRocket");
    autoPathDirections.push_back(drivetrain->FORWARD);
    autoArmPresets.push_back(oi->FRONT_LOW_HATCH_POSITION);

    autoPathNames.push_back("RightFrontRocketToRightStation");
    autoPathDirections.push_back(drivetrain->REVERSE);
    autoArmPresets.push_back(oi->BACK_LOW_HATCH_POSITION);

    autoPathNames.push_back("RightStationToRightBackRocket"); //high on opposite side
    autoPathDirections.push_back(drivetrain->FORWARD);
    autoArmPresets.push_back(oi->FRONT_LOW_HATCH_POSITION);

    drivetrain->FollowPathInit(autoPathNames.at(path_count));
    arm->MoveArmToPosition(autoArmPresets.at(path_count), false, false, false);
    auto_state = FOLLOW_PATH_STATE;
  }

  else if (m_autoSelected == kAutoRunTeleop)
  {
    TeleopPeriodic();
  }
  else
  {
    TeleopPeriodic();
  }
}

void Robot::AutonomousPeriodic()
{
  //Decide what to do if running auto
  if (m_autoSelected == kAutoHatchRightCargo)
  {
    AutoStateMachine();
  }
  else if (m_autoSelected == kAutoHatchLeftCargo)
  {
    AutoStateMachine();
  }
  else if (m_autoSelected == kAutoHatchHighRightRocket)
  {
    AutoStateMachine();
  }
  else if (m_autoSelected == kAutoHatchLowRightRocket)
  {
    AutoStateMachine();
  }
  else if (m_autoSelected == kAutoHatchHighLeftRocket)
  {
    AutoStateMachine();
  }
  else if (m_autoSelected == kAutoHatchLowLeftRocket)
  {
    AutoStateMachine();
  }
  else if (m_autoSelected == kAutoTest)
  {
    AutoStateMachine();
  }
  else if (m_autoSelected == kAutoRunTeleop)
  {
    TeleopPeriodic();
  }
  else
  {
    TeleopPeriodic();
  }
}

void Robot::AutoStateMachine()
{
  //switch to teleop if needed
  if (oi->GetStopLLMove())
  {
    auto_state = TELEOP_STATE;
  }

  //autononomous state machine, which has two steps
  //1. follow path and 2. execute a LL tracking sequence 
  switch (auto_state)
  {
  case FOLLOW_PATH_STATE:
  {
    //if arm delay timer is finished, move arm
    if (armMoveDelayTimer->Get() > ARM_MOVE_DELAY)
    {
      arm->MoveArmToPosition(autoArmPresets.at(path_count), false, false, false);
      oi->SetTargetArmPosition(autoArmPresets.at(path_count));
      oi->SetPlacingMode(false);
    }
    else if (armMoveDelayTimer->Get() < ARM_MOVE_DELAY && path_count != 0)
    {
      if (abs(arm->GetCurrentArmPosition()) < abs(lowestArmAngle))
      {
        lowestArmAngle = arm->GetCurrentArmPosition();
      }
      arm->MoveArmToPosition(lowestArmAngle, false, false, false);

      oi->SetTargetArmPosition(arm->GetCurrentArmPosition());
      oi->SetPlacingMode(false);
    }

    //follow path until it's done, then switch to Drive by LL state
    bool isPathDone = drivetrain->FollowPath(autoPathDirections[path_count]);
    if (isPathDone)
    {
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
    bool isLLFinished = drivetrain->AutoDrive(true, oi->GetLeftDriveInput(), oi->GetRightDriveInput(), false, false);

    if (isLLFinished == false)
    {
      arm->SetDesiredHatchGripperState(autoIsGripperClosed); //Keep grippers in desired state grippers
      ToggleGrippersTimer = 0;
      lowestArmAngle = arm->GetCurrentArmPosition();
    }
    if (isLLFinished) //toggle gripper, move back and switch states
    {
      arm->SetDesiredHatchGripperState(!autoIsGripperClosed); //Toggle Grippers
      ToggleGrippersTimer++;
      if (ToggleGrippersTimer > LOOPS_TO_TOGGLE_GRIPPER) // move backwards
      {
        drivetrain->AutoDriveBackwards(true, true); //Move backwards automatically

        //keep arm at lowest position seen
        if (abs(arm->GetCurrentArmPosition()) < abs(lowestArmAngle))
        {
          lowestArmAngle = arm->GetCurrentArmPosition();
        }
        arm->MoveArmToPosition(lowestArmAngle, false, false, false);

        oi->SetTargetArmPosition(lowestArmAngle);
        oi->SetPlacingMode(false);
      }
      if (ToggleGrippersTimer > LOOPS_TO_TOGGLE_GRIPPER + 30)
      {
        //update path counter
        path_count++;
        //if no more paths, go to teleop
        if (path_count >= autoPathNames.size())
        {
          auto_state = TELEOP_STATE;
        }
        else //otherwise, initialize next path, reset arm move timer, and go to path follow state
        {
          //update autoIsGripperClosed
          autoIsGripperClosed = !autoIsGripperClosed;
          //init new paths
          drivetrain->FollowPathInit(autoPathNames.at(path_count));
          armMoveDelayTimer->Reset();
          armMoveDelayTimer->Start();
          
          drivetrain->AutoDrive(false, oi->GetLeftDriveInput(), oi->GetRightDriveInput(), false, false);
          //change state
          auto_state = FOLLOW_PATH_STATE;
        }
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
  isBeforeMatch = false;

  //populate the shuffle board once after startup
  if (isShufflePopulated == false)
  {
    shufflemanager->TabInit();
    shufflemanager->VariableInit();
    isShufflePopulated = true;
  }
}

void Robot::TeleopPeriodic()
{
  led->IsHatchOpen(arm->GetIsGripperGripped(), (oi->GetDriveByLimelightPlace() || oi->GetDriveByLimelightPickup()));

  arm->UpdateArmAndWristInManual(oi->GetIsArmInManual(), oi->GetIsWristInManual());

  
  drivetrain->AutoDriveForward(oi->GetAutoDriveForward(), oi->GetVelocityTest());
  GetDesiredLLDistances(oi->GetTargetArmPosition());
  drivetrain->SetDesiredLLDistances(xDesiredInches, zDesiredInches);
  bool isLLFinished = drivetrain->AutoDrive((oi->GetDriveByLimelightPickup() || oi->GetDriveByLimelightPlace()), oi->GetLeftDriveInput(), oi->GetRightDriveInput(), oi->GetPlacingMode(), oi->GetStopLLMove());

  //Hatch Mode Limelight Arm/Gripper and backup of drivetrain control
  if (!oi->GetPlacingMode()) //not in ball mode, aka if is in hatch mode
  {
    //control the arm/gripper and backup of drive during auto LL pickup
    if (oi->GetDriveByLimelightPickup())
    {
      if (isLLFinished == false)
      {
        arm->SetDesiredHatchGripperState(false); //Open grippers
        ToggleGrippersTimer = 0;
      }
      if (isLLFinished)
      {
        arm->SetDesiredHatchGripperState(true); //Close grippers
        ToggleGrippersTimer++;
        if (ToggleGrippersTimer > LOOPS_TO_TOGGLE_GRIPPER)
        {
          if (!oi->GetPlacingMode()) //not in ball mode, aka if is in hatch mode
          {
            drivetrain->AutoDriveBackwards(true, true); //Move backwards automatically
          }
        }
      }
    }

    //control the arm/gripper and backup of drive during auto LL placing 
    if (oi->GetDriveByLimelightPlace())
    {
      if (isLLFinished == false)
      {
        arm->SetDesiredHatchGripperState(true); //Close grippers
        ToggleGrippersTimer = 0;
      }
      if (isLLFinished)
      {
        arm->SetDesiredHatchGripperState(false); //Open grippers
        ToggleGrippersTimer++;
        if (ToggleGrippersTimer > LOOPS_TO_TOGGLE_GRIPPER)
        {
          if (!oi->GetPlacingMode()) //not in ball mode, aka if is in hatch mode
          {
            drivetrain->AutoDriveBackwards(true, true); //Move backwards automatically
          }
        }
      }
    }
  }

  //Run wheels to eject ball when limelight tracking is finished
  if (isLLFinished)
  {
    arm->RunIntake(AUTO_RELEASE_SPEED); 
  }
  else //Otherwise manually control the ball intake wheels 
  {
    arm->RunIntake(oi->GetIntakeInput());
  }

  //Manual and automatic control of the arm and wrist
  arm->ManualRotateArm(oi->GetArmInput());
  arm->ManualRotateWrist(oi->GetWristInput());
  arm->MoveArmToPosition(oi->GetTargetArmPosition(), oi->GetPlacingMode(), oi->GetIsInBallPickup(), oi->IsInCargoShipMode());

  //Call the fourbar functions
  fourbar->ExtendOrRetract(oi->GetFourbarExtend(), oi->GetFourbarRetract());
  fourbar->FourbarHome(oi->GetFourbarHome());

  //Manual tank drive call
  drivetrain->TankDrive(oi->GetLeftDriveInput(), oi->GetRightDriveInput());

  //Call to gear shifters
  if (oi->SwitchGears())
  {
    drivetrain->SetGearShifter(oi->GetIsHighGear());
  }

  //Call to manually changing the hatch gripper
  if (oi->SwitchGripper())
  {
    arm->SetDesiredHatchGripperState(oi->GetIsGripperClosed());
  }
}

void Robot::DisabledInit()
{
  // alliance = frc::DriverStation::GetInstance().GetAlliance();
}

void Robot::DisabledPeriodic()
{
  //populate the shuffle board once after startup
  if (isShufflePopulated == false)
  {
    shufflemanager->TabInit();
    shufflemanager->VariableInit();
    isShufflePopulated = true;
  }

  //set the startup animation if before the match in disabled
  if (isBeforeMatch)
  {
    led->StartUp();
  }

  //set the shutdown animation if after the match in disabled
  if (!isBeforeMatch)
  {
    led->ShutDown();
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

  //set arm isFront based on the arm target position 
  if (armTargetPosition >= 0)
  {
    drivetrain->SetIsFrontLL(true);
  }
  else
  {
    drivetrain->SetIsFrontLL(false);
  }
}