#include "Drivetrain.h"
//#include <frc/shuffleboard/Shuffleboard.h>


Drivetrain *Drivetrain::s_instance = 0;

//Static Singleton Method
Drivetrain *Drivetrain::GetInstance()
{
  if (s_instance == 0)
  {
    s_instance = new Drivetrain();
  }
  return s_instance;
}

//Constructor
Drivetrain::Drivetrain()
{
  //Establish Talons according to ID's
  leftFront = new TalonSRX(leftFrontID);
  leftMid = new TalonSRX(leftMidID);
  leftBack = new TalonSRX(leftBackID);
  rightFront = new TalonSRX(rightFrontID);
  rightMid = new TalonSRX(rightMidID);
  rightBack = new TalonSRX(rightBackID);

  //Initialize Double Solenoid
  gearShifter = new frc::DoubleSolenoid(0, 1);

  //Initialize Limelight
  limelight = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  //Set Talons to be in same direction
  leftFront->SetInverted(false);
  leftBack->SetInverted(false);
  rightFront->SetInverted(true);
  rightBack->SetInverted(true);
  rightMid->SetInverted(false);
  leftMid->SetInverted(true);

  //Set the sign of the encoder (true means switch sign)
  leftBack->SetSensorPhase(true);
  rightBack->SetSensorPhase(true);

  //configure Talon encoder
  armEncoderTalon = leftFront;
  armEncoderTalon->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute, 0, 0);
  wristEncoderTalon = rightFront;
  wristEncoderTalon->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute, 0, 0);

  //Config for the Talon internal PID loop for speedControl
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
}

//Public Methods
void Drivetrain::PutData() {
//Send limelight and drivetrain variables to SD
//VISION TAB PLS

  targetOffsetAngle_Horizontal = limelight->GetNumber("tx", 0.0);
  targetOffsetAngle_Vertical = limelight->GetNumber("ty", 0.0);
  targetArea = limelight->GetNumber("ta", 0.0);
  targetSkew = limelight->GetNumber("ts", 0.0);

  frc::SmartDashboard::PutNumber("Heading", targetOffsetAngle_Horizontal);
  frc::SmartDashboard::PutNumber("Skew", targetSkew);

  rightDashboardSpeed = rightBack->GetSelectedSensorVelocity(0) * NU_TO_FEET * SECONDS_TO_100MS;
  leftDashboardSpeed = leftBack->GetSelectedSensorVelocity(0) * NU_TO_FEET * SECONDS_TO_100MS;

  //rightDashboardSpeed = NU_TO_FEET;
  //leftDashboardSpeed = SECONDS_TO_100MS;

  frc::SmartDashboard::PutNumber("NU_PER_REV", NU_PER_REV);
  frc::SmartDashboard::PutNumber("CIRCUMFERENCE_INCHES", CIRCUMFERENCE_INCHES);

  frc::SmartDashboard::PutNumber("RADIUS_INCHES", RADIUS_INCHES);
  frc::SmartDashboard::PutNumber("INCHES_PER_REV", INCHES_PER_REV);
  frc::SmartDashboard::PutNumber("NU_TO_FEET", NU_TO_FEET);
  frc::SmartDashboard::PutNumber("FEET_TO_NU", FEET_TO_NU);
  frc::SmartDashboard::PutNumber("SECONDS_TO_100MS", SECONDS_TO_100MS);
  frc::SmartDashboard::PutNumber("CONVERT_100MS_TO_SECONDS", CONVERT_100MS_TO_SECONDS);

  frc::SmartDashboard::PutNumber("Speed Error Right", desiredRightFPS - rightDashboardSpeed);
  frc::SmartDashboard::PutNumber("Speed Error Left", desiredLeftFPS - leftDashboardSpeed);

  frc::SmartDashboard::PutNumber("Ft-Sec-Right", rightDashboardSpeed);
  frc::SmartDashboard::PutNumber("Ft-Sec-Left", leftDashboardSpeed);
  frc::SmartDashboard::PutNumber("NU-100ms Left", leftBack->GetSelectedSensorVelocity(0));
  frc::SmartDashboard::PutNumber("NU-100ms Right", rightBack->GetSelectedSensorVelocity(0));
  frc::SmartDashboard::PutNumber("Target Area", targetArea);

  currentDistanceInches = (TARGET_LOW_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES) / tan((LIMELIGHT_ANGLE + targetOffsetAngle_Vertical) * (M_PI / 180)); //current distance from target
  frc::SmartDashboard::PutNumber("current distance", currentDistanceInches);
}

void Drivetrain::AutoDrive() {
  /*
  if (xbox->GetStartButton())
  {
    double p_dist_loop = 0;
    // double currentDistanceInches = (TARGET_LOW_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES) / tan((LIMELIGHT_ANGLE + targetOffsetAngle_Vertical) * (M_PI/180)); //current distance from target

    //Target is to the left of the Robot
    if (targetOffsetAngle_Horizontal < -1.0)
    {
      adjust = kP_ANGLE * targetOffsetAngle_Horizontal;
    }
    //Target is to the right of the Robot
    else if (targetOffsetAngle_Horizontal > 1.0)
    {
      adjust = kP_ANGLE * targetOffsetAngle_Horizontal;
    }

    p_dist_loop = kP_DIST * (DESIRED_DISTANCE_INCHES - currentDistanceInches);

    leftPower = adjust + p_dist_loop;
    rightPower = -adjust + p_dist_loop;

    leftBack->Set(ControlMode::Velocity, leftPower);
    leftFront->Set(ControlMode::Follower, LEFT_TALON_MASTER);
    rightBack->Set(ControlMode::Velocity, rightPower);
    rightFront->Set(ControlMode::Follower, RIGHT_TALON_MASTER);
    // frc::SmartDashboard::PutNumber("current distance", currentDistanceInches);
    // driveTrain->TankDrive(leftPower, rightPower, false);
  }
  else if (xbox->GetBackButton())
  {

    desiredLeftFPS = desiredRightFPS = 2.0;

    leftBack->Set(ControlMode::Velocity, desiredLeftFPS * FEET_TO_NU * CONVERT_100MS_TO_SECONDS);
    leftFront->Set(ControlMode::Follower, LEFT_TALON_MASTER);
    rightBack->Set(ControlMode::Velocity, desiredRightFPS * FEET_TO_NU * CONVERT_100MS_TO_SECONDS);
    rightFront->Set(ControlMode::Follower, RIGHT_TALON_MASTER);
  }
  
  //looking at what type of drive system is active based on buttons
  else if (!arcadeDrive)
  {
    if (driveWithXbox)
    {

      leftPower = -xbox->GetY(leftHand);
      rightPower = -xbox->GetY(rightHand);

      // speedTankDrive(xbox->GetY(leftHand), xbox->GetY(rightHand));
      leftBack->Set(ControlMode::PercentOutput, leftPower);
      leftFront->Set(ControlMode::Follower, LEFT_TALON_MASTER);
      rightBack->Set(ControlMode::PercentOutput, rightPower);
      rightFront->Set(ControlMode::Follower, RIGHT_TALON_MASTER);
    }
    else
    {

      leftPower = -leftStick->GetY();
      rightPower = -rightStick->GetY();

      // speedTankDrive(leftStick->GetY(), rightStick->GetY());
      leftBack->Set(ControlMode::PercentOutput, leftPower);
      leftFront->Set(ControlMode::Follower, LEFT_TALON_MASTER);
      rightBack->Set(ControlMode::PercentOutput, rightPower);
      rightFront->Set(ControlMode::Follower, RIGHT_TALON_MASTER);
    }
  }
  else
  {
    if (driveWithXbox)
    {
      // driveTrain->ArcadeDrive(xbox->GetY(leftHand), xbox->GetX(rightHand), false);
    }
    else
    {
      // driveTrain->ArcadeDrive(leftStick->GetY(), rightStick->GetX(), false);
    }
  }

  frc::SmartDashboard::PutNumber("Left Power", leftPower);
  frc::SmartDashboard::PutNumber("Right Power", rightPower);
  */
}

void Drivetrain::TankDrive(double leftValue, double rightValue) {

  leftBack->Set(ControlMode::PercentOutput, leftValue);
  rightBack->Set(ControlMode::PercentOutput, rightValue);
  leftMid->Set(ControlMode::Follower, 27.0);
  rightMid->Set(ControlMode::Follower, 26.0);
  leftFront->Set(ControlMode::Follower, 27.0);
  rightFront->Set(ControlMode::Follower, 26.0);
}

// Use these methods in other classes to interact with the limelight
void Drivetrain::LimelightSet(std::tuple <bool, std::string, double> data) {
  if (std::get<0>(data)){
    limelight->PutNumber(std::get<1>(data), std::get<2>(data));
  }
}

double Drivetrain::LimelightGet(std::string key) {
  return limelight->GetNumber(key, 0.0);
}

double Drivetrain::GetArmEncoderValue()
{
  return armEncoderTalon->GetSelectedSensorPosition();
}

double Drivetrain::GetWristEncoderValue()
{
  return wristEncoderTalon->GetSelectedSensorPosition();
}

void Drivetrain::CheckSwitchGears(bool isHighGear)
{

  if (isHighGear)
  {
    gearShifter->Set(frc::DoubleSolenoid::Value::kReverse);
  }
  else if (!isHighGear)
  {
    gearShifter->Set(frc::DoubleSolenoid::Value::kForward);
  }
}

// void Drivetrain::PutOnShuffleboard()
// {
//   if (isInitialized == false)
//   {
//     frc::ShuffleboardTab &PreCompTab = frc::Shuffleboard::GetTab("Pre-Comp Check");
//     frc::ShuffleboardTab &ArmWristtab = frc::Shuffleboard::GetTab("Arm&Wrist Debug");
//     //PreCompTab.Add("Arm Encoder Val", drivetrain->GetArmEncoderValue());

//     isInitialized = true;
//   }
// }