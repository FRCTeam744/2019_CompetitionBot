
#include "Drivetrain.h"

Drivetrain* Drivetrain::s_instance = 0;

//Public Methods

Drivetrain* Drivetrain::getInstance() {
  if (s_instance == 0){
    s_instance = new Drivetrain();
  }
  return s_instance;
}

Drivetrain::Drivetrain() {
  //Establish Talons according to ID's
  leftFront = new TalonSRX(23);
  leftBack = new TalonSRX(27);
  rightFront = new TalonSRX(12);
  rightBack = new TalonSRX(22);

  //Set Talons to be in same direction
  leftFront->SetInverted(false);
  leftBack->SetInverted(false);
  rightFront->SetInverted(true);
  rightBack->SetInverted(true);

  // //Set the sign of the encoder (true means switch sign)
  leftBack->SetSensorPhase(true);
  rightBack->SetSensorPhase(true);


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

  limelight = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

}

void Drivetrain::Periodic() {
// Set limelight and drivetrain variables to SD
  targetOffsetAngle_Horizontal = limelight->GetNumber("tx", 0.0);
  targetOffsetAngle_Vertical = limelight->GetNumber("ty", 0.0);
  targetArea = limelight->GetNumber("ta", 0.0);
  targetSkew = limelight->GetNumber("ts", 0.0);

  frc::SmartDashboard::PutNumber("Heading", targetOffsetAngle_Horizontal);
  frc::SmartDashboard::PutNumber("Skew", targetSkew);

  rightDashboardSpeed = leftBack->GetSelectedSensorVelocity(0) * NU_TO_FEET * SECONDS_TO_100MS;
  leftDashboardSpeed = rightBack->GetSelectedSensorVelocity(0) * NU_TO_FEET * SECONDS_TO_100MS;

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

void Drivetrain::TankDrive (double leftValue, double rightValue) {

  leftBack->Set(ControlMode::PercentOutput, leftValue);
  rightBack->Set(ControlMode::PercentOutput, rightValue);
  leftMid->Set(ControlMode::Follower, 27);
  rightMid->Set(ControlMode::Follower, 26);
  leftFront->Set(ControlMode::Follower, 27);
  rightFront->Set(ControlMode::Follower, 26);
}

// Use these methods in other classes to interact with the limelight
void Drivetrain::LimelightPut(std::string key, int value) {

  limelight->PutNumber(key, 0.0);
}

double Drivetrain::LimelightGet(std::string key){

  return limelight->GetNumber(key, 0.0);
}

// Private Methods
