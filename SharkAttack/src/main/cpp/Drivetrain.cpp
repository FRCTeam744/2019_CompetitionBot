#include <ctre/phoenix.h>

#include "Tunables.h"
#include "Constants.h"
#include "Objects.h"
#include "Drivetrain.h"


double rightSpeed = 0.0;
double leftSpeed = 0.0;
double leftPower = 0.0;
double rightPower = 0.0;
double desiredRightFPS = 0.0;
double desiredLeftFPS = 0.0;

double realRightSpeedNUPer100ms = 0.0;
double realLeftSpeedNUPer100ms = 0.0;

double targetOffsetAngle_Horizontal;
double targetOffsetAngle_Vertical;
double targetArea;
double targetSkew;

double adjust = 0.0;

double currentDistanceInches = 0.0;

//Public Methods
void Drivetrain::Init()
{

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

  table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

void Drivetrain::Periodic()
{

  targetOffsetAngle_Horizontal = table->GetNumber("tx", 0.0);
  targetOffsetAngle_Vertical = table->GetNumber("ty", 0.0);
  targetArea = table->GetNumber("ta", 0.0);
  targetSkew = table->GetNumber("ts", 0.0);

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
  frc::SmartDashboard::PutNumber("Target Area", targetArea);

  currentDistanceInches = (TARGET_LOW_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES) / tan((LIMELIGHT_ANGLE + targetOffsetAngle_Vertical) * (M_PI / 180)); //current distance from target
  frc::SmartDashboard::PutNumber("current distance", currentDistanceInches);
}

void Drivetrain::Limelight(){
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

  
}

void Drivetrain::SetSensorPhase()
{
}




//Private Methods
