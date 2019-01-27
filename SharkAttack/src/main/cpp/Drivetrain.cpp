#include <ctre/phoenix.h>

double rightSpeed = 0.0;
double leftSpeed = 0.0;

double currentDistanceInches = 0.0;

double desiredRightFPS = 0.0;
double desiredLeftFPS = 0.0;

double realRightSpeedNUPer100ms = 0.0;
double realLeftSpeedNUPer100ms = 0.0;


double rightSpeed = 0.0;

  double targetOffsetAngle_Horizontal;
  double targetOffsetAngle_Vertical;
  double targetArea;
  double targetSkew;

  double leftPower = 0.0;
  double rightPower = 0.0;
  double adjust = 0.0;
  double leftSpeed = 0.0;

class Drivetrain {
    
    //Instance Variables




    //Public Methods





    //Private Methods


}
void DriveTrain::DrivetrainInit() {

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

void SetSensorPhase(){
    
}

void Drivetrain::RobotPeriodic() {

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
  frc::SmartDashboard::PutNumber("Target Area", targetArea);

  currentDistanceInches = (TARGET_LOW_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES) / tan((LIMELIGHT_ANGLE + targetOffsetAngle_Vertical) * (M_PI/180)); //current distance from target
  frc::SmartDashboard::PutNumber("current distance", currentDistanceInches);

}