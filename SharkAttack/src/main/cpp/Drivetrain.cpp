/*----------------------------------------------------------------------------------*/

//Methods for the Drivetrain class (any and all things drivetrain/locomotion)

/*----------------------------------------------------------------------------------*/

#include "Drivetrain.h"
#include "ShuffleManager.h"

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
  leftFront = new TalonSRX(LEFT_FRONT_ID);
  leftMid = new TalonSRX(LEFT_MID_ID);
  leftBack = new TalonSRX(LEFT_BACK_ID);
  rightFront = new TalonSRX(RIGHT_FRONT_ID);
  rightMid = new TalonSRX(RIGHT_MID_ID);
  rightBack = new TalonSRX(RIGHT_BACK_ID);

  //Initialize Double Solenoid
  gearShifter = new frc::DoubleSolenoid(0, 1);

  //Initialize Limelight
  limelight = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  //Set Talons to be in same direction
  leftFront->SetInverted(false);
  leftMid->SetInverted(true);
  leftBack->SetInverted(false);
  rightFront->SetInverted(true);
  rightMid->SetInverted(false);
  rightBack->SetInverted(true);

  //Set Coast Or Brake
  leftFront->SetNeutralMode(motorcontrol::NeutralMode::Coast);
  leftMid->SetNeutralMode(motorcontrol::NeutralMode::Coast);
  leftBack->SetNeutralMode(motorcontrol::NeutralMode::Brake);
  rightFront->SetNeutralMode(motorcontrol::NeutralMode::Coast);
  rightMid->SetNeutralMode(motorcontrol::NeutralMode::Coast);
  rightBack->SetNeutralMode(motorcontrol::NeutralMode::Brake);

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

  isInAutoDrive = false;
}

//Public Methods
void Drivetrain::PutData() {
//Send limelight and drivetrain variables to SD

  targetOffsetAngle_Horizontal = limelight->GetNumber("tx", 0.0);
  targetOffsetAngle_Vertical = limelight->GetNumber("ty", 0.0);
  targetArea = limelight->GetNumber("ta", 0.0);
  targetSkew = limelight->GetNumber("ts", 0.0);

  // frc::SmartDashboard::PutNumber("Heading", targetOffsetAngle_Horizontal);
  // frc::SmartDashboard::PutNumber("Skew", targetSkew);
  
  // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->VisionTab, "Heading", targetOffsetAngle_Horizontal);
  // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->VisionTab, "Skew", targetSkew);

  rightDashboardSpeed = rightBack->GetSelectedSensorVelocity(0) * NU_TO_FEET * SECONDS_TO_100MS;//rightDashboardSpeed = NU_TO_FEET;
  leftDashboardSpeed = leftBack->GetSelectedSensorVelocity(0) * NU_TO_FEET * SECONDS_TO_100MS;//leftDashboardSpeed = SECONDS_TO_100MS;

  // frc::SmartDashboard::PutNumber("RIGHT REAL SPEED", rightDashboardSpeed);
  // frc::SmartDashboard::PutNumber("LEFT REAL SPEED", leftDashboardSpeed);
  
  // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->VisionTab, "NU_PER_REV", targetSkew);

  //change these?
  // frc::SmartDashboard::PutNumber("Speed Error Right", desiredRightFPS - rightDashboardSpeed);
  // frc::SmartDashboard::PutNumber("Speed Error Left", desiredLeftFPS - leftDashboardSpeed);
  // frc::SmartDashboard::PutNumber("Ft-Sec-Right", rightDashboardSpeed);
  // frc::SmartDashboard::PutNumber("Ft-Sec-Left", leftDashboardSpeed);
  
  // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, "Ft-Sec-Right_PRE", rightDashboardSpeed);
  // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, "Ft-Sec-Left_PRE", leftDashboardSpeed);
  // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, "Ft-Sec-Right", rightDashboardSpeed);
  //ShuffleManager::GetShuffleVariable().;
  //ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, "Ft-Sec-Left", leftDashboardSpeed);

  // frc::SmartDashboard::PutNumber("NU-100ms Left", leftBack->GetSelectedSensorVelocity(0));
  // frc::SmartDashboard::PutNumber("NU-100ms Right", rightBack->GetSelectedSensorVelocity(0));
  // frc::SmartDashboard::PutNumber("Target Area", targetArea);

  currentDistanceInches = (TARGET_LOW_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES) / tan((LIMELIGHT_ANGLE + targetOffsetAngle_Vertical) * (M_PI / 180)); //current distance from target
  ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->testVal , currentDistanceInches);

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

  if(!isInAutoDrive){
    leftBack->Set(ControlMode::PercentOutput, leftValue);
    rightBack->Set(ControlMode::PercentOutput, rightValue);
    leftMid->Set(ControlMode::Follower, LEFT_BACK_ID);
    rightMid->Set(ControlMode::Follower, RIGHT_BACK_ID);
    leftFront->Set(ControlMode::Follower, LEFT_BACK_ID);
    rightFront->Set(ControlMode::Follower, RIGHT_BACK_ID);
  }
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

void Drivetrain::CheckSwitchGears(bool isHighGear) {

  if (isHighGear)
  {
    gearShifter->Set(frc::DoubleSolenoid::Value::kReverse);
  }
  else if (!isHighGear)
  {
    gearShifter->Set(frc::DoubleSolenoid::Value::kForward);
  }
}

void Drivetrain::AutoDriveForward(bool isBut, bool isVelocityControl){
  if (isBut && !isVelocityControl) {
    isInAutoDrive = true;

    leftBack->Set(ControlMode::PercentOutput, 0.5);
    rightBack->Set(ControlMode::PercentOutput, 0.5);
    leftMid->Set(ControlMode::Follower, LEFT_BACK_ID);
    rightMid->Set(ControlMode::Follower, RIGHT_BACK_ID);
    leftFront->Set(ControlMode::Follower, LEFT_BACK_ID);
    rightFront->Set(ControlMode::Follower, RIGHT_BACK_ID);

  }else if(isVelocityControl){
    isInAutoDrive = true;

    leftBack->Set(ControlMode::Velocity, 3 * FEET_TO_NU * CONVERT_100MS_TO_SECONDS); //in feet/s
    rightBack->Set(ControlMode::Velocity, 3 * FEET_TO_NU * CONVERT_100MS_TO_SECONDS);
    leftMid->Set(ControlMode::Follower, LEFT_BACK_ID);
    rightMid->Set(ControlMode::Follower, RIGHT_BACK_ID);
    leftFront->Set(ControlMode::Follower, LEFT_BACK_ID);
    rightFront->Set(ControlMode::Follower, RIGHT_BACK_ID);
  }
  else {
    isInAutoDrive = false;
  }
}