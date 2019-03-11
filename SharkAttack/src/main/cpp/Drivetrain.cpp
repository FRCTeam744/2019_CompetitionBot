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

    //Initialize limelight
    limelightFront = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front");
    limelightBack = nt::NetworkTableInstance::GetDefault().GetTable("limelight-back");

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
    isInLLDrive = false;
}

//Public Methods
void Drivetrain::PutData()
{
    //Send limelightFront and drivetrain variables to SD

    targetOffsetAngle_Horizontal = limelightFront->GetNumber("tx", 0.0);
    targetOffsetAngle_Vertical = limelightFront->GetNumber("ty", 0.0);
    targetArea = limelightFront->GetNumber("ta", 0.0);
    targetSkew = limelightFront->GetNumber("ts", 0.0);

    // frc::SmartDashboard::PutNumber("Heading", targetOffsetAngle_Horizontal);
    // frc::SmartDashboard::PutNumber("Skew", targetSkew);

    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->VisionTab, "Heading", targetOffsetAngle_Horizontal);
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->VisionTab, "Skew", targetSkew);

    rightDashboardSpeed = rightBack->GetSelectedSensorVelocity(0) * NU_TO_FEET * SECONDS_TO_100MS; //rightDashboardSpeed = NU_TO_FEET;
    leftDashboardSpeed = leftBack->GetSelectedSensorVelocity(0) * NU_TO_FEET * SECONDS_TO_100MS;   //leftDashboardSpeed = SECONDS_TO_100MS;

    // frc::SmartDashboard::PutNumber("RIGHT REAL SPEED", rightDashboardSpeed);
    // frc::SmartDashboard::PutNumber("LEFT REAL SPEED", leftDashboardSpeed);

    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->VisionTab, "NU_PER_REV", targetSkew);

    //change these?
    frc::SmartDashboard::PutNumber("Speed Error Right", desiredRightFPS - rightDashboardSpeed);
    frc::SmartDashboard::PutNumber("Speed Error Left", desiredLeftFPS - leftDashboardSpeed);
    frc::SmartDashboard::PutNumber("Ft-Sec-Right", rightDashboardSpeed);
    frc::SmartDashboard::PutNumber("Ft-Sec-Left", leftDashboardSpeed);

    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, "Ft-Sec-Right_PRE", rightDashboardSpeed);
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, "Ft-Sec-Left_PRE", leftDashboardSpeed);
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, "Ft-Sec-Right", rightDashboardSpeed);
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, "Ft-Sec-Left", leftDashboardSpeed);

    frc::SmartDashboard::PutNumber("NU-100ms Left", leftBack->GetSelectedSensorVelocity(0));
    frc::SmartDashboard::PutNumber("NU-100ms Right", rightBack->GetSelectedSensorVelocity(0));
    // frc::SmartDashboard::PutNumber("Target Area", targetArea);

    //current distance from target
    // currentDistanceInches = (TARGET_LOW_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES) / tan((LIMELIGHT_ANGLE + targetOffsetAngle_Vertical) * (M_PI / 180)); 
    //frc::SmartDashboard::PutNumber("current distance", currentDistanceInches);
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->VisionTab, "Current Distance", currentDistanceInches);
}

void Drivetrain::AutoDrive()
{
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

void Drivetrain::TankDrive(double leftValue, double rightValue)
{
    if (!isInAutoDrive && !isInLLDrive)
    {
        leftBack->Set(ControlMode::PercentOutput, leftValue);
        rightBack->Set(ControlMode::PercentOutput, rightValue);
        leftMid->Set(ControlMode::PercentOutput, leftValue);
        rightMid->Set(ControlMode::PercentOutput, rightValue);
        leftFront->Set(ControlMode::PercentOutput, leftValue);
        rightFront->Set(ControlMode::PercentOutput, rightValue);
    }
}

// Use these methods in other classes to interact with the limelightFront
void Drivetrain::LimelightSet(std::tuple<bool, std::string, double> data)
{
    if (std::get<0>(data))
    {
        limelightFront->PutNumber(std::get<1>(data), std::get<2>(data));
    }
}

double Drivetrain::LimelightGet(std::string key)
{
    return limelightFront->GetNumber(key, 0.0);
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

void Drivetrain::AutoDriveForward(bool isBut, bool isVelocityControl)
{
    if (isBut && !isVelocityControl)
    {
        isInAutoDrive = true;

        leftBack->Set(ControlMode::PercentOutput, TEST_PERCENT_OUTPUT);
        rightBack->Set(ControlMode::PercentOutput, TEST_PERCENT_OUTPUT);
        leftMid->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightMid->Set(ControlMode::Follower, RIGHT_BACK_ID);
        leftFront->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightFront->Set(ControlMode::Follower, RIGHT_BACK_ID);
    }
    else if (isVelocityControl)
    {
        isInAutoDrive = true;

        desiredLeftFPS = desiredRightFPS = 2.0;

        leftBack->Set(ControlMode::Velocity, desiredLeftFPS * FEET_TO_NU * CONVERT_100MS_TO_SECONDS); //in feet/s
        rightBack->Set(ControlMode::Velocity, desiredRightFPS * FEET_TO_NU * CONVERT_100MS_TO_SECONDS);
        leftMid->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightMid->Set(ControlMode::Follower, RIGHT_BACK_ID);
        leftFront->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightFront->Set(ControlMode::Follower, RIGHT_BACK_ID);
    }
    else
    {
        isInAutoDrive = false;
    }
}

void Drivetrain::AutoDriveLL(bool wantLimelight, bool isHatch, bool isMid, bool isFront, double leftTank, double rightTank)
{
    if (!wantLimelight)
    {
        isInLLDrive = false;
        limelightFront->PutNumber("pipeline", 1.0);
        limelightBack->PutNumber("pipeline", 1.0);
        return;
    }

    double X;
    double Y;
    double Z;
    double roll;
    double pitch;
    double yaw;

    if (!isInLLDrive) //init
    {
        //set limelight pipeline and turn on leds, once
        if (isFront)
        {
            limelightFront->PutNumber("pipeline", 0.0);
        }
        else
        {
            limelightBack->PutNumber("pipeline", 0.0);
        }
        isInLLDrive = true;

        X = 0;
        Y = 0;
        Z = 0;
        roll = 0;
        pitch = 0;
        yaw = 0;
        prevX = 0;
        prevY = 0;
        prevZ = 0;
        prevRoll = 0;
        prevPitch = 0;
        prevYaw = 0;
        
    }

    if (isFront)
    {  
        if(limelightFront->GetNumber("tv", 0.0) > 0 && limelightFront->GetNumber("getpipe", 0.0) == 0.0) {
            limelightPose = limelightFront->GetNumberArray("camtran", 0.0);
        }
        else {
            return;
        }
    }
    else
    {
        if(limelightBack->GetNumber("tv", 0.0) > 0 && limelightBack->GetNumber("getpipe", 0.0) == 0.0) {
            limelightPose = limelightBack->GetNumberArray("camtran", 0.0);
        }
        else {
            return;
        }    
    }

    try {
        X = limelightPose.at(0);   frc::SmartDashboard::PutNumber("X", X);
        Y = limelightPose.at(1);   frc::SmartDashboard::PutNumber("Y", Y);
        Z = limelightPose.at(2);   frc::SmartDashboard::PutNumber("Z", Z);
        roll = limelightPose.at(3);   frc::SmartDashboard::PutNumber("roll", roll);
        pitch = limelightPose.at(4);   frc::SmartDashboard::PutNumber("pitch", pitch);
        yaw = limelightPose.at(5);  frc::SmartDashboard::PutNumber("yaw", yaw);
        
        if(X==0 && Y==0 && Z==0 && roll==0 && pitch==0 && yaw==0) {
            //no signal from solvepnp, what do?
            isInLLDrive = false;
            TankDrive(leftTank, rightTank);
            isInLLDrive = true;
            return;  
        }
        if(prevX==0 && prevY==0 && prevZ==0 && prevRoll==0 && prevPitch==0 && prevYaw==0) {
            //first run through - no filter
        } else{
            X = alpha*X + (1-alpha)*prevX;
            Y = alpha*Y + (1-alpha)*prevY;
            // Z = alpha*Z + (1-alpha)*prevZ;
            roll = alpha*roll + (1-alpha)*prevRoll;
            pitch = alpha*pitch + (1-alpha)*prevPitch;
            yaw = alpha*yaw + (1-alpha)*prevYaw;
            if(abs(X-prevX) > 5) { X = prevX; }
            if(abs(Y-prevY) > 5) { Y = prevY; }
            if(abs(Z-prevZ) > 5) { Z = prevZ; }
            if(abs(roll-prevRoll) > 5) { roll = prevRoll; }
            if(abs(yaw-prevYaw) > 5) { yaw = prevYaw; }
            if(abs(pitch-prevPitch) > 5) { pitch = prevPitch; }
        }

        frc::SmartDashboard::PutNumber("Filter X", X);
        frc::SmartDashboard::PutNumber("Filter Y", Y);
        frc::SmartDashboard::PutNumber("Filter Z", Z);
        frc::SmartDashboard::PutNumber("Filter roll", roll);
        frc::SmartDashboard::PutNumber("Filter pitch", pitch);
        frc::SmartDashboard::PutNumber("Filter yaw", yaw);

        prevX = X;
        prevY = Y;
        prevZ = Z;
        prevRoll = roll;
        prevPitch = pitch;
        prevYaw = yaw;
    } catch(...) {
        return;
    }

    if(X==0 && Y==0 && Z==0 && roll==0 && pitch==0 && yaw==0) {
      //no signal from solvepnp, what do?
        return;  
    }

    // //get x/y and z desired
    double xDesiredInches = 0+LL_FRONT_X_OFFSET;
    double zDesiredInches = -40;

    double xErrorInches_robot = (xDesiredInches - X);
    double zErrorInches = zDesiredInches - Z;

    // // if()
    
    double thetaDesired_Robot = xErrorInches_robot * kP_THETA_DESIRED;
    if(thetaDesired_Robot > 30){
        thetaDesired_Robot = 30;
    } else if (thetaDesired_Robot < -30) {
        thetaDesired_Robot = -30;
    }
    double thetaError_Robot = (thetaDesired_Robot - (pitch - LL_FRONT_THETA_OFFSET));    

    double forwardSpeed = zErrorInches*kP_FORWARD;
    if(forwardSpeed > .3) {
        forwardSpeed = .3;
    }
    else if (forwardSpeed < -.3) {
        forwardSpeed = -.3;
    }
    double adjustment = thetaError_Robot*kP_THETA;

    frc::SmartDashboard::PutNumber("Desired Theta (Robot)", thetaDesired_Robot);
    frc::SmartDashboard::PutNumber("Theta Error (Robot)", thetaError_Robot);
    frc::SmartDashboard::PutNumber("X Error (Robot)", xErrorInches_robot);
    frc::SmartDashboard::PutNumber("Z Error", zErrorInches);
    

    if(abs(zErrorInches) < 1 && abs(xErrorInches_robot) < 2 && abs(thetaDesired_Robot) < 5) {
        leftBack->Set(ControlMode::PercentOutput, 0);
        rightBack->Set(ControlMode::PercentOutput, 0);
        leftMid->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightMid->Set(ControlMode::Follower, RIGHT_BACK_ID);
        leftFront->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightFront->Set(ControlMode::Follower, RIGHT_BACK_ID);
    }
    else{
        leftBack->Set(ControlMode::PercentOutput, forwardSpeed - adjustment);
        rightBack->Set(ControlMode::PercentOutput, forwardSpeed + adjustment);
        leftMid->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightMid->Set(ControlMode::Follower, RIGHT_BACK_ID);
        leftFront->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightFront->Set(ControlMode::Follower, RIGHT_BACK_ID);
    }
}