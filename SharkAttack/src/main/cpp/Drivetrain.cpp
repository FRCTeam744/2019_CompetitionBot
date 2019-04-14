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
    gearShifter = new frc::Solenoid(0);

    //Initialize Timer for end of LL drive
    hatchPlaceTimer = new frc::Timer();
    angleDGainTimer = new frc::Timer();

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

    //Config for Talon Loop Ramp Rates in seconds
    leftFront->ConfigClosedloopRamp(talonRampRate);
    leftMid->ConfigClosedloopRamp(talonRampRate);
    leftBack->ConfigClosedloopRamp(talonRampRate);
    rightFront->ConfigClosedloopRamp(talonRampRate);
    rightMid->ConfigClosedloopRamp(talonRampRate);
    rightBack->ConfigClosedloopRamp(talonRampRate);
    leftFront->ConfigOpenloopRamp(talonRampRate);
    leftMid->ConfigOpenloopRamp(talonRampRate);
    leftBack->ConfigOpenloopRamp(talonRampRate);
    rightFront->ConfigOpenloopRamp(talonRampRate);
    rightMid->ConfigOpenloopRamp(talonRampRate);
    rightBack->ConfigOpenloopRamp(talonRampRate);

    isTargetAcquired = false;

    xDesiredInches = 0;
    zDesiredInches = 36;
    slopeForAngleCalc = 0.0;
    interceptForAngleCalc = 0.0;
    crosshairAngle = 0.0;
    prevT = 0;

    //Gyro
    ahrs = new AHRS(SerialPort::Port::kUSB);
}

//Public Methods
void Drivetrain::PrintDriveShuffleInfo()
{
    //Send limelight and drivetrain variables to SB

    // targetOffsetAngle_Horizontal = limelightFront->GetNumber("tx", 0.0);
    // targetOffsetAngle_Vertical = limelightFront->GetNumber("ty", 0.0);
    // targetArea = limelightFront->GetNumber("ta", 0.0);
    // targetSkew = limelightFront->GetNumber("ts", 0.0);

    // frc::SmartDashboard::PutNumber("Heading", targetOffsetAngle_Horizontal);
    // frc::SmartDashboard::PutNumber("Skew", targetSkew);
    //   ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->VisionTab, ShuffleManager::GetInstance()->headingVision, targetOffsetAngle_Horizontal);
    //   ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->VisionTab, ShuffleManager::GetInstance()->skewVision, targetSkew);

    rightDashboardSpeed = rightBack->GetSelectedSensorVelocity(0) * NU_TO_FEET * SECONDS_TO_100MS; //rightDashboardSpeed = NU_TO_FEET;
    leftDashboardSpeed = leftBack->GetSelectedSensorVelocity(0) * NU_TO_FEET * SECONDS_TO_100MS;   //leftDashboardSpeed = SECONDS_TO_100MS;

    frc::SmartDashboard::PutNumber("Right Speed FPS", rightDashboardSpeed);
    frc::SmartDashboard::PutNumber("Left Speed FPS", leftDashboardSpeed);

    // frc::SmartDashboard::PutNumber("NU-100ms Left", leftBack->GetSelectedSensorVelocity(0));
    // frc::SmartDashboard::PutNumber("NU-100ms Right", rightBack->GetSelectedSensorVelocity(0));
    // frc::SmartDashboard::PutNumber("Target Area", targetArea);

    // old, didnt know if needed ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->VisionTab, "NU_PER_REV", targetSkew);

    frc::SmartDashboard::PutNumber("Speed Error Right", desiredRightFPS - rightDashboardSpeed);
    frc::SmartDashboard::PutNumber("Speed Error Left", desiredLeftFPS - leftDashboardSpeed);
    //   ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, ShuffleManager::GetInstance()->speedErrorRightPreComp, desiredRightFPS - rightDashboardSpeed);
    //   ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, ShuffleManager::GetInstance()->speedErrorLeftPreComp, desiredLeftFPS - leftDashboardSpeed);

    //   ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->rightDrivePreComp , rightDashboardSpeed);
    //   ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->leftDrivePreComp , rightDashboardSpeed);
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->rightDriveVision, rightDashboardSpeed);
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->leftDriveVision, leftDashboardSpeed);
    //ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->limeLightAngleErrorVision, angleError);
    //ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->limeLightDistanceErrorVision, distanceError);
    //   currentDistanceInches = (TARGET_LOW_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES) / tan((LIMELIGHT_ANGLE + targetOffsetAngle_Vertical) * (M_PI / 180)); //current distance from target
    //   ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->VisionTab, ShuffleManager::GetInstance()->currentDistanceInchesVision , currentDistanceInches);

    // //Gyro
    SmartDashboard::PutBoolean("IMU_Connected", ahrs->IsConnected());
    SmartDashboard::PutNumber("IMU_Yaw", ahrs->GetYaw());
    SmartDashboard::PutBoolean("IMU_IsCalibrating", ahrs->IsCalibrating());
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->gyroYaw, ahrs->GetYaw());

    // std::cout << "IMU_Connected: " << ahrs->IsConnected() << std::endl;
    // std::cout << "Gyro Yaw: " << ahrs->GetYaw() << std::endl;
    // std::cout << "IMU_IsCalibrating: " << ahrs->IsCalibrating() << std::endl;
}

std::string Drivetrain::get_trajectory_file(std::string name)
{
    wpi::SmallString<256> path;
    frc::filesystem::GetDeployDirectory(path);
    wpi::sys::path::append(path, "paths", name + ".pf1.csv");

    std::cout << "Path: " << path << std::endl;
    if (!wpi::sys::fs::exists(path))
    {
        throw std::runtime_error("Path " + std::string(path.c_str()) + " does not exist!");
    }
    return std::string(path.c_str());
}

int Drivetrain::get_trajectory(std::string name, Segment *traj_out)
{
    FILE *fp = fopen(get_trajectory_file(name).c_str(), "r");
    int len = pathfinder_deserialize_csv(fp, traj_out);
    fclose(fp);
    return len;
}

double Drivetrain::pathfinder_follow_encoder(Segment s, int trajectory_length)
{
    return s.velocity;
}

void Drivetrain::RobotInit()
{
    angleDGainTimer->Reset();
    angleDGainTimer->Start();
    prevT = 0;
}

void Drivetrain::AutonomousInit()
{
    ahrs->ZeroYaw();
}

void Drivetrain::FollowPathInit(std::string pathName)
{
    left_trajectory_length = get_trajectory(pathName + ".right", leftTrajectory);  //This is supposed to be flipped! This is a bug in FRC's libraries
    right_trajectory_length = get_trajectory(pathName + ".left", rightTrajectory); //This is supposed to be flipped! This is a bug in FRC's libraries
    follow_path_counter = 0;
}

bool Drivetrain::FollowPath(bool isReverse)
{
    if (follow_path_counter < (left_trajectory_length - PATH_CUTOFF_TIME/0.02)) //adjust if needed 
    {
        // std::cout << "leftHeading: " << leftTrajectory[follow_path_counter].heading << std::endl;
        // std::cout << "rightHeading: " << rightTrajectory[follow_path_counter].heading << std::endl;

        double leftVelocity = 0;
        double rightVelocity = 0;

        double gyro_heading = ahrs->GetYaw();
        double desired_heading = r2d(leftTrajectory[follow_path_counter].heading);

        if (isReverse)
        {
            leftVelocity = -1 * rightTrajectory[follow_path_counter].velocity;
            rightVelocity = -1 * leftTrajectory[follow_path_counter].velocity;
            // turn *= -1;
            desired_heading -= 180;
        }
        else
        {
            leftVelocity = leftTrajectory[follow_path_counter].velocity;
            rightVelocity = rightTrajectory[follow_path_counter].velocity;
        }

        double angle_difference = desired_heading - gyro_heading;
        frc::SmartDashboard::PutNumber("Follow Path Desired Heading", desired_heading);
        angle_difference = std::fmod(angle_difference, 360.0);
        if (std::abs(angle_difference) > 180.0)
        {
            angle_difference = (angle_difference > 0) ? angle_difference - 360 : angle_difference + 360;
        }
        frc::SmartDashboard::PutNumber("Follow Path Angle Error", angle_difference);

        double turn = 0.1 * angle_difference; //was -0.002
        frc::SmartDashboard::PutNumber("Follow Path Turn", turn);

        leftBack->Set(ControlMode::Velocity, ((leftVelocity + turn) * FEET_TO_NU * CONVERT_100MS_TO_SECONDS)); //in feet/s
        rightBack->Set(ControlMode::Velocity, ((rightVelocity - turn) * FEET_TO_NU * CONVERT_100MS_TO_SECONDS));
        leftMid->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightMid->Set(ControlMode::Follower, RIGHT_BACK_ID);
        leftFront->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightFront->Set(ControlMode::Follower, RIGHT_BACK_ID);

        follow_path_counter++;
        return false;
    }
    else
    {
        StopMotors();
        return true;
    }
}

bool Drivetrain::AutoDrive(bool wantLimelight, double leftTank, double rightTank, bool isBallMode, bool wantToNotMove)
{
    if (!wantLimelight) //when not limelight tracking
    {
        if (isInLLDrive)
        {
            limelightFront->PutNumber("pipeline", DRIVER_PIPELINE);
            limelightBack->PutNumber("pipeline", DRIVER_PIPELINE);
        }
        isInLLDrive = false;
        wantToDriveHatchInPlace = false;
        return false;
    }
    double currentTime = angleDGainTimer->Get();

    //drive hatch into place, and skip LL code if you've gotten near the target
    if (wantToDriveHatchInPlace)
    {
        if (hatchPlaceTimer->Get() > 0.4)
        {
            if (StopMotorsTimer == 0)
            {
                StopMotors();
                StopMotorsTimer++;
            }
            return true;
        }
        else
        {
            StopMotorsTimer = 0;
            AutoDriveForward(false, true);
            return false;
        }
    }

    if (!isInLLDrive) //init pipelines
    {
        //set limelight pipeline and turn on leds, once
        if (isFront)
        {
            limelightFront->PutNumber("pipeline", pipelineNumber);
        }
        else
        {
            limelightBack->PutNumber("pipeline", pipelineNumber);
        }
        isInLLDrive = true;
    }

    if (isFront)
    {
        tv = limelightFront->GetNumber("tv", 0.0);
        actualPipeline = tv = limelightFront->GetNumber("pipeline", 0.0);
        targetOffsetAngle_Horizontal = limelightFront->GetNumber("tx", 0.0);
        targetOffsetAngle_Vertical = limelightFront->GetNumber("ty", 0.0);
        currentArea = limelightFront->GetNumber("ta", 0.0);
        targetSkew = limelightFront->GetNumber("ts", 0.0);
        // frc::SmartDashboard::PutNumber("Num Targets", tv);
        // frc::SmartDashboard::PutNumber("Actual Pipeline", pipelineNumber);
        // frc::SmartDashboard::PutNumber("Angle Offset (tx)", targetOffsetAngle_Horizontal);
        // frc::SmartDashboard::PutNumber("ty", targetOffsetAngle_Vertical);
        // frc::SmartDashboard::PutNumber("Current Area", currentArea);
        // frc::SmartDashboard::PutNumber("targetSkew", targetSkew);

        //wait for pipeline change and target acquisition
        if (limelightFront->GetNumber("tv", 0.0) == 0 || limelightFront->GetNumber("getpipe", 0.0) == DRIVER_PIPELINE)
        {
            IsTargetNotAcquired(leftTank, rightTank);
            return false;
        }

        //only 1 valid target
        if (limelightFront->GetNumber("tv", 0.0) > 0 && limelightFront->GetNumber("getpipe", 0.0) != DRIVER_PIPELINE && !isTargetAcquired)
        {
            isTargetAcquired = true;
            overrideEnabled = false;
            counter = 0;
            limelightFront->PutNumber("pipeline", pipelineNumber);
            accumAngleError = 0;
            prevDistanceError = 100;
            prevAngleError = 100;
            prevDistance = 0;
            prevAngle = 0;
        }
    }
    else
    {
        tv = limelightBack->GetNumber("tv", 0.0);
        actualPipeline = tv = limelightBack->GetNumber("pipeline", 0.0);
        targetOffsetAngle_Horizontal = limelightBack->GetNumber("tx", 0.0);
        targetOffsetAngle_Vertical = limelightBack->GetNumber("ty", 0.0);
        currentArea = limelightBack->GetNumber("ta", 0.0);
        targetSkew = limelightBack->GetNumber("ts", 0.0);

        // frc::SmartDashboard::PutNumber("Num Targets", tv);
        // frc::SmartDashboard::PutNumber("Actual Pipeline", pipelineNumber);
        // frc::SmartDashboard::PutNumber("Angle Offset", targetOffsetAngle_Horizontal);
        // frc::SmartDashboard::PutNumber("ty", targetOffsetAngle_Vertical);
        // frc::SmartDashboard::PutNumber("Current Area", currentArea);
        // frc::SmartDashboard::PutNumber("targetSkew", targetSkew);

        //wait for pipeline change and target acquisition
        if (limelightBack->GetNumber("tv", 0.0) == 0 || limelightBack->GetNumber("getpipe", 0.0) == DRIVER_PIPELINE)
        {
            IsTargetNotAcquired(leftTank, rightTank);
            return false;
        }
        if (limelightBack->GetNumber("tv", 0.0) > 0 && limelightBack->GetNumber("getpipe", 0.0) != DRIVER_PIPELINE && !isTargetAcquired)
        {
            isTargetAcquired = true;
            overrideEnabled = false;
            counter = 0;
            limelightBack->PutNumber("pipeline", pipelineNumber);
            accumAngleError = 0;
            prevDistanceError = 100;
            prevAngleError = 100;
            prevDistance = 0;
            prevAngle = 0;
        }
    }

    double p_dist_loop = 0;
    double targetHeight = TARGET_LOW_HEIGHT_INCHES;
    double limelightAngle = LIMELIGHT_ANGLE_FRONT;
    double kP_DIST = kP_DIST_FPS;
    if (isFront)
    {
        kP_DIST = kP_DIST_FPS;
        limelightAngle = LIMELIGHT_ANGLE_FRONT;
    }
    else
    {
        kP_DIST = kP_DIST_FPS_BACK;
        limelightAngle = LIMELIGHT_ANGLE_BACK;
    }
    double currentDistanceInches = (LIMELIGHT_HEIGHT_INCHES - targetHeight) / tan((limelightAngle + crosshairAngle - targetOffsetAngle_Vertical) * (M_PI / 180)); //current distance from target
    frc::SmartDashboard::PutNumber("current distance", currentDistanceInches);
    frc::SmartDashboard::PutNumber("zDesiredInches", zDesiredInches);
    double desiredAngle = targetOffsetAngle_Vertical * slopeForAngleCalc + interceptForAngleCalc;
    double angleError = targetOffsetAngle_Horizontal - desiredAngle;

    //I Gain component
    if (abs(angleError) < I_ZONE_ANGLE)
    {
        accumAngleError += angleError;
    }
    else
    {
        accumAngleError = 0;
    }

    //D Gain component
    double slopeError = (angleError - prevAngleError) / (currentTime - prevT);

    frc::SmartDashboard::PutNumber("Desired Angle", desiredAngle);
    frc::SmartDashboard::PutNumber("Angle Error", angleError);

    frc::SmartDashboard::PutNumber("Intercept", interceptForAngleCalc);
    frc::SmartDashboard::PutNumber("Slope", slopeForAngleCalc);
    frc::SmartDashboard::PutNumber("Crosshair Angle", crosshairAngle);

    adjust = kP_ANGLE * angleError + kI_ANGLE * accumAngleError - kD_ANGLE * slopeError;
    frc::SmartDashboard::PutNumber("Adjust", adjust);

    double distanceError = (zDesiredInches - currentDistanceInches);
    p_dist_loop = kP_DIST * distanceError;
    if (p_dist_loop > LL_MAX_FEET_PER_SEC)
    {
        p_dist_loop = LL_MAX_FEET_PER_SEC;
    }
    else if (p_dist_loop < -LL_MAX_FEET_PER_SEC)
    {
        p_dist_loop = -LL_MAX_FEET_PER_SEC;
    }

    desiredLeftFPS = p_dist_loop + adjust;
    desiredRightFPS = p_dist_loop - adjust;

    frc::SmartDashboard::PutNumber("Desired FPS Left", desiredLeftFPS);
    frc::SmartDashboard::PutNumber("Desired FPS Right", desiredRightFPS);
    frc::SmartDashboard::PutNumber("Distance Error", distanceError);
    frc::SmartDashboard::PutNumber("Prev Distance", prevDistance);
    frc::SmartDashboard::PutNumber("Prev Angle", prevAngle);
    // std::cout << "PrevDistErr," << prevDistanceError;
    // std::cout << " PrevAngleErr," << prevAngleError;
    // std::cout << " Dist," << currentDistanceInches;
    // std::cout << " Angle," << targetOffsetAngle_Horizontal;
    // std::cout << " PrevDist," << prevDistance;
    // std::cout << " PrevAngle," << prevAngle;

    if (abs(prevDistanceError) < 10 && abs(prevAngleError) < 5)
    {                                                        //close to target{
        if (abs(currentDistanceInches - prevDistance) > 5 || //sudden jump in target
            abs(prevAngle - targetOffsetAngle_Horizontal) > 5)
        {
            overrideEnabled = true;
        }
    }
    std::cout << " wantToNotMove," << wantToNotMove << std::endl;

    if (abs(distanceError) < 2 && abs(angleError) < ALLOWED_ANGLE_ERROR_LL)
    {
        if (isBallMode)
        {
            StopMotors();
            return true;
        }
        else
        {
            wantToDriveHatchInPlace = true;
            hatchPlaceTimer->Reset();
            hatchPlaceTimer->Start();
        }
    }

    if (wantToNotMove || overrideEnabled)
    {
        frc::SmartDashboard::PutString("test string", "I am in want to not move");
        StopMotors();
    }
    else
    {
        leftBack->Set(ControlMode::Velocity, desiredLeftFPS * FEET_TO_NU * CONVERT_100MS_TO_SECONDS); //in feet/s
        rightBack->Set(ControlMode::Velocity, desiredRightFPS * FEET_TO_NU * CONVERT_100MS_TO_SECONDS);
        leftMid->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightMid->Set(ControlMode::Follower, RIGHT_BACK_ID);
        leftFront->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightFront->Set(ControlMode::Follower, RIGHT_BACK_ID);
    }

    prevDistance = currentDistanceInches;
    prevAngle = targetOffsetAngle_Horizontal;
    prevDistanceError = distanceError;
    prevAngleError = angleError;
    prevT = currentTime;
    return false;
}

void Drivetrain::TankDrive(double leftValue, double rightValue)
{
    // m_follower_notifier.Stop();
    //  leftValue = leftValue*0.5;
    //  rightValue = rightValue*0.5;
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
        gearShifter->Set(false);
    }
    else if (!isHighGear)
    {
        gearShifter->Set(true);
    }
}

void Drivetrain::AutoDriveForward(bool isBut, bool isVelocityControl)
{

    double testPercentOut = frc::SmartDashboard::GetNumber("Test PercentOut Speed", 0.5);

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

        desiredLeftFPS = desiredRightFPS = 2.5; //Was 5.0, changed by rObErT
        if (!isFront)
        {
            desiredLeftFPS = desiredRightFPS = -desiredLeftFPS;
        }
        std::cout << "Desired NU per 100MS: " << (desiredLeftFPS * FEET_TO_NU * CONVERT_100MS_TO_SECONDS) << std::endl;
        std::cout << "kFeedforwardGain " << (desiredLeftFPS * FEET_TO_NU * CONVERT_100MS_TO_SECONDS) << std::endl;

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

void Drivetrain::AutoDriveBackwards(bool isBut, bool isVelocityControl)
{

    // double testPercentOut = frc::SmartDashboard::GetNumber("Test PercentOut Speed", 0.5);

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

        desiredLeftFPS = desiredRightFPS = -2.5; //Was 5.0, changed by rObErT
        if (!isFront)
        {
            desiredLeftFPS = desiredRightFPS = -desiredLeftFPS;
        }
        std::cout << "Desired NU per 100MS: " << (desiredLeftFPS * FEET_TO_NU * CONVERT_100MS_TO_SECONDS) << std::endl;
        std::cout << "kFeedforwardGain " << (desiredLeftFPS * FEET_TO_NU * CONVERT_100MS_TO_SECONDS) << std::endl;

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

void Drivetrain::IsTargetNotAcquired(double leftTank, double rightTank)
{
    isInLLDrive = false;
    TankDrive(leftTank, rightTank);
    isInLLDrive = true;
    isTargetAcquired = false;
}

void Drivetrain::SetDesiredLLDistances(double xDesiredInches, double zDesiredInches)
{
    this->xDesiredInches = xDesiredInches;
    this->zDesiredInches = zDesiredInches;
    //std::cout << "zDesiredInches: " << zDesiredInches << std::endl;
}

void Drivetrain::SetSlopeInterceptForAngleCalc(double slope, double intercept)
{
    slopeForAngleCalc = slope;
    interceptForAngleCalc = intercept;
}

void Drivetrain::SetCrosshairAngle(double crosshairAngle)
{
    this->crosshairAngle = crosshairAngle;
}

void Drivetrain::SetIsFrontLL(bool isFront)
{
    this->isFront = isFront;
}

void Drivetrain::SetPipelineNumber(int pipelineNumber)
{
    this->pipelineNumber = pipelineNumber;
}

void Drivetrain::StopMotors()
{
    leftBack->Set(ControlMode::PercentOutput, 0.0);
    rightBack->Set(ControlMode::PercentOutput, 0.0);
    leftMid->Set(ControlMode::Follower, LEFT_BACK_ID);
    rightMid->Set(ControlMode::Follower, RIGHT_BACK_ID);
    leftFront->Set(ControlMode::Follower, LEFT_BACK_ID);
    rightFront->Set(ControlMode::Follower, RIGHT_BACK_ID);
}