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

    //Initialize Solenoids
    gearShifter = new frc::Solenoid(GEAR_SHIFTER_ID);

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
    leftBack->Config_kF(0, kFF_SPEED, TALON_TIMEOUT);
    rightBack->Config_kF(0, kFF_SPEED, TALON_TIMEOUT);
    leftBack->Config_kP(0, kP_SPEED, TALON_TIMEOUT);
    rightBack->Config_kP(0, kP_SPEED, TALON_TIMEOUT);
    leftBack->Config_kD(0, kD_SPEED_LEFT, TALON_TIMEOUT);
    rightBack->Config_kD(0, kD_SPEED_RIGHT, TALON_TIMEOUT);
    leftBack->Config_kI(0, kI_SPEED, TALON_TIMEOUT);
    rightBack->Config_kI(0, kI_SPEED, TALON_TIMEOUT);
    leftBack->Config_IntegralZone(0, kI_ZONE, TALON_TIMEOUT);
    rightBack->Config_IntegralZone(0, kI_ZONE, TALON_TIMEOUT);

    //initialize auto versus manual driving state
    isInAutoDrive = false;
    isInLLDrive = false;

    //Config for Talon Loop Ramp Rates in seconds
    leftFront->ConfigClosedloopRamp(TALON_RAMP_RATE);
    leftMid->ConfigClosedloopRamp(TALON_RAMP_RATE);
    leftBack->ConfigClosedloopRamp(TALON_RAMP_RATE);
    rightFront->ConfigClosedloopRamp(TALON_RAMP_RATE);
    rightMid->ConfigClosedloopRamp(TALON_RAMP_RATE);
    rightBack->ConfigClosedloopRamp(TALON_RAMP_RATE);
    leftFront->ConfigOpenloopRamp(TALON_RAMP_RATE);
    leftMid->ConfigOpenloopRamp(TALON_RAMP_RATE);
    leftBack->ConfigOpenloopRamp(TALON_RAMP_RATE);
    rightFront->ConfigOpenloopRamp(TALON_RAMP_RATE);
    rightMid->ConfigOpenloopRamp(TALON_RAMP_RATE);
    rightBack->ConfigOpenloopRamp(TALON_RAMP_RATE);

    //state of whether or not a target is acquired 
    isTargetAcquired = false;

    //initialize variables that change depending on position the arm is in for LL tracking
    //x and z change based on front and back, low, middle, high and ball/hatch
    //slope, intercept and crosshair angle is different for frint and back cameras
    xDesiredInches = 0;
    zDesiredInches = 36;
    slopeForAngleCalc = 0.0;
    interceptForAngleCalc = 0.0;
    crosshairAngle = 0.0;
    prevT = 0;

    //Gyro
    // ahrs = new AHRS(SerialPort::Port::kUSB); //was in USB port, bad idea, don't do this
    ahrs = new AHRS(SPI::Port::kMXP);
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

    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->checkDriveTrainGearDriver, driveTrainGearShuffle);


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
    //append full path to the file name input to the method
    wpi::SmallString<256> path;
    frc::filesystem::GetDeployDirectory(path);
    wpi::sys::path::append(path, "paths", name + ".pf1.csv");

    if (!wpi::sys::fs::exists(path))
    {
        throw std::runtime_error("Path " + std::string(path.c_str()) + " does not exist!");
    }
    return std::string(path.c_str());
}

int Drivetrain::get_trajectory(std::string name, Segment *traj_out)
{
    //put the contents of the file with the name 'name' into the traj_out variable
    FILE *fp = fopen(get_trajectory_file(name).c_str(), "r");
    int len = pathfinder_deserialize_csv(fp, traj_out);
    fclose(fp);
    return len;
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
    //calls methods to deserialize and set trajectories for the left and right side of the drivetrain
    left_trajectory_length = get_trajectory(pathName + ".right", leftTrajectory);  //This is supposed to be flipped! This is a bug in FRC's libraries
    right_trajectory_length = get_trajectory(pathName + ".left", rightTrajectory); //This is supposed to be flipped! This is a bug in FRC's libraries
    follow_path_counter = 0;
}

bool Drivetrain::FollowPath(bool isReverse)
{
    //continue following path while the path is not almost finished
    //cut off early to save time by not stopping before tracking with the Limelight
    if (follow_path_counter < (left_trajectory_length - PATH_CUTOFF_TIME/0.02)) //adjust if needed 
    {
        double leftVelocity = 0;
        double rightVelocity = 0;

        double gyro_heading = ahrs->GetYaw();
        double desired_heading = r2d(leftTrajectory[follow_path_counter].heading);

        //flip sides and sign of velocity and adjust angle if the path is meant to be driven backwards
        if (isReverse)
        {
            leftVelocity = -1 * rightTrajectory[follow_path_counter].velocity;
            rightVelocity = -1 * leftTrajectory[follow_path_counter].velocity;
            desired_heading -= 180;
        }
        else //otherwise, retreive velocity normally, no angle adjustment
        {
            leftVelocity = leftTrajectory[follow_path_counter].velocity;
            rightVelocity = rightTrajectory[follow_path_counter].velocity;
        }

        //Calculate angle error
        double angle_difference = desired_heading - gyro_heading;
        frc::SmartDashboard::PutNumber("Follow Path Desired Heading", desired_heading);
        angle_difference = std::fmod(angle_difference, 360.0);
        if (std::abs(angle_difference) > 180.0)
        {
            angle_difference = (angle_difference > 0) ? angle_difference - 360 : angle_difference + 360;
        }
        frc::SmartDashboard::PutNumber("Follow Path Angle Error", angle_difference);

        //calculate adjustment to velocity serpoint based on error in angle
        double turnAdjustment = 0.1 * angle_difference; //was -0.002
        frc::SmartDashboard::PutNumber("Follow Path Turn Adjustment", turnAdjustment);

        //Set velocities from the trajectory velocity +/- turn adjustment for the drivetrains's velocity PID to handle
        leftBack->Set(ControlMode::Velocity, ((leftVelocity + turnAdjustment) * FEET_TO_NU * CONVERT_100MS_TO_SECONDS)); //in feet/s
        rightBack->Set(ControlMode::Velocity, ((rightVelocity - turnAdjustment) * FEET_TO_NU * CONVERT_100MS_TO_SECONDS));
        leftMid->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightMid->Set(ControlMode::Follower, RIGHT_BACK_ID);
        leftFront->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightFront->Set(ControlMode::Follower, RIGHT_BACK_ID);

        follow_path_counter++;
        return false;
    }
    //stop motors when finished running trajectory
    else
    {
        StopMotors();
        return true;
    }
}

bool Drivetrain::AutoDrive(bool wantLimelight, double leftTank, double rightTank, bool isBallMode, bool wantToNotMove)
{
    //when not limelight tracking, reset LL drive state variables and return
    if (!wantLimelight) 
    {
        isInLLDrive = false;
        wantToDriveHatchInPlace = false;
        isTargetAcquired = false;
        return false;
    }

    double currentTime = angleDGainTimer->Get();
    //drive hatch into place, and skip LL code if you've gotten near the target and want to drive the hatch forward
    if (wantToDriveHatchInPlace)
    {
        //Drive forward to push hatch panel in place, then return true
        if (hatchPlaceTimer->Get() > LL_HATCH_PLACE_TIMER) 
        {
            return true;
        }
        else
        {
            AutoDriveForward(false, true);
            return false;
        }
    }

    //init pipelines
    if (!isInLLDrive) 
    {
        //set limelight pipeline, happens once because of isInLLDrive
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

    //get data from front LL, if the arm is in front
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
            limelightBack->PutNumber("pipeline", pipelineNumber);
            accumAngleError = 0;
            prevDistanceError = 100;
            prevAngleError = 100;
            prevDistance = 0;
            prevAngle = 0;
        }
    }

    //set variables that are dependent on front/back and (not implemented) low/high target
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

    //calcualte current distance from target
    double currentDistanceInches = (LIMELIGHT_HEIGHT_INCHES - targetHeight) / tan((limelightAngle + crosshairAngle - targetOffsetAngle_Vertical) * (M_PI / 180)); //current distance from target
    frc::SmartDashboard::PutNumber("current distance", currentDistanceInches);
    frc::SmartDashboard::PutNumber("zDesiredInches", zDesiredInches);
    
    //calculate desired angle of target in the image and the angle error form that desired angle
    double desiredAngle = targetOffsetAngle_Vertical * slopeForAngleCalc + interceptForAngleCalc;
    double angleError = targetOffsetAngle_Horizontal - desiredAngle;

    //I Gain component - accumulated error
    if (abs(angleError) < I_ZONE_ANGLE)
    {
        accumAngleError += angleError;
    }
    else
    {
        accumAngleError = 0;
    }

    //D Gain component - change in error over time
    double slopeError = (angleError - prevAngleError) / (currentTime - prevT);

    //Debugging prints to smart dashboard
    frc::SmartDashboard::PutNumber("Desired Angle", desiredAngle);
    frc::SmartDashboard::PutNumber("Angle Error", angleError);

    frc::SmartDashboard::PutNumber("Intercept", interceptForAngleCalc);
    frc::SmartDashboard::PutNumber("Slope", slopeForAngleCalc);
    frc::SmartDashboard::PutNumber("Crosshair Angle", crosshairAngle);

    //Total PID Angle adjustment factor calculation
    angleErrorAdjustmentValue_LLTracking = kP_ANGLE * angleError + kI_ANGLE * accumAngleError - kD_ANGLE * slopeError;
    frc::SmartDashboard::PutNumber("angleErrorAdjustmentValue_LLTracking", angleErrorAdjustmentValue_LLTracking);

    //Calculate error in distance from the target, and the P control input value
    double distanceError = (zDesiredInches - currentDistanceInches);
    p_dist_loop = kP_DIST * distanceError;

    //ensure robot will not be asked to go over a set maximum speed
    if (p_dist_loop > LL_MAX_FEET_PER_SEC)
    {
        p_dist_loop = LL_MAX_FEET_PER_SEC;
    }
    else if (p_dist_loop < -LL_MAX_FEET_PER_SEC)
    {
        p_dist_loop = -LL_MAX_FEET_PER_SEC;
    }

    //add distance P control to angle error PID adjustment to get 
    //velocities for input to the drivetrain velocity PID controller 
    desiredLeftFPS = p_dist_loop + angleErrorAdjustmentValue_LLTracking;
    desiredRightFPS = p_dist_loop - angleErrorAdjustmentValue_LLTracking;

    //Debugging SmartDaashboard prints
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

    //If close to the target, and you see a large jump, set overrideEnabled to stop tracking
    if (abs(prevDistanceError) < DISTANCE_WITHIN_NO_JUMPS_LL && abs(prevAngleError) < ANGLE_WITHIN_NO_JUMPS_LL)
    {                                                        //close to target{
        if (abs(currentDistanceInches - prevDistance) > MAX_JUMP_DISTANCE_ALLOWED || //sudden jump in target
            abs(prevAngle - targetOffsetAngle_Horizontal) > MAX_JUMP_ANGLE_ALLOWED)
        {
            overrideEnabled = true;
        }
    }

    //If close enough to target, stop tracking
    if (abs(distanceError) < ALLOWED_DISTANCE_ERROR_LL && abs(angleError) < ALLOWED_ANGLE_ERROR_LL)
    {
        //if in ball mode, just stop and return that you are done
        if (isBallMode)
        {
            StopMotors();
            return true;
        }
        //if in hatch mode, start the reset the timer and set the flag to drive the hatch into place
        else
        {
            wantToDriveHatchInPlace = true;
            hatchPlaceTimer->Reset();
            hatchPlaceTimer->Start();
        }
    }

    //If for whatever reason we're not moving, don't set velocities, but instead stop motor
    if (wantToNotMove || overrideEnabled)
    {
        frc::SmartDashboard::PutString("test string", "I am in want to not move");
        StopMotors();
    }
    // otherwise, send calculated velocities to drivetrain velocity PID controller
    else
    {
        leftBack->Set(ControlMode::Velocity, desiredLeftFPS * FEET_TO_NU * CONVERT_100MS_TO_SECONDS); //in feet/s
        rightBack->Set(ControlMode::Velocity, desiredRightFPS * FEET_TO_NU * CONVERT_100MS_TO_SECONDS);
        leftMid->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightMid->Set(ControlMode::Follower, RIGHT_BACK_ID);
        leftFront->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightFront->Set(ControlMode::Follower, RIGHT_BACK_ID);
    }

    //update the previous values of distance and angle, their errors, and time
    prevDistance = currentDistanceInches;
    prevAngle = targetOffsetAngle_Horizontal;
    prevDistanceError = distanceError;
    prevAngleError = angleError;
    prevT = currentTime;
    return false;
}

void Drivetrain::TankDrive(double leftValue, double rightValue)
{
    //  Comment these in for demo-ing
    //  leftValue = leftValue*0.5;
    //  rightValue = rightValue*0.5;

    //if not driving automatically, set the values to the drivetrain here
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

void Drivetrain::SetGearShifter(bool isHighGear)
{
    //for shuffleboard output
    driveTrainGearShuffle = isHighGear;

    //if wanting high gear, set to high gear
    if (isHighGear)
    {
        gearShifter->Set(false);
    }
    //otherwise, set to low gear
    else if (!isHighGear)
    {
        gearShifter->Set(true);
    }
}

void Drivetrain::AutoDriveForward(bool isTestPercentOut, bool isVelocityControl)
{
    //if wanting to automatically give a percent output, use the one defined
    if (isTestPercentOut && !isVelocityControl)
    {
        isInAutoDrive = true;

        leftBack->Set(ControlMode::PercentOutput, TEST_PERCENT_OUTPUT);
        rightBack->Set(ControlMode::PercentOutput, TEST_PERCENT_OUTPUT);
        leftMid->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightMid->Set(ControlMode::Follower, RIGHT_BACK_ID);
        leftFront->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightFront->Set(ControlMode::Follower, RIGHT_BACK_ID);
    }
    //if wanting to drive with velocity control in the direction of the arm
    else if (isVelocityControl)
    {
        isInAutoDrive = true;

        desiredLeftFPS = desiredRightFPS = AUTO_VELOCITY_CONTROL_DRIVE_SPEED;
        if (!isFront)
        {
            desiredLeftFPS = desiredRightFPS = -desiredLeftFPS;
        }

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

void Drivetrain::AutoDriveBackwards(bool isTestPercentOut, bool isVelocityControl)
{
    //if wanting to automatically give a percent output, use the one defined
    if (isTestPercentOut && !isVelocityControl)
    {
        isInAutoDrive = true;

        leftBack->Set(ControlMode::PercentOutput, TEST_PERCENT_OUTPUT);
        rightBack->Set(ControlMode::PercentOutput, TEST_PERCENT_OUTPUT);
        leftMid->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightMid->Set(ControlMode::Follower, RIGHT_BACK_ID);
        leftFront->Set(ControlMode::Follower, LEFT_BACK_ID);
        rightFront->Set(ControlMode::Follower, RIGHT_BACK_ID);
    }
    //if wanting to drive with velocity control in the opposite direction of the arm
    else if (isVelocityControl)
    {
        isInAutoDrive = true;

        desiredLeftFPS = desiredRightFPS = -AUTO_VELOCITY_CONTROL_DRIVE_SPEED;
        if (!isFront)
        {
            desiredLeftFPS = desiredRightFPS = -desiredLeftFPS;
        }
        
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
    //used to reenable tank drive within LL tracking while no target is in view
    isInLLDrive = false;
    TankDrive(leftTank, rightTank);
    isInLLDrive = true;
    isTargetAcquired = false;
}

void Drivetrain::SetDesiredLLDistances(double xDesiredInches, double zDesiredInches)
{
    this->xDesiredInches = xDesiredInches;
    this->zDesiredInches = zDesiredInches;
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