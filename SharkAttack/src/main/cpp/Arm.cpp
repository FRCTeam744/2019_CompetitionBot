/*----------------------------------------------------------------------------------*/

//Methods for the Arm class (any and all things arm/wrist/placing)

/*----------------------------------------------------------------------------------*/

#include "Arm.h"
#include "ShuffleManager.h"

Arm *Arm::s_instance = 0;

//Static Singleton Method
Arm *Arm::GetInstance()
{
    if (s_instance == 0)
    {
        s_instance = new Arm();
    }
    return s_instance;
}

//Constructor
Arm::Arm()
{
    //Initialize Arm Objects
    leftArm = new rev::CANSparkMax(LEFT_ARM_ID, BRUSHLESS);
    rightArm = new rev::CANSparkMax(RIGHT_ARM_ID, BRUSHLESS);
    leftWrist = new rev::CANSparkMax(LEFT_WRIST_ID, BRUSHLESS);
    intake = new VictorSPX(INTAKE_ID);

    hatchGripper = new frc::DoubleSolenoid(2, 3);

    armLimitSwitch = new frc::DigitalInput(2);
    wristLimitSwitch = new frc::DigitalInput(3);

    armEncoder = new rev::CANEncoder(*rightArm);
    armPID = new rev::CANPIDController(*rightArm);

    wristEncoder = new rev::CANEncoder(*leftWrist);
    wristPID = new rev::CANPIDController(*leftWrist);

    //ARM PositionPID SETUP
    rightArm->SetClosedLoopRampRate(RAMP_RATE_PIT);
    leftArm->SetClosedLoopRampRate(RAMP_RATE_PIT);

    armPID->SetP(P_GAIN_ARM);
    armPID->SetD(D_GAIN_ARM);
    armPID->SetI(I_GAIN_ARM);
    armPID->SetIZone(I_ZONE_ARM);
    armPID->SetFF(ARM_FF_GAIN);

    armPID->SetOutputRange(MIN_POWER_ARM_PIT, MAX_POWER_ARM_PIT);

    //WRIST PositionPID SETUP
    wristPID->SetP(P_GAIN_WRIST);
    wristPID->SetD(D_GAIN_WRIST);

    // pdp = new frc::PowerDistributionPanel(0);

    //Set the Conversion Factor for Encoder output to read Degrees
    armEncoder->SetPositionConversionFactor(ARM_DEGREES_PER_MOTOR_ROTATION);
    wristEncoder->SetPositionConversionFactor(WRIST_DEGREES_PER_MOTOR_ROTATION);
    armEncoder->SetVelocityConversionFactor(ARM_RPM_TO_DEGREES_PER_SECOND);
    wristEncoder->SetVelocityConversionFactor(WRIST_RPM_TO_DEGREES_PER_SECOND);

    //Set arm Sparks invertions
    leftArm->SetInverted(false); //this one doesnt matter bc it is a follower
    rightArm->SetInverted(false);
    leftWrist->SetInverted(true);

    intake->SetInverted(false);

    leftArm->Follow(*rightArm, true); //make sure this is always inverted, otherwise arm motors will fight

    //Set to brake or coast
    leftArm->SetIdleMode(BRAKE);
    rightArm->SetIdleMode(COAST);
    leftWrist->SetIdleMode(BRAKE);
    intake->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    //Set Current Limits to not kill the Neos
    leftArm->SetSmartCurrentLimit(ARM_CURRENT_LIMIT);
    rightArm->SetSmartCurrentLimit(ARM_CURRENT_LIMIT);
    leftWrist->SetSmartCurrentLimit(WRIST_CURRENT_LIMIT);

    //SetFollowers
    // rightArm->Follow(*leftArm, false);
    // rightWrist->Follow(*leftWrist, false);

    wasArmLimitSwitchTripped = true;
    wasWristLimitSwitchTripped = true;

    isArmInManual = true;
    isWristInManual = true;
    previousTargetPosition = 0.0;
    previousTargetWristPosition = 0.0;

    armEncoder->SetPosition(0.0);
    wristEncoder->SetPosition(0.0);

    isArmInBack = false;
    isArmSwitchingSides = false;
    isArmMoving = false;
    isWristMoving = false;
    isArmInDefenseMode = false;
}

//Public Methods
void Arm::ManualRotateArm(double input)
{
    frc::SmartDashboard::PutNumber("Arm Control Input", input);
    //ShuffleManager
    if (input != 0.0)
    {
        isArmInManual = true;
        // std::cout << "set arm to manual 1" <<std::endl;
        rightArm->Set(input);
    }
    if (input == 0.0 && isArmInManual)
    {
        rightArm->Set(input);
    }
    double leftVoltage = (input * leftArm->GetBusVoltage());
    double rightVoltage = (input * rightArm->GetBusVoltage());
    frc::SmartDashboard::PutNumber("Arm Voltage Left", leftVoltage);
    frc::SmartDashboard::PutNumber("Arm Voltage Right", rightVoltage);
}

void Arm::ManualRotateWrist(double input)
{
    if (input != 0.0)
    {
        isWristInManual = true;
        leftWrist->Set(input);
    }
    if (input == 0.0 && isWristInManual)
    {
        leftWrist->Set(input);
    }
}

double Arm::GetCurrentArmPosition() {
    return armEncoder->GetPosition();
}

void Arm::UpdateArmAndWristInManual(bool arm, bool wrist)
{
    isArmInManual = arm;
    // std::cout << "set arm to manual 2" <<std::endl;

    isWristInManual = wrist;
}

void Arm::RunIntake(double input)
{
  //  std::cout << "Intake input: " << input << std::endl;
    intake->Set(motorcontrol::ControlMode::PercentOutput, input);
}

//Parameter: targetPosition -> Given final position in degrees for arm
//Work in Progress
void Arm::MoveArmToPosition(double targetPosition, bool isInBallMode, bool isInBallPickup, bool isInCargoShipMode)
{
    armTargetPositionShuffle = targetPosition;
    isInHatchMode = !isInBallMode;
   // std::cout << "Target pos" << targetPosition << std::endl;
    currentArmPos = armEncoder->GetPosition();
    currentWristPos = wristEncoder->GetPosition();

    areWheelsVeryDown = (currentWristPos > 60 || currentWristPos < -60);
    willArmEnterDZ = ((currentArmPos < -ARM_DANGERZONE && targetPosition > -ARM_DANGERZONE) || (currentArmPos > ARM_DANGERZONE && targetPosition < ARM_DANGERZONE));
    areWheelsUp = (currentWristPos < 1.0 && currentWristPos > -1.0);
    isArmInDZ = (currentArmPos > -ARM_DANGERZONE && currentArmPos < ARM_DANGERZONE);
    // std::cout << "isArmInDZ: " << isArmInDZ << std::endl;
    isArmGoingToBack = (targetPosition < 0);

    FFVoltage = MAX_FF_GAIN * (sin(currentArmPos * M_PI / 180));
    frc::SmartDashboard::PutNumber("FFVoltage", FFVoltage);

     if(isArmInDefenseMode) {
        targetPosition = 0.0;
        MoveWristToPosition(WRIST_NEUTRAL);
        CloseHatchGripper();
    }

    else if (isArmInDZ)
    {
        if (!isHatchGripperClosed)
        {
            CloseHatchGripper();
        }

        if (areWheelsVeryDown)
        {
            isArmInManual = true;
            // std::cout << "set arm to manual 3" <<std::endl;

            isWristInManual = true;
        }
        else if (areWheelsUp)
        {
            MoveWristToPosition(WRIST_NEUTRAL);
        }
        else
        {
            targetPosition = currentArmPos;
            MoveWristToPosition(WRIST_NEUTRAL);
        }
    }
    else if (willArmEnterDZ)
    {
        if (!isHatchGripperClosed)
        {
            CloseHatchGripper();
        }
        MoveWristToPosition(WRIST_NEUTRAL);

        if (!areWheelsUp)
        {
            targetPosition = copysign(ARM_CHECKPOINT, currentArmPos);
        }
    }
    else if (!isArmInDZ && !willArmEnterDZ)
    {
        if (isHatchGripperClosed == true && wantHatchGripperClosed == false)
        {
            OpenHatchGripper();
        }
        else if (isHatchGripperClosed == false && wantHatchGripperClosed == true)
        {
            CloseHatchGripper();
        }
        else
        {
            //in desired state already
        }
        MoveWristToPosition(FindWristFinalPosition(isArmGoingToBack, isInBallMode, isInBallPickup, isInCargoShipMode));
    }

    // std::cout << "isArmInManual" << isArmInManual << std::endl;
    if (!isArmInManual)
    {
        // std::cout << "Set arm reference: " << targetPosition << std::endl;
        armPID->SetReference(targetPosition, rev::ControlType::kPosition, 0, FFVoltage);
    }

    if (isInBallMode == true)
    {
        wasInBallMode = true;
        if (isHatchGripperClosed)
        {
            // OpenHatchGripper();
            wantHatchGripperClosed = false;
        }
    }
    else if (!isInBallMode && wasInBallMode){
        wantHatchGripperClosed = true;
        wasInBallMode = false;
    }
}

double Arm::FindWristFinalPosition(bool isGoingToBack, bool isInBallMode, bool isInBallPickup, bool isInCargoShipMode)
{
    if (isInBallMode && !isArmGoingToBack && isInCargoShipMode)
    {
        return WRIST_CARGO_SHIP_FRONT;
    }
    else if (isInBallMode && isArmGoingToBack && isInCargoShipMode)
    {
        return WRIST_CARGO_SHIP_BACK;
    }
    else if (isInBallPickup && !isArmGoingToBack)
    {
        return WRIST_BALL_PICKUP_FRONT;
    }
    else if (isInBallPickup && isArmGoingToBack)
    {
        return WRIST_BALL_PICKUP_BACK;
    }
    else if (!isInBallMode && !isArmGoingToBack)
    {
        return WRIST_HATCH_FRONT;
    }
    else if (!isInBallMode && isArmGoingToBack)
    {
        return WRIST_HATCH_BACK;
    }
    else if (isInBallMode && !isArmGoingToBack)
    {
        return WRIST_BALL_FRONT;
    }
    else if (isInBallMode && isArmGoingToBack)
    {
        return WRIST_BALL_BACK;
    }
    else
    {
        return WRIST_NEUTRAL; //this should never be reached bc the ifs above cover all possibilities
    }
}

// void Arm::MoveWristToPosition(double wristCurrentPosition, double armCurrentPosition)
void Arm::MoveWristToPosition(double wristTargetPosition)
{
    wristTargetPositionShuffle = wristTargetPosition;
    double wristTargetRelativeToArm = armEncoder->GetPosition() - wristTargetPosition;

    frc::SmartDashboard::PutNumber("Wrist Target Position", wristTargetPosition);

    frc::SmartDashboard::PutNumber("Wrist Position Error", wristEncoder->GetPosition() - wristTargetPosition);

    if (wantHatchGripperClosed)
    {
        if (wristTargetRelativeToArm > WRIST_HATCH_LIMIT)
        {
            wristTargetPosition = (currentArmPos - WRIST_HATCH_LIMIT);
        }
        else if (wristTargetRelativeToArm < -WRIST_HATCH_LIMIT)
        {
            wristTargetPosition = (currentArmPos - (-WRIST_HATCH_LIMIT));
        }
    }

    if (!isWristInManual)
    {
        wristPID->SetReference(wristTargetPosition, rev::ControlType::kPosition);
    }
}

void Arm::OpenHatchGripper()
{
    hatchGripper->Set(frc::DoubleSolenoid::Value::kForward);
    // std::cout << "Open Hatch Gripper" << std::endl;
    isHatchGripperClosed = false;
}

void Arm::CloseHatchGripper()
{
    hatchGripper->Set(frc::DoubleSolenoid::Value::kReverse);
    // std::cout << "Close Hatch Gripper" << std::endl;
    isHatchGripperClosed = true;
}

void Arm::CheckHatchGripper(bool isClosed)
{
    if (isClosed)
    {
        wantHatchGripperClosed = true;
    }
    else if (!isClosed)
    {
        wantHatchGripperClosed = false;
    }
    // std::cout << "wantHatchGripperClosed: " << wantHatchGripperClosed << std::endl;
}

void Arm::PrintArmShuffleInfo()
{
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, ShuffleManager::GetInstance()->leftArmCurrentArmWrist, leftArm->GetOutputCurrent());
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, ShuffleManager::GetInstance()->rightArmCurrentArmWrist, rightArm->GetOutputCurrent());
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->armEncoderDriver, armEncoder->GetPosition());
    // //ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, ShuffleManager::GetInstance()->armEncoderPreComp, armEncoder->GetPosition());
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->wristEncoderDriver, wristEncoder->GetPosition());
    // //ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, ShuffleManager::GetInstance()->wristEncoderPreComp, wristEncoder->GetPosition());

    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->checkHatchGripperDriver, isHatchGripperClosed);
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->checkWristModeDriver, isInHatchMode);
    // //ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, ShuffleManager::GetInstance()->checkWristModePreComp, isInHatchMode);
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, ShuffleManager::GetInstance()->checkWristModeArmWrist, isInHatchMode);

    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->targetWristPositionDegreesDriver, wristTargetPositionShuffle);
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->targetArmPositionDegreesDriver, armTargetPositionShuffle);
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->targetWristPositionDegreesArmWrist, wristTargetPositionShuffle);
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->targetArmPositionDegreesArmWrist, armTargetPositionShuffle);

    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, ShuffleManager::GetInstance()->armVelocityArmWrist, armEncoder->GetVelocity());
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, ShuffleManager::GetInstance()->armVelocityErrorArmWrist, 15 - armEncoder->GetVelocity());

    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->checkArmManualDriver, isArmInManual);
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->checkWristManualDriver, isWristInManual);
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, ShuffleManager::GetInstance()->checkArmManualArmWrist, isArmInManual);
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, ShuffleManager::GetInstance()->checkWristManualArmWrist, isWristInManual);
   
    //frc::SmartDashboard::PutNumber("Arm Speed Degrees Per Sec", armEncoder->GetVelocity());
    //frc::SmartDashboard::PutNumber("Arm Velocity Error", 15 - armEncoder->GetVelocity());
    // frc::SmartDashboard::PutNumber("Left Arm Current", leftArm->GetOutputCurrent());
    // frc::SmartDashboard::PutNumber("Right Arm Current", rightArm->GetOutputCurrent());

    // frc::SmartDashboard::PutBoolean("IsArmInManual", isArmInManual);
    // frc::SmartDashboard::PutBoolean("IsWristInManual", isWristInManual); //these

    frc::SmartDashboard::PutBoolean("Is Gripper Closed", isHatchGripperClosed);

    frc::SmartDashboard::PutNumber("Arm Encoder", armEncoder->GetPosition());
    frc::SmartDashboard::PutNumber("Wrist Encoder", wristEncoder->GetPosition());
    // frc::SmartDashboard::PutNumber("Arm Speed Degrees Per Sec", armEncoder->GetVelocity());
    // frc::SmartDashboard::PutNumber("Arm Velocity Error", 15 - armEncoder->GetVelocity());
    // \huffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, "Left Arm Current", leftArm->GetOutputCurrent());
}

void Arm::PrintArmInfotoConsole()
{
    if (frc::DriverStation::GetInstance().IsFMSAttached() == true)
    {
        compPrintCount++;
        if (compPrintCount > 1000)
        {
            // std::cout << "Arm Amps Left: " << leftArm->GetOutputCurrent();
            // std::cout << "Arm Amps Right: " << rightArm->GetOutputCurrent();
            compPrintCount = 0;
        }
        else
        {
            printCount++;
            if (printCount > 30)
            {
                // std::cout << "Arm Amps Testing Left: " << leftArm->GetOutputCurrent();
                // std::cout << "Arm Amps Testing Right: " << rightArm->GetOutputCurrent();
                printCount = 0;
            }
        }
    }
}

void Arm::ManualCalibrateArm()
{
    //     if (!GetArmLimitSwitch())
    //     {
    //         wasArmLimitSwitchTripped = false;
    //     }
    //     else if (GetArmLimitSwitch() && !wasArmLimitSwitchTripped && armEncoder->GetVelocity() < 0 && armEncoder->GetVelocity() > CALIBRATION_SPEED)
    //     {
    //         armEncoder->SetPosition(LIMIT_SWITCH_OFFSET);
    //     }
    //     else
    //     {
    //         wasArmLimitSwitchTripped = true;
    //     }
}

bool Arm::GetArmLimitSwitch()
{
    return !armLimitSwitch->Get();
}

bool Arm::GetWristLimitSwitch()
{
    return !wristLimitSwitch->Get();
}

// double Arm::GetMAX_FF_GAIN()
// {
//     return MAX_FF_GAIN;
// }

// void Arm::SetMAX_FF_GAIN(double ArmFFVoltage)
// {
//     MAX_FF_GAIN = ArmFFVoltage;
// }

void Arm::SetToMatchMode()
{
    rightArm->SetClosedLoopRampRate(RAMP_RATE_FIELD);
    leftArm->SetClosedLoopRampRate(RAMP_RATE_FIELD);
    armPID->SetOutputRange(MIN_POWER_ARM_FIELD, MAX_POWER_ARM_FIELD);
    // std::cout << "Set to Match Mode is working " << std::endl;
}

void Arm::ToggleDefenseMode(bool wantsDefenseMode) {
    if(wantsDefenseMode && !isArmInDefenseMode) {
        leftArm->SetIdleMode(BRAKE);
        rightArm->SetIdleMode(BRAKE);
        armPID->SetP(P_GAIN_ARM_DEFENSE);
        armPID->SetD(D_GAIN_ARM_DEFENSE);
        armPID->SetI(I_GAIN_ARM_DEFENSE);
        armPID->SetIZone(I_ZONE_ARM_DEFENSE);
        // armPID->SetFF(ARM_FF_GAIN_DEFNSE);
         

        isArmInDefenseMode = true;
    }
    else if(!wantsDefenseMode && isArmInDefenseMode) {
        leftArm->SetIdleMode(BRAKE);
        rightArm->SetIdleMode(COAST);
        armPID->SetP(P_GAIN_ARM);
        armPID->SetD(D_GAIN_ARM);
        armPID->SetI(I_GAIN_ARM);
        armPID->SetIZone(I_ZONE_ARM);
        // armPID->SetFF(ARM_FF_GAIN_DEFNSE);        
        isArmInDefenseMode = false;
    }
}

bool Arm::GetIsGripperGripped(){
    return isHatchGripperClosed;
}