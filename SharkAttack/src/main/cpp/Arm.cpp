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
    wrist = new rev::CANSparkMax(LEFT_WRIST_ID, BRUSHLESS);
    intake = new VictorSPX(INTAKE_ID);

    hatchGripper = new frc::DoubleSolenoid(SOLENOID_FORWARD, SOLENOID_REVERSE);

    armEncoder = new rev::CANEncoder(*rightArm);
    armPID = new rev::CANPIDController(*rightArm);

    wristEncoder = new rev::CANEncoder(*wrist);
    wristPID = new rev::CANPIDController(*wrist);

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

    //Set the Conversion Factor for Encoder output to read Degrees
    armEncoder->SetPositionConversionFactor(ARM_DEGREES_PER_MOTOR_ROTATION);
    wristEncoder->SetPositionConversionFactor(WRIST_DEGREES_PER_MOTOR_ROTATION);
    armEncoder->SetVelocityConversionFactor(ARM_RPM_TO_DEGREES_PER_SECOND);
    wristEncoder->SetVelocityConversionFactor(WRIST_RPM_TO_DEGREES_PER_SECOND);

    //Set arm Sparks invertions
    leftArm->SetInverted(false); //this one doesnt matter bc it is a follower
    rightArm->SetInverted(false);
    wrist->SetInverted(true);

    intake->SetInverted(false);

    leftArm->Follow(*rightArm, true); //make sure this is always inverted, otherwise arm motors will fight

    //Set to brake or coast
    leftArm->SetIdleMode(BRAKE);
    rightArm->SetIdleMode(COAST);
    wrist->SetIdleMode(BRAKE);
    intake->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    //Set Current Limits to not kill the Neos
    leftArm->SetSmartCurrentLimit(ARM_CURRENT_LIMIT);
    rightArm->SetSmartCurrentLimit(ARM_CURRENT_LIMIT);
    wrist->SetSmartCurrentLimit(WRIST_CURRENT_LIMIT);

    //Initialize arm/wrist starting state
    isArmInManual = true;
    isWristInManual = true;
    previousTargetPosition = 0.0;
    previousTargetWristPosition = 0.0;

    //default encoder position when robot starts
    armEncoder->SetPosition(0.0);
    wristEncoder->SetPosition(0.0);

    //Initialize values for arm movement state machine
    isArmInBack = false;
    isArmSwitchingSides = false;
    isArmMoving = false;
    isWristMoving = false;
    isArmInDefenseMode = false;
}

//Public Methods
void Arm::ManualRotateArm(double manualArmPower)
{
    frc::SmartDashboard::PutNumber("Arm Control Input", manualArmPower);
    if (manualArmPower != 0.0)
    {
        isArmInManual = true;
        rightArm->Set(manualArmPower);
    }
    if (manualArmPower == 0.0 && isArmInManual)
    {
        rightArm->Set(manualArmPower);
    }
    double leftVoltage = (manualArmPower * leftArm->GetBusVoltage());
    double rightVoltage = (manualArmPower * rightArm->GetBusVoltage());
    frc::SmartDashboard::PutNumber("Arm Voltage Left", leftVoltage);
    frc::SmartDashboard::PutNumber("Arm Voltage Right", rightVoltage);
}

void Arm::ManualRotateWrist(double manualWristPower)
{
    if (manualWristPower != 0.0)
    {
        isWristInManual = true;
        wrist->Set(manualWristPower);
    }
    if (manualWristPower == 0.0 && isWristInManual)
    {
        wrist->Set(manualWristPower);
    }
}

double Arm::GetCurrentArmPosition()
{
    return armEncoder->GetPosition();
}

void Arm::UpdateArmAndWristInManual(bool arm, bool wrist)
{
    isArmInManual = arm;
    isWristInManual = wrist;
}

void Arm::RunIntake(double intakeSpeed)
{
    intake->Set(motorcontrol::ControlMode::PercentOutput, intakeSpeed);
}

//Parameter: targetPosition -> Given final position in degrees for arm
void Arm::MoveArmToPosition(double targetPosition, bool isInBallMode, bool isInBallPickup, bool isInCargoShipMode)
{
    //defining variables for ShuffleManager
    armTargetPositionShuffle = targetPosition;
    isInHatchMode = !isInBallMode;

    //Getting arm/wrist positions
    currentArmPos = armEncoder->GetPosition();
    currentWristPos = wristEncoder->GetPosition();

    //defining the state of the arm system
    areWheelsVeryDown = (currentWristPos > 60 || currentWristPos < -60);
    willArmEnterDZ = ((currentArmPos < -ARM_DANGERZONE && targetPosition > -ARM_DANGERZONE) || (currentArmPos > ARM_DANGERZONE && targetPosition < ARM_DANGERZONE));
    areWheelsUp = (currentWristPos < 1.0 && currentWristPos > -1.0);
    isArmInDZ = (currentArmPos > -ARM_DANGERZONE && currentArmPos < ARM_DANGERZONE);
    isArmGoingToBack = (targetPosition < 0);

    //Calculate the voltage FeedForward to linearize the system for PID control        
    FFVoltage = MAX_FF_GAIN * (sin(currentArmPos * M_PI / 180));
    frc::SmartDashboard::PutNumber("FFVoltage", FFVoltage);
    
    //Move wrist to neutral position if in defense mode
    if(isArmInDefenseMode) {
        targetPosition = 0.0;
        MoveWristToPosition(WRIST_NEUTRAL);
        CloseHatchGripper();
    }
    //If the arm is in the danger zone, move wrist to neutral
    //and close hatch gripper. If wheels are "very down", switch
    //arm/wrist control to manual (something is wrong).
    else if (isArmInDZ)
    {
        if (!isHatchGripperClosed)
        {
            CloseHatchGripper();
        }

        if (areWheelsVeryDown)
        {
            isArmInManual = true;
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
    //If the arm will enter the Danger Zone, close the hatch gripper and
    //move the wrist to neutral. And if wrist is not neutral yet, 
    //set the arm target position to the arm_checkpoint on the current side
    else if (willArmEnterDZ)
    {
        if (!isHatchGripperClosed)
        {
            CloseHatchGripper();
        }
        MoveWristToPosition(WRIST_NEUTRAL);

        if (!areWheelsUp)
        {
            //returns the value of ARM_CHECKPOINT with the sign of currentArmPos
            targetPosition = copysign(ARM_CHECKPOINT, currentArmPos);
        }
    }
    //if arm is outside of frame perimeter, on the correct side
    //move arm and to final position, and hatch gripper to desired position
    else if (!isArmInDZ && !willArmEnterDZ)
    {
        if (isHatchGripperClosed && !wantHatchGripperClosed)
        {
            OpenHatchGripper();
        }
        else if (!isHatchGripperClosed && wantHatchGripperClosed)
        {
            CloseHatchGripper();
        }
        else
        {
            //in desired state already
        }
        MoveWristToPosition(FindWristFinalPosition(isArmGoingToBack, isInBallMode, isInBallPickup, isInCargoShipMode));
    }

    //If arm is not in manual, set the target for the pid to move towards.
    if (!isArmInManual)
    {
        armPID->SetReference(targetPosition, rev::ControlType::kPosition, 0, FFVoltage);
    }
    
    //if in ball pickup mode and gripper is closed/gripped, set gripper desired state to open/release    
    if (isInBallMode)
    {
        wasInBallMode = true;
        if (isHatchGripperClosed)
        {
            //desired state
            wantHatchGripperClosed = false;
        }
    }
    //if switching to hatch mode, set gripper desired state to closed/gripped
    else if (!isInBallMode && wasInBallMode){
        wantHatchGripperClosed = true;
        wasInBallMode = false;
    }
}

//returns desired preset wrist position in degrees, based on the current mode
//to be used in MoveWristToPosition
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

void Arm::MoveWristToPosition(double wristTargetPosition)
{
    wristTargetPositionShuffle = wristTargetPosition;
    double wristTargetRelativeToArm = armEncoder->GetPosition() - wristTargetPosition;

    frc::SmartDashboard::PutNumber("Wrist Target Position", wristTargetPosition);

    frc::SmartDashboard::PutNumber("Wrist Position Error", wristEncoder->GetPosition() - wristTargetPosition);

    //if the gripper is closed/gripped, do not allow wrist to bang hatch into the arm
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
    //If wrist is not in manual, set the target for the pid to move towards.
    if (!isWristInManual)
    {
        wristPID->SetReference(wristTargetPosition, rev::ControlType::kPosition);
    }
}

void Arm::OpenHatchGripper()
{
    hatchGripper->Set(frc::DoubleSolenoid::Value::kForward);
    isHatchGripperClosed = false;
}

void Arm::CloseHatchGripper()
{
    hatchGripper->Set(frc::DoubleSolenoid::Value::kReverse);
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
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->wristEncoderDriver, wristEncoder->GetPosition());

    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->checkHatchGripperDriver, isHatchGripperClosed);
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->checkWristModeDriver, isInHatchMode);
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