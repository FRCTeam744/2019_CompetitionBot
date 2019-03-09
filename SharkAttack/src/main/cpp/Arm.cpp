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
    rightArm->SetClosedLoopRampRate(RAMP_RATE);
    leftArm->SetClosedLoopRampRate(RAMP_RATE);

    armPID->SetP(P_GAIN_ARM);
    armPID->SetD(D_GAIN_ARM);
    armPID->SetI(I_GAIN_ARM);
    armPID->SetIZone(I_ZONE_ARM);
    armPID->SetFF(ARM_FF_GAIN);

    armPID->SetOutputRange(MIN_POWER_ARM, MAX_POWER_ARM);

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

    isArmInBack = false;
    isArmSwitchingSides = false;
    isArmMoving = false;
    isWristMoving = false;
}

//Public Methods
void Arm::ManualRotateArm(double input)
{
    frc::SmartDashboard::PutNumber("Arm Control Input", input);
    //ShuffleManager
    if (input != 0.0)
    {
        isArmInManual = true;
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

void Arm::UpdateArmAndWristInManual(bool arm, bool wrist)
{
    isArmInManual = arm;
    isWristInManual = wrist;
}

void Arm::RunIntake(double input)
{

    // if (in != 0.0 && !hasBall)
    // {
    //     intake->Set(motorcontrol::ControlMode::PercentOutput, in);

    //     // if(pdp->GetCurrent(INTAKE_PDP_PORT) < INTAKE_MAX_CURRENT){
    //     //     hasBall = true;
    //     // }
    // }
    // else if (in != 0.0 && hasBall)
    // {
    //     intake->Set(motorcontrol::ControlMode::PercentOutput, HOLD_BALL_SPEED);
    // }
    // else if (out != 0.0)
    // {
    //     hasBall = false;
    //     intake->Set(motorcontrol::ControlMode::PercentOutput, -out);
    // }
    // else
    // {
    //     hasBall = false;
    //     intake->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
    // }
    intake->Set(motorcontrol::ControlMode::PercentOutput, input);
}

//Parameter: targetPosition -> Given final position in degrees for arm
//Work in Progress
// void Arm::MoveArmToPosition(double targetPosition, bool isInBallMode)
// {

//     currentArmPos = armEncoder->GetPosition();
//     currentWristPos = wristEncoder->GetPosition();

//     areWheelsVeryDown = (currentWristPos > 60 || currentWristPos < -60);
//     willArmEnterDZ = ((currentArmPos < -ARM_DANGERZONE && targetPosition > -ARM_DANGERZONE) || (currentArmPos > ARM_DANGERZONE && targetPosition < ARM_DANGERZONE));
//     areWheelsUp = (currentWristPos < 1.0 && currentWristPos > -1.0);
//     isArmInDZ = (currentArmPos > -ARM_DANGERZONE && currentArmPos < ARM_DANGERZONE);
//     isArmGoingToBack = (targetPosition < 0);

//     if (isArmInDZ && areWheelsVeryDown)
//     {
//         isArmInManual = true;
//         isWristInManual = true;
//     }
//     else if (!isArmInDZ && !willArmEnterDZ)
//     {
//         MoveWristToPosition(FindWristFinalPosition(isArmGoingToBack, isInBallMode));
//     }
//     else if (willArmEnterDZ)
//     {
//         MoveWristToPosition(WRIST_NEUTRAL);
//     }

//     if (isInBallMode == true)
//     {
//         isInHatchMode = false;
//     }
//     else if (isInBallMode = false)
//     {
//         isInHatchMode = true;
//     }
// }

double Arm::FindWristFinalPosition(bool isGoingToBack, bool isInBallMode)
{

    if (!isInBallMode && !isArmGoingToBack)
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

void Arm::MoveArmToPosition(double targetPosition, bool isInBallMode)
{
    if (targetPosition < 0)
    {
        isArmInBack = true;
    }
    else
    {
        isArmInBack = false;
    }

    if (targetPosition != previousTargetPosition)
    {
        //checks to see if switching sides
        if (previousTargetPosition <= 0.0 && targetPosition > 0.0 || previousTargetPosition >= 0.0 && targetPosition < 0.0)
        {
            isArmSwitchingSides = true;
        }

        previousTargetPosition = targetPosition;
        isArmInManual = false;
    }

    if ((armEncoder->GetPosition() * targetPosition) <= 0)
    {
        isArmSwitchingSides = true;
    }
    else
    {
        isArmSwitchingSides = false;
    }

    if (armEncoder->GetVelocity() < 2.0 && armEncoder->GetVelocity() > -2.0)
    {
        isArmMoving = true;
    }
    else
    {
        isArmMoving = false;
    }

    //Wrist Control
    if (isArmSwitchingSides || targetPosition == 0.0)
    {
        MoveWristToPosition(WRIST_NEUTRAL);
    }
    else if (!isInBallMode && !isArmInBack)
    {
        MoveWristToPosition(WRIST_HATCH_FRONT);
    }
    else if (!isInBallMode && isArmInBack)
    {
        MoveWristToPosition(WRIST_HATCH_BACK);
    }
    else if (isInBallMode && !isArmInBack)
    {
        MoveWristToPosition(WRIST_BALL_FRONT);
    }
    else if (isInBallMode && isArmInBack)
    {
        MoveWristToPosition(WRIST_BALL_BACK);
    }

    FFVoltage = MAX_FF_GAIN * (sin(armEncoder->GetPosition() * M_PI / 180));
    frc::SmartDashboard::PutNumber("FFVoltage", FFVoltage);

    if (!isArmInManual && !isWristMoving)
    {
        armPID->SetReference(targetPosition, rev::ControlType::kPosition, 0, FFVoltage);
    }

    double positionError = armEncoder->GetPosition() - targetPosition;
    frc::SmartDashboard::PutNumber("Arm Position Error", positionError);
    if (positionError < 3.0 && positionError > -3.0)
    {
        isArmSwitchingSides = false;
    }

    if (isInBallMode == true)
    {
        isInHatchMode = false;
    }
    else if (isInBallMode = false)
    {
        isInHatchMode = true;
    }
}

// void Arm::MoveWristToPosition(double wristCurrentPosition, double armCurrentPosition)
void Arm::MoveWristToPosition(double wristTargetPosition)
{
    frc::SmartDashboard::PutNumber("Wrist Target Position", wristTargetPosition);

    frc::SmartDashboard::PutNumber("Wrist Position Error", wristEncoder->GetPosition() - wristTargetPosition);

    if ((isHatchGripperClosed == true) && (isInHatchMode == true))
    {
        if (wristRelativeToArm > WRIST_HATCH_LIMIT)
        {
            wristTargetPosition = (WRIST_HATCH_LIMIT - currentArmPos);
        }
        else if (wristRelativeToArm < -WRIST_HATCH_LIMIT)
        {
            wristTargetPosition = (-WRIST_HATCH_LIMIT - currentArmPos);
        }
    }

    if (!isWristInManual)
    {
        wristPID->SetReference(wristTargetPosition, rev::ControlType::kPosition);
    }
}

void Arm::CheckHatchGripper(bool isClosed)
{
    if (isClosed)
    {
        hatchGripper->Set(frc::DoubleSolenoid::Value::kReverse);
        isHatchGripperClosed = true;
    }
    else if (!isClosed)
    {
        hatchGripper->Set(frc::DoubleSolenoid::Value::kForward);
        isHatchGripperClosed = false;
    }
}

void Arm::PrintArmInfo()
{
    // frc::SmartDashboard::PutNumber("Left Arm Current", leftArm->GetOutputCurrent());
    // frc::SmartDashboard::PutNumber("Right Arm Current", rightArm->GetOutputCurrent());

    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, ShuffleManager:;GetInstance()->leftArmCurrentArmWrist, leftArm->GetOutputCurrent();
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, ShuffleManager:;GetInstance()->rightArmCurrentArmWrist, rightArm->GetOutputCurrent(); 
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->armEncoderDriver , armEncoder->GetPosition());
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, ShuffleManager::GetInstance()->armEncoderPreComp, armEncoder->GetPosition());
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->wristEncoderDriver, wristEncoder->GetPosition());
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, ShuffleManager::GetInstance()->wristEncoderPreComp, wristEncoder->GetPosition();)

    //frc::SmartDashboard::PutNumber("Wrist Encoder", wristEncoder->GetPosition());
    frc::SmartDashboard::PutNumber("Arm Speed Degrees Per Sec", armEncoder->GetVelocity());
    frc::SmartDashboard::PutNumber("Arm Velocity Error", 15 - armEncoder->GetVelocity());
}

void Arm::PrintArmInfotoConsole()
{
    if (frc::DriverStation::GetInstance().IsFMSAttached() == true)
    {
        compPrintCount++;
        if (compPrintCount > 1000)
        {
            std::cout << "Arm Amps Left: " << leftArm->GetOutputCurrent();
            std::cout << "Arm Amps Right: " << rightArm->GetOutputCurrent();
            compPrintCount = 0;
        }
        else
        {
            printCount++;
            if (printCount > 30)
            {
                std::cout << "Arm Amps Testing Left: " << leftArm->GetOutputCurrent();
                std::cout << "Arm Amps Testing Right: " << rightArm->GetOutputCurrent();
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