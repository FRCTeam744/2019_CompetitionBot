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
    //Initialize arm motors
    leftArm = new rev::CANSparkMax(LEFT_ARM_ID, BRUSHLESS);
    rightArm = new rev::CANSparkMax(RIGHT_ARM_ID, BRUSHLESS);
    leftWrist = new rev::CANSparkMax(LEFT_WRIST_ID, BRUSHLESS);
    rightWrist = new rev::CANSparkMax(RIGHT_WRIST_ID, BRUSHLESS);
    intake = new VictorSPX(INTAKE_ID);

    hatchGripper = new frc::DoubleSolenoid(2, 3);

    armEncoder = new rev::CANEncoder(*leftArm);
    //wristEncoder = new rev::CANEncoder(*leftWrist);
    wristEncoder = new rev::CANEncoder(*rightWrist); //Robert made this change for testing encoder values

    armLimitSwitch = new frc::DigitalInput(2);
    wristLimitSwitch = new frc::DigitalInput(3);

    // pdp = new frc::PowerDistributionPanel(0);

    //Set the Conversion Factor for Encoder output to read Degrees
    armEncoder->SetPositionConversionFactor(DEGREES_PER_MOTOR_ROTATION);
    wristEncoder->SetPositionConversionFactor(DEGREES_PER_MOTOR_ROTATION);

    //Set arm Sparks invertions
    leftArm->SetInverted(false);
    rightArm->SetInverted(true);
    leftWrist->SetInverted(false);
    rightWrist->SetInverted(true);

    intake->SetInverted(false);

    //Set to brake or coast
    leftArm->SetIdleMode(BRAKE);
    rightArm->SetIdleMode(BRAKE);
    leftWrist->SetIdleMode(BRAKE);
    rightWrist->SetIdleMode(BRAKE);
    intake->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    leftArm->SetSmartCurrentLimit(ARM_CURRENT_LIMIT);
    rightArm->SetSmartCurrentLimit(ARM_CURRENT_LIMIT);
    leftWrist->SetSmartCurrentLimit(WRIST_CURRENT_LIMIT);
    rightWrist->SetSmartCurrentLimit(WRIST_CURRENT_LIMIT);

    //SetFollowers
    // rightArm->Follow(*leftArm, false);
    // rightWrist->Follow(*leftWrist, false);

    wasArmLimitSwitchTripped = true;
    wasWristLimitSwitchTripped = true;
}

//Public Methods
void Arm::ManualRotateArm(double input)
{
    frc::SmartDashboard::PutNumber("Arm Control Input", input);
    //ShuffleManager
    //shuffleboard?
    leftArm->Set(input);
    rightArm->Set(input);
}

void Arm::ManualRotateWrist(double input)
{
    leftWrist->Set(input);
    rightWrist->Set(input);
}

void Arm::RunIntake(double in, double out)
{

    if (in != 0.0 && !hasBall)
    {
        intake->Set(motorcontrol::ControlMode::PercentOutput, in);

        // if(pdp->GetCurrent(INTAKE_PDP_PORT) < INTAKE_MAX_CURRENT){
        //     hasBall = true;
        // }
    }
    else if (in != 0.0 && hasBall)
    {
        intake->Set(motorcontrol::ControlMode::PercentOutput, HOLD_BALL_SPEED);
    }
    else if (out != 0.0)
    {
        hasBall = false;
        intake->Set(motorcontrol::ControlMode::PercentOutput, -out);
    }
    else
    {
        hasBall = false;
        intake->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
    }
}

double Arm::GetArmEncoderValue()
{
    frc::SmartDashboard::PutNumber("armEncoder", (armEncoder->GetPosition()));
    return (armEncoder->GetPosition());
}

double Arm::GetWristEncoderValue()
{
    frc::SmartDashboard::PutNumber("wristEncoder", (wristEncoder->GetPosition() * DEGREES_PER_MOTOR_ROTATION));
    return (wristEncoder->GetPosition());
}

void Arm::MoveArmToPosition(double targetPosition, double wristCurrentPosition, double armCurrentPosition)
{
    std::cout << "target Pos:  " << (targetPosition) << std::endl;
    frc::SmartDashboard::PutNumber("Arm position error:", targetPosition-armCurrentPosition);
    double delta = (targetPosition - armCurrentPosition) / ARM_ADJUSTER;
    // MoveWristToPosition(wristCurrentPosition, armCurrentPosition);
    leftArm->Set(delta);
    rightArm->Set(delta);
}

void Arm::MoveWristToPosition(double wristCurrentPosition, double armCurrentPosition)
{
    double targetPosition;
    double delta;
    if (armCurrentPosition > DANGER_ZONE_LIMIT)
    {
        //normal operations
        targetPosition = FRONT_BALL_PICKUP_POSITION;
    }
    else if (armCurrentPosition < -DANGER_ZONE_LIMIT)
    {
        //other normal operations
    }
    else
    {
        //Inside DANGER Z O N E
        targetPosition = 0;
    }
    delta = (targetPosition - wristCurrentPosition) / WRIST_ADJUSTER;
    leftWrist->Set(delta);
}

void Arm::CheckHatchGripper(bool isClosed)
{
    if (isClosed)
    {
        hatchGripper->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else if (!isClosed)
    {
        hatchGripper->Set(frc::DoubleSolenoid::Value::kForward);
    }
}

void Arm::PrintArmCurrent()
{
    frc::SmartDashboard::PutNumber("Left Arm Current", leftArm->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Right Arm Current", rightArm->GetOutputCurrent());

    // \huffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, "Left Arm Current", leftArm->GetOutputCurrent());
}

void Arm::ManualCalibrateArm()
{
    if (!GetArmLimitSwitch())
    {
        wasArmLimitSwitchTripped = false;
    }
    else if (GetArmLimitSwitch() && !wasArmLimitSwitchTripped && armEncoder->GetVelocity() < 0 && armEncoder->GetVelocity() > CALIBRATION_SPEED)
    {
        armEncoder->SetPosition(LIMIT_SWITCH_OFFSET);
    }
    else
    {
        wasArmLimitSwitchTripped = true;
    }
}

bool Arm::GetArmLimitSwitch()
{
    return !armLimitSwitch->Get();
}

bool Arm::GetWristLimitSwitch()
{
    return !wristLimitSwitch->Get();
}