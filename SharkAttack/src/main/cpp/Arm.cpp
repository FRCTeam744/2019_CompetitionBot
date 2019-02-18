/*----------------------------------------------------------------------------------*/

//Methods for the Arm class (any and all things arm/wrist/placing)

/*----------------------------------------------------------------------------------*/

#include "Arm.h"

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
    leftArm = new rev::CANSparkMax(42, BRUSHLESS);
    rightArm = new rev::CANSparkMax(43, BRUSHLESS);
    leftWrist = new rev::CANSparkMax(44, BRUSHLESS);
    rightWrist = new rev::CANSparkMax(45, BRUSHLESS);
    intake = new TalonSRX(46);

    hatchGripper = new frc::DoubleSolenoid(2, 3);

    armEncoder = new rev::CANEncoder(*leftArm);
    wristEncoder = new rev::CANEncoder(*leftWrist);

    armLimitSwitch = new frc::DigitalInput(2);
    wristLimitSwitch = new frc::DigitalInput(3);

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

    leftArm->SetSmartCurrentLimit(10);
    rightArm->SetSmartCurrentLimit(10);
    leftWrist->SetSmartCurrentLimit(10);
    rightWrist->SetSmartCurrentLimit(10);
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
    leftArm->Set(input);
    rightArm->Set(input);
}

void Arm::ManualRotateWrist(double input)
{
    leftWrist->Set(input);
    rightWrist->Set(input);
}

void Arm::Intake(bool buttonIsPressed)
{
    if(buttonIsPressed) {
        intake->Set(motorcontrol::ControlMode::PercentOutput, INTAKE_SPEED);
    }
}

void Arm::MoveArmToPosition(double targetPosition, double wristCurrentPosition, double armCurrentPosition)
{
    double delta = (targetPosition - armCurrentPosition) / ARM_ADJUSTER;
    MoveWristToPosition(wristCurrentPosition, armCurrentPosition);
    leftArm->Set(delta);
    rightArm->Set(delta);
}

void Arm::MoveWristToPosition(double wristCurrentPosition, double armCurrentPosition)
{
    double targetPosition;
    double delta;
    if ((armCurrentPosition > DANGER_ZONE_LIMIT) && (armCurrentPosition < -DANGER_ZONE_LIMIT))
    {
        //normal operations
    }
    else
    {
        //Inside DANGER Z O N E
        targetPosition = 0;
    }
    delta = (targetPosition - wristCurrentPosition) / WRIST_ADJUSTER;
    leftWrist->Set(delta);
}

void Arm::CheckHatchGripper(bool isClosed) {
    
    if (isClosed) {
        hatchGripper->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else if (!isClosed) {
        hatchGripper->Set(frc::DoubleSolenoid::Value::kForward);
    }
}

void Arm::PrintArmCurrent(){
    frc::SmartDashboard::PutNumber("Left Arm Current", leftArm->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Right Arm Current", rightArm->GetOutputCurrent());
}

void Arm::ManualCalibrateArm(){
    if(!GetArmLimitSwitch()){
        wasArmLimitSwitchTripped = false;
    }
    else if (GetArmLimitSwitch() && !wasArmLimitSwitchTripped && armEncoder->GetVelocity() < 0 && armEncoder->GetVelocity() < CALIBRATION_SPEED){
    }
    else {
        wasArmLimitSwitchTripped = true;
    }
}


bool Arm::GetArmLimitSwitch(){
    return !armLimitSwitch->Get();
}

bool Arm::GetWristLimitSwitch(){
    return !wristLimitSwitch->Get();
}