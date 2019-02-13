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
    arm1 = new rev::CANSparkMax(30, BRUSHLESS);
    arm2 = new rev::CANSparkMax(31, BRUSHLESS);
    wrist = new rev::CANSparkMax(32, BRUSHLESS);
    intake = new rev::CANSparkMax(33, BRUSHLESS);

    hatchGripper = new frc::DoubleSolenoid(2,3);

    //Initialize encoders
    // armEncoder = new Encoder(0, 1, false, Encoder::EncodingType::k2X);

    //Set Encoder Parameters
    // armEncoder->SetMaxPeriod(.1);
    // armEncoder->SetMinRate(10);
    // armEncoder->SetDistancePerPulse(5);
    // armEncoder->SetReverseDirection(true);
    // armEncoder->SetSamplesToAverage(7);

    //Set arm Sparks invertions
    arm1->SetInverted(false);
    arm2->SetInverted(true);
    wrist->SetInverted(false);
    intake->SetInverted(false);

    //Set to brake or coast
    arm1->SetIdleMode(BRAKE);
    arm2->SetIdleMode(BRAKE);
    wrist->SetIdleMode(BRAKE);
    intake->SetIdleMode(BRAKE);
}

//Public Methods
void Arm::ManualRotateArm(double input)
{

    arm1->Set(input);
    arm2->Set(input);
}

void Arm::ManualRotateWrist(double input)
{
    wrist->Set(input);
}

void Arm::Intake(bool buttonIsPressed)
{

    intake->Set(INTAKE_SPEED);
}

void AutoRotateArm(double position) {
}

void AutoRotateWrist(double position) {
}

void Arm::CheckHatchGripper(bool isClosed){
    if(isClosed){
        hatchGripper->Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else if (!isClosed){
        hatchGripper->Set(frc::DoubleSolenoid::Value::kForward);
    }
}
