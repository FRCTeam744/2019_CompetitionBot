/*----------------------------------------------------------------------------------*/

//Header file for Arm.cpp

/*----------------------------------------------------------------------------------*/

#pragma once
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <frc/DoubleSolenoid.h>
//#include <ctre/Encoder.h>

class Arm
{
  public:
    static Arm *GetInstance();

    void ManualRotateArm(double input);
    void ManualRotateWrist(double input);
    void Intake(bool buttonIsPressed);
    //one button per preset : total of 8 buttons
    //2 buttons to switch between ball and panel
    //struct for different wrist and panel 
    void AutoRotateArm(double targetPosition, double armCurrentPosition);   //Degrees
    void AutoRotateWrist(double targetPosition, double wristCurrentPosition, double armCurrentPosition); //Degrees
    void CheckHatchGripper(bool isClosed);

    void MoveArmToPosition(const double encoderType, double armEncoderVal);

  private:
    static Arm *s_instance;
    Arm();

    //Private Objects
    rev::CANSparkMax *arm1, *arm2, *wrist, *intake;
    // Encoder *armEncoder;

    frc::DoubleSolenoid *hatchGripper;

    //Tunables
    const double INTAKE_SPEED = 0.5;
    const double ARM_ADJUSTER = 400;
    const double WRIST_ADJUSTER = 400;
    const double DANGER_ZONE_LIMIT = 50;

    //Constants
    const rev::CANSparkMax::MotorType BRUSHLESS = rev::CANSparkMax::MotorType::kBrushless;
    const rev::CANSparkMax::MotorType BRUSHED = rev::CANSparkMax::MotorType::kBrushed;
    const rev::CANSparkMax::IdleMode COAST = rev::CANSparkMax::IdleMode::kCoast;
    const rev::CANSparkMax::IdleMode BRAKE = rev::CANSparkMax::IdleMode::kBrake;
};