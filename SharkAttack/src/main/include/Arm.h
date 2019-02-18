/*----------------------------------------------------------------------------------*/

//Header file for Arm.cpp

/*----------------------------------------------------------------------------------*/

#pragma once
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>

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
    void MoveArmToPosition(double targetPosition, double wristCurrentPosition, double armCurrentPosition);   //Degrees
    void MoveWristToPosition(double wristCurrentPosition, double armCurrentPosition); //Degrees
    void CheckHatchGripper(bool isClosed);

    void ManualCalibrateArm();

    void PrintArmCurrent();


  private:
    static Arm *s_instance;
    Arm();

    bool GetArmLimitSwitch();
    bool GetWristLimitSwitch();

    //Private Objects
    rev::CANSparkMax *leftArm, *rightArm, *leftWrist, *rightWrist;
    TalonSRX *intake;

    rev::CANEncoder *armEncoder;
    rev::CANEncoder *wristEncoder;

    frc::DigitalInput *armLimitSwitch;
    frc::DigitalInput *wristLimitSwitch;

    frc::DoubleSolenoid *hatchGripper;

    bool wasArmLimitSwitchTripped;
    bool wasWristLimitSwitchTripped;

    //Tunables
    const double INTAKE_SPEED = 0.5;
    const double ARM_ADJUSTER = 400;
    const double WRIST_ADJUSTER = 400;
    const double DANGER_ZONE_LIMIT = 50;
    const double CALIBRATION_SPEED = 3000.0;
    const double OFFSET = -5.0; 

    //Constants
    const rev::CANSparkMax::MotorType BRUSHLESS = rev::CANSparkMax::MotorType::kBrushless;
    const rev::CANSparkMax::MotorType BRUSHED = rev::CANSparkMax::MotorType::kBrushed;
    const rev::CANSparkMax::IdleMode COAST = rev::CANSparkMax::IdleMode::kCoast;
    const rev::CANSparkMax::IdleMode BRAKE = rev::CANSparkMax::IdleMode::kBrake;
};