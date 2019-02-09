/*----------------------------------------------------------------------------------*/

//Header file for Arm.cpp

/*----------------------------------------------------------------------------------*/

#pragma once
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>


class Arm {
    public:

        static Arm* GetInstance();

        void ManualRotateArm(double input);
        void ManualRotateWrist(double input);
        void Intake(bool buttonIsPressed);
        void AutoRotateArm(double position); //Doesn't do anything yet
        void AutoRotateWrist(double position); //Doesn't do anything yet
    
    private:
    
        static Arm* s_instance;
        Arm();

        //Private Objects
        rev::CANSparkMax *arm1, *arm2, *wrist, *intake;
        // Encoder *armEncoder;

        //Tunables
        const double INTAKE_SPEED = 0.5;

        //Constants
        const rev::CANSparkMax::MotorType BRUSHLESS = rev::CANSparkMax::MotorType::kBrushless;
        const rev::CANSparkMax::MotorType BRUSHED = rev::CANSparkMax::MotorType::kBrushed;
        const rev::CANSparkMax::IdleMode COAST = rev::CANSparkMax::IdleMode::kCoast;
        const rev::CANSparkMax::IdleMode BRAKE = rev::CANSparkMax::IdleMode::kBrake;
};