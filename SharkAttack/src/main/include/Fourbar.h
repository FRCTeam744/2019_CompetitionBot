#pragma once

#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Ultrasonic.h>
#include <frc/DigitalInput.h>
#include <rev/CANEncoder.h>
#include <rev/CANPIDController.h>

#include "ShuffleManager.h"

class Fourbar{
    public:
        static Fourbar* GetInstance();  
        void ExtendOrRetract(bool extendBut, bool retractBut);
        void UpdateFourbarSpeed();
        void FourbarHome(bool homingBut);
        void PrintFourbarShuffleInfo();
        
    private:

        static Fourbar* s_instance;

        Fourbar(); 

        bool IsExtendedTripped();
        bool IsRetractedTripped();

        bool isInitialized = false;
    
        rev::CANSparkMax *fourbarExtender;
        rev::CANEncoder *fourbarEncoder;

        double fourbarSpeed; 
        double newSpeed;

        frc::DigitalInput *extended;
        frc::DigitalInput *retracted;

        bool isPastRetracted;
        bool isPastExtended;
        bool isHoming;

        double rotationsSinceRetractTripped;
        double rotationsAtRetractTripped;

        const double DESIRED_MM_TO_HOME = 0.43;
        const double GEAR_RATIO = 5.0;
        const double MM_PER_REVOLUTIONS = 2.0;
        const double ROTATIONS_TO_HOME = DESIRED_MM_TO_HOME * GEAR_RATIO / MM_PER_REVOLUTIONS;

};