#pragma once

#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Ultrasonic.h>
#include <frc/DigitalInput.h>
#include <rev/CANEncoder.h>

class Fourbar{
    public:
        static Fourbar* GetInstance();  
        void ExtendOrRetract(bool extendBut, bool retractBut);
        void UpdateFourbarSpeed();

        void PrintClimberRPM();

    private:

        static Fourbar* s_instance;

        Fourbar(); 
    
        rev::CANSparkMax *fourbarExtender;
        rev::CANEncoder *fourbarEncoder;

        double fourbarSpeed; 
        double newSpeed;

        frc::DigitalInput *extended;
        frc::DigitalInput *retracted;

        bool retractedTripped;
        bool extendedTripped;
                    
        
};