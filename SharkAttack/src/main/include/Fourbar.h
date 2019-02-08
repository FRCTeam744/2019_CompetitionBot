#pragma once

#include <rev/CANSparkMax.h>

class Fourbar{
    public:
        static Fourbar* GetInstance();  
        void ExtendBar(bool inputBar);
        void RetractBar(bool inputBar);
        void UpdateFourbarSpeed(double newSpeed);

    private:

        static Fourbar* s_instance;

        Fourbar(); 
    
        rev::CANSparkMax *fourbarExtender;

        double fourbarSpeed; 
                    
};