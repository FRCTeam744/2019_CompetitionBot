/*----------------------------------------------------------------------------------*/

//Header file for LED.cpp

/*----------------------------------------------------------------------------------*/

#pragma once

#include <frc/SerialPort.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

class LED {

    public:

        static LED* GetInstance();

        void LEDsOff();
        void SwimmingShark();
        void StartUp();
        void StartUpBlue();
        void StartUpRed();
        void LiftOffBlue();
        void LiftOffRed();
        void ShutDown();

        void HatchOrBallMode(bool isInBallMode);

    private:

        static LED* s_instance;
        LED();

        frc::SerialPort *arduino;
        

};