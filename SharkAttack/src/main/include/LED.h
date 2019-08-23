/*----------------------------------------------------------------------------------*/

//Header file for LED.cpp

/*----------------------------------------------------------------------------------*/

#pragma once

#include <frc/SerialPort.h>
#include <frc/DigitalOutput.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

/**
 @brief Each method in this class configures the four DIO outputs to a different 
        predetermined combination to send to a connected offboard microprocessor
        that controls a programmable LED strip
 */
class LED {

    public:

        static LED* GetInstance();

        /**
        @brief Sends an output combination to turn the LED strip off
        */
        void LEDsOff();
        /**
        @brief Sends an output combination to begin a swimming shark animation
        */
        void SwimmingShark();
        /**
        @brief Sends an output combination to begin a startup animation
        */
        void StartUp();
        /**
        @brief Sends an output combination to begin a blue startup animation
        */
        void StartUpBlue();
        /**
        @brief Sends an output combination to begin a red startup animation
        */
        void StartUpRed();
        /**
        @brief Sends an output combination to begin a blue lift off animation
        */
        void LiftOffBlue();
        /**
        @brief Sends an output combination to begin a red lift off animation
        */
        void LiftOffRed();
        /**
        @brief Sends an output combination to begin a shutdown animation
        */
        void ShutDown();

        /**
        @brief Sends an output combination to change the color of the leds based
               on the intake mode of the arm
        @param isInBallMode Whether the arm is in ball intake mode or hatch intake mode
        @param isVisionTracking Whether the robot is running a vision tracking routine
        */
        void HatchOrBallMode(bool isInBallMode, bool isVisionTracking);

        /**
        @brief Sends an output combination to change the color of the leds based
               on the state of the gripper
        @param isHatchGripperOpen Whether the hatch gripper is open or closed
        @param isVisionTracking Whether the robot is running a vision tracking routine
        */
        void IsHatchOpen(bool isHatchGripperOpen, bool isVisionTracking);

    private:

        static LED* s_instance;
        LED();

        frc::SerialPort *arduino;

        frc::DigitalOutput *byte1, *byte2, *byte3, *byte4;
        

};