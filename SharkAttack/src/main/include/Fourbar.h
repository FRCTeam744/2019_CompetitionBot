#pragma once

#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Ultrasonic.h>
#include <frc/DigitalInput.h>
#include <rev/CANEncoder.h>
#include <rev/CANPIDController.h>

#include "ShuffleManager.h"

/**
 @brief Class to handle all control of the fourbar.
 */
class Fourbar{
    public:
        static Fourbar* GetInstance();  

        /**
			@brief Triggers the speed controller for the fourbar
			@param extendBut true if wanting to extend the fourbar, false otherwise
			@param retractBut true if wanting to retract the fourbar, false otherwise

            Extention and retraction of the fourbar, taking into account protection from the 
            limit switch (magnetic read) sensors.
		*/
        void ExtendOrRetract(bool extendBut, bool retractBut);

        /**
			@brief Reads in a new speed from smart dashboard for the spped the fourbar will go.

            Used in testing, no longer used. Speed is now 1.0.
        */
        void UpdateFourbarSpeed();

		/**
			@brief Sets fourbar to home position.
			@param homingBut True if homing the fourbar.

            The fourbar needs to stow a little past the retracted limit switch. The homing routine
            takes care of slowly retracting the fourbar such that it can read the rising edge of the 
            retracted limit switch and then lower speed to correctly reach the desired millimeters beyond that
            rising edge.
		*/
        void FourbarHome(bool homingBut);

    	/**
			@brief Print method for Fourbar SB data. Called in Robot.cpp to print to ShuffleBoard
		*/
        void PrintFourbarShuffleInfo();
        
    private:

        static Fourbar* s_instance;

        Fourbar(); 

		/**
			@brief Method to read the extended limit switch (magnetic read) sensor
            @return true is the extended limit switch is tripped (i.e. the fourbar is fully extended), false otherwise

            Method to read the extended limit switch (magnetic read) sensor.
	    */
        bool IsExtendedTripped();

        /**
			@brief Method to read the retracted limit switch (magnetic read) sensor
            @return true is the retracted limit switch is tripped (i.e. the fourbar is fully retracted), false otherwise

            Method to read the retracted limit switch (magnetic read) sensor.
	    */
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

        const double DESIRED_MM_TO_HOME = 0.73;
        const double GEAR_RATIO = 3.0;
        const double MM_PER_REVOLUTIONS = 2.0;
        const double ROTATIONS_TO_HOME = DESIRED_MM_TO_HOME * GEAR_RATIO / MM_PER_REVOLUTIONS;

        //motor speeds
        const double EXTEND_SPEED = 1.0;
        const double RETRACT_SPEED = -1.0;
        const double HOMING_PRE_LIMIT_SWITCH_SPEED = -0.2;
        const double HOMING_POST_LIMIT_SWITCH_SPEED = -0.05;
        

};