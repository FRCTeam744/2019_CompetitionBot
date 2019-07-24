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

        /**
			@brief Triggers the speed controller for the fourbar
			@param tab Specified tab that the variable is displayed on
			@param var Name of the NetworkTableEntry. Naming convention of [Variable_Descriptor][Intended_Tab_Name] must match the tab name, for both organization's sake and assuring there are no duplication errors
		*/
        void ExtendOrRetract(bool extendBut, bool retractBut);

        /**
			@brief Basic print statement for fourbar speed
        */
        void UpdateFourbarSpeed();

		/**
			@brief Sets fourbar to home position?
			@param homingBut ??
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
			@brief Organizer for updatable SB data. Is grouped within a method called in Robot.cpp in order to print to ShuffleBoard
	    */
        bool IsExtendedTripped();

        /**
			@brief Organizer for updatable SB data. Is grouped within a method called in Robot.cpp in order to print to ShuffleBoard
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

};