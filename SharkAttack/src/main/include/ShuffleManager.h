/*----------------------------------------------------------------------------------*/

//Header file for ShuffleManager.cpp
//NOTE: commented out variables are functional. The list is tailored for our drive team or debugging
//so the NetworkTable isn't cluttered with empty or unused variables each startup

/*----------------------------------------------------------------------------------*/

#pragma once

#include "frc/WPILib.h"
#include <frc/smartdashboard/Smartdashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Preferences.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

class ShuffleManager {

    public:
    	static ShuffleManager *GetInstance();
		frc::ShuffleboardTab *DriverTab;
		//frc::ShuffleboardTab *PreCompTab;
		frc::ShuffleboardTab *ArmWristTab;
		frc::ShuffleboardTab *VisionTab;
		frc::ShuffleboardTab *FourbarTab;

		//This massive list creates variables to be stored in the NetworkTable.
		//Implements the native table hierarchy that RoboRio uses on startup
		//Naming convention: [Variable_Descriptor][Intended_Tab_Name] - because the same variable can be put on multiple tabs,
		//but needs to be its own NetworkTableEntry due to "variable is already existing" errors.
		//organized by class

		//Drivetrain.cpp
		nt::NetworkTableEntry leftDriveVision;
		nt::NetworkTableEntry rightDriveVision;
		// nt::NetworkTableEntry leftDriveDriver;
		// nt::NetworkTableEntry rightDriveDriver;
		nt::NetworkTableEntry headingVision;
		nt::NetworkTableEntry skewVision;
		nt::NetworkTableEntry currentDistanceInchesVision;
		// nt::NetworkTableEntry speedErrorRightPreComp;
		// nt::NetworkTableEntry speedErrorLeftPreComp;

		// nt::NetworkTableEntry limeLightXVision;
		// nt::NetworkTableEntry limelightYVision;
		// nt::NetworkTableEntry limeLightZVision;
		// nt::NetworkTableEntry limeLightFilteredXVision;
		// nt::NetworkTableEntry limelightFilteredYVision;
		// nt::NetworkTableEntry limeLightFilteredZVision;
		// nt::NetworkTableEntry limeLightRollVision;
		// nt::NetworkTableEntry limelightPitchVision;
		// nt::NetworkTableEntry limeLightYawVision;
		// nt::NetworkTableEntry limeLightFilteredRollVision;
		// nt::NetworkTableEntry limelightFilteredPitchVision;
		// nt::NetworkTableEntry limeLightFilteredYawVision;
		// nt::NetworkTableEntry limeLightThetaDesiredVision;
		// nt::NetworkTableEntry limelightThetaErrorVision;
		// nt::NetworkTableEntry limeLightXErrorVision;
		// nt::NetworkTableEntry limelightZErrorVision;

		// nt::NetworkTableEntry limeLightAngleErrorVision;
		// nt::NetworkTableEntry limeLightDistanceErrorVision;

		//Robot.cpp
		nt::NetworkTableEntry periodMatchTimeFMS;

		//Fourbar.cpp
		nt::NetworkTableEntry checkRetractedTrippedDriver;
		nt::NetworkTableEntry checkExtendedTrippedDriver;
		nt::NetworkTableEntry fourbarEncoderDriver;
		nt::NetworkTableEntry checkRetractedTrippedFourbar;
		nt::NetworkTableEntry checkExtendedTrippedFourbar;
		nt::NetworkTableEntry fourbarEncoderFourbar;
		nt::NetworkTableEntry fourbarRotationsToHomeFourbar;
		nt::NetworkTableEntry fourbarCurrentFourbar;
		nt::NetworkTableEntry fourbarRPMFourbar;
		nt::NetworkTableEntry fourbarSpeedFourbar;
		nt::NetworkTableEntry fourbarRotationsSinceRetractFourbar;

		//Arm.cpp
		nt::NetworkTableEntry armEncoderDriver;
		//nt::NetworkTableEntry armEncoderPreComp;
		nt::NetworkTableEntry wristEncoderDriver;
		//nt::NetworkTableEntry wristEncoderPreComp;
		nt::NetworkTableEntry leftArmCurrentArmWrist;
		nt::NetworkTableEntry rightArmCurrentArmWrist;
		nt::NetworkTableEntry armVelocityErrorArmWrist;
		nt::NetworkTableEntry armVelocityArmWrist;
		nt::NetworkTableEntry checkWristModeDriver;
		//nt::NetworkTableEntry checkWristModePreComp;
		nt::NetworkTableEntry checkWristModeArmWrist;
		nt::NetworkTableEntry targetArmPositionDegreesDriver;
		nt::NetworkTableEntry targetWristPositionDegreesDriver;
		nt::NetworkTableEntry targetArmPositionDegreesArmWrist;
		nt::NetworkTableEntry targetWristPositionDegreesArmWrist;
		nt::NetworkTableEntry checkArmManualDriver;
		nt::NetworkTableEntry checkWristManualDriver;
		nt::NetworkTableEntry checkArmManualArmWrist;
		nt::NetworkTableEntry checkWristManualArmWrist;
		// nt::NetworkTableEntry checkArmManualPreComp;
		// nt::NetworkTableEntry checkWristManualPreComp;
		// nt::NetworkTableEntry checkArmLimitSwitchDriver;
		// nt::NetworkTableEntry checkWristLimitSwitchDriver;
		// nt::NetworkTableEntry checkArmLimitSwitchArmWrist;
		// nt::NetworkTableEntry checkWristLimitSwitchArmWrist;
		// nt::NetworkTableEntry checkArmLimitSwitchPreComp;
		// nt::NetworkTableEntry checkWristLimitSwitchPreComp;

		//OI.cpp
		nt::NetworkTableEntry checkHatchGripperDriver;
		nt::NetworkTableEntry checkDriveTrainGearDriver;
		nt::NetworkTableEntry gyroYaw;

		/**
			@brief Creates tabs for ShuffleBoard to display on the Driver Station
		*/
        void TabInit();

		/**
			@brief Takes NetworkTableEntries and initializes them with placeholder values
		*/
		void VariableInit();
	
		/**
			@brief Organizer for updatable SB data. Is grouped within a method called in Robot.cpp in order to print to ShuffleBoard
			@param tab Specified tab that the variable is displayed on
			@param var Name of the NetworkTableEntry. Naming convention of [Variable_Descriptor][Intended_Tab_Name] must match the tab name, for both organization's sake and assuring there are no duplication errors
			@param val Value to replace the basic placeholder initialized in VariableInit(). val must be a double to be used in this method
		*/
		void OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, double val);

		/**
			@brief Organizer for updatable SB data. Is grouped within a method called in Robot.cpp in order to print to ShuffleBoard
			@param tab Specified tab that the variable is displayed on
			@param var Name of the NetworkTableEntry. Naming convention of [Variable_Descriptor][Intended_Tab_Name] must match the tab name, for both organization's sake and assuring there are no duplication errors
			@param val Value to replace the basic placeholder initialized in VariableInit(). val must be a float to be used in this method
		*/
		void OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, float val);

		/**
			@brief Organizer for updatable SB data. Is grouped within a method called in Robot.cpp in order to print to ShuffleBoard
			@param tab Specified tab that the variable is displayed on
			@param var Name of the NetworkTableEntry. Naming convention of [Variable_Descriptor][Intended_Tab_Name] must match the tab name, for both organization's sake and assuring there are no duplication errors
			@param val Value to replace the basic placeholder initialized in VariableInit(). val must be an int to be used in this method
		*/
		void OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, int val);

		/**
			@brief Organizer for updatable SB data. Is grouped within a method called in Robot.cpp in order to print to ShuffleBoard
			@param tab Specified tab that the variable is displayed on
			@param var Name of the NetworkTableEntry. Naming convention of [Variable_Descriptor][Intended_Tab_Name] must match the tab name, for both organization's sake and assuring there are no duplication errors
			@param val Value to replace the basic placeholder initialized in VariableInit(). val must be a String to be used in this method
		*/
		void OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, const char* val);

		/**
			@brief Organizer for updatable SB data. Is grouped within a method called in Robot.cpp in order to print to ShuffleBoard
			@param tab Specified tab that the variable is displayed on
			@param var Name of the NetworkTableEntry. Naming convention of [Variable_Descriptor][Intended_Tab_Name] must match the tab name, for both organization's sake and assuring there are no duplication errors
			@param val Value to replace the basic placeholder initialized in VariableInit(). val must be a boolean to be used in this method
		*/
		void OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, bool val);

    private:
        static ShuffleManager *s_instance;
        ShuffleManager();


};