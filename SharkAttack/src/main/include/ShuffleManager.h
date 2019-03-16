/*----------------------------------------------------------------------------------*/

//Header file for ShuffleManager.cpp

/*----------------------------------------------------------------------------------*/

#pragma once

#include "frc/WPILib.h"
#include <frc/smartdashboard/Smartdashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/Preferences.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

// #include <vector>  //for std::vector
// #include <string>  //for std::string

class ShuffleManager {

    public:
    	static ShuffleManager *GetInstance();
		frc::ShuffleboardTab *DriverTab;
		//frc::ShuffleboardTab *PreCompTab;
		frc::ShuffleboardTab *ArmWristTab;
		frc::ShuffleboardTab *VisionTab;
		frc::ShuffleboardTab *FourbarTab;

		//int shuffleUpdate = 1000;

		//using native hierarchy that RoboRio uses on startup
		//organized by class
		//Drivetrain.cpp
		nt::NetworkTableEntry leftDriveDriver;
		nt::NetworkTableEntry rightDriveDriver;
		// nt::NetworkTableEntry leftDrivePreComp;
		// nt::NetworkTableEntry rightDrivePreComp;
		nt::NetworkTableEntry headingVision;
		nt::NetworkTableEntry skewVision;
		nt::NetworkTableEntry currentDistanceInchesDriver;
		//nt::NetworkTableEntry speedErrorRightPreComp;
		// nt::NetworkTableEntry speedErrorLeftPreComp;

		nt::NetworkTableEntry limeLightXVision;
		nt::NetworkTableEntry limelightYVision;
		nt::NetworkTableEntry limeLightZVision;
		nt::NetworkTableEntry limeLightFilteredXVision;
		nt::NetworkTableEntry limelightFilteredYVision;
		nt::NetworkTableEntry limeLightFilteredZVision;
		nt::NetworkTableEntry limeLightRollVision;
		nt::NetworkTableEntry limelightPitchVision;
		nt::NetworkTableEntry limeLightYawVision;
		nt::NetworkTableEntry limeLightFilteredRollVision;
		nt::NetworkTableEntry limelightFilteredPitchVision;
		nt::NetworkTableEntry limeLightFilteredYawVision;
		nt::NetworkTableEntry limeLightThetaDesiredVision;
		nt::NetworkTableEntry limelightThetaErrorVision;
		nt::NetworkTableEntry limeLightXErrorVision;
		nt::NetworkTableEntry limelightZErrorVision;

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
		nt::NetworkTableEntry targetWristPositionDegreesDriver;
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



        void ShuffleInit();
		void OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, double val);
		void OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, float val);
		void OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, int val);
		void OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, const char* val);
		void OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, bool val);

		void VariableInit();
		//void GetShuffleVariable();

    private:
        static ShuffleManager *s_instance;
        ShuffleManager();


};