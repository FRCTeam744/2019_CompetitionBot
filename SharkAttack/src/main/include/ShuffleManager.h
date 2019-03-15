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


		//OI.cpp?

		//Fourbar.cpp
		nt::NetworkTableEntry fourbarSpeedFourbar;

		//Arm.cpp
		nt::NetworkTableEntry armEncoderDriver;
		//nt::NetworkTableEntry armEncoderPreComp;
		nt::NetworkTableEntry wristEncoderDriver;
		//nt::NetworkTableEntry wristEncoderPreComp;
		nt::NetworkTableEntry leftArmCurrentArmWrist;
		nt::NetworkTableEntry rightArmCurrentArmWrist;

        void ShuffleInit();
		void OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, double val);
		void OnShfl(frc::ShuffleboardTab *tab, const char* label, float val);
		void OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, int val);
		void OnShfl(frc::ShuffleboardTab *tab, const char* label, const char* val);

		void VariableInit();
		//void GetShuffleVariable();

    private:
        static ShuffleManager *s_instance;
        ShuffleManager();


};