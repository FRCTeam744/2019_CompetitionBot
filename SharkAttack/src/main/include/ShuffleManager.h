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
		frc::ShuffleboardTab *PreCompTab;
		frc::ShuffleboardTab *ArmWristTab;
		frc::ShuffleboardTab *VisionTab;
		frc::ShuffleboardTab *FourbarTab;

		nt::NetworkTableEntry testVal;
		nt::NetworkTableEntry armEncoder;

        void ShuffleInit();
		//void OnShfl(frc::ShuffleboardTab *tab, const char* label, double val);
		void OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, double val);
		void OnShfl(frc::ShuffleboardTab *tab, const char* label, float val);
		void OnShfl(frc::ShuffleboardTab *tab, const char* label, int val);
		void OnShfl(frc::ShuffleboardTab *tab, const char* label, const char* val);

		nt::NetworkTableEntry test;

		void VariableInit();
		void GetShuffleVariable();

    //bool isInitalized = false;


    private:
        static ShuffleManager *s_instance;
        ShuffleManager();


};