/*----------------------------------------------------------------------------------*/

//Header file for ShuffleManager.cpp

/*----------------------------------------------------------------------------------*/

#pragma once

#include "frc/WPILib.h"
#include <frc/smartdashboard/Smartdashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/Preferences.h>


class ShuffleManager {

    public:
    	static ShuffleManager *GetInstance();
		static frc::ShuffleboardTab *Drivertab;
		static frc::ShuffleboardTab *PreCompTab;
		static frc::ShuffleboardTab *ArmWristtab;
		static frc::ShuffleboardTab *Visiontab;

        static void ShuffleInit();
		static void OnShfl(frc::ShuffleboardTab *tab, char* label, double val);
		static void OnShfl(frc::ShuffleboardTab *tab, char* label, float val);
		static void OnShfl(frc::ShuffleboardTab *tab, char* label, int val);
		static void OnShfl(frc::ShuffleboardTab *tab, char* label, char* val);

    private:
        static ShuffleManager *s_instance;
        ShuffleManager();


};