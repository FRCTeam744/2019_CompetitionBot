/*----------------------------------------------------------------------------------*/

//Methods for the Shuffle Manager class (any and all things shuffleboard)

/*----------------------------------------------------------------------------------*/

#include "ShuffleManager.h"

ShuffleManager* ShuffleManager::s_instance = 0;

ShuffleManager *ShuffleManager::GetInstance(){
    if (s_instance == 0)
    {
        s_instance = new ShuffleManager();
    }
    return s_instance;
}

ShuffleManager::ShuffleManager(){
    
}

void ShuffleManager::ShuffleInit(){
    Drivertab = &frc::Shuffleboard::GetTab("DriverView");
    PreCompTab = &frc::Shuffleboard::GetTab("Pre-Comp Check");
    ArmWristtab = &frc::Shuffleboard::GetTab("Arm&Wrist Debug");
    Visiontab = &frc::Shuffleboard::GetTab("Vision Testing");
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, char* label, double val){
   if(!(frc::DriverStation::GetInstance().IsFMSAttached() == true) || tab == Drivertab){
        tab->Add(label, val);
   }
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, char* label, float val){
    if(!(frc::DriverStation::GetInstance().IsFMSAttached() == true) || tab == Drivertab){
        tab->Add(label, val);
   }
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, char* label, int val){
    if(!(frc::DriverStation::GetInstance().IsFMSAttached() == true) || tab == Drivertab){
        tab->Add(label, val);
   }
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, char* label, char* val){
    if(!(frc::DriverStation::GetInstance().IsFMSAttached() == true) || tab == Drivertab){
        tab->Add(label, val);
   }
}