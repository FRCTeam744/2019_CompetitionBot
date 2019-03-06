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
    DriverTab = &frc::Shuffleboard::GetTab("DriverView");
    PreCompTab = &frc::Shuffleboard::GetTab("Pre-Comp Check");
    ArmWristTab = &frc::Shuffleboard::GetTab("Arm&Wrist Debug");
    VisionTab = &frc::Shuffleboard::GetTab("Vision Testing");
    FourbarTab = &frc::Shuffleboard::GetTab("Fourbar Testing");
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, const char* label, double val){
    if(!(frc::DriverStation::GetInstance().IsFMSAttached() == true) || tab == DriverTab){ //negates everything on other tabs when FMS is connected
        tab->Add(label, val).GetEntry();
    }    
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, const char* label, float val){
    if(!(frc::DriverStation::GetInstance().IsFMSAttached() == true) || tab == DriverTab){
        tab->Add(label, val).GetEntry();
   }
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, const char* label, int val){
    if(!(frc::DriverStation::GetInstance().IsFMSAttached() == true) || tab == DriverTab){
        tab->Add(label, val).GetEntry();
   }
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, const char* label, const char* val){
    if(!(frc::DriverStation::GetInstance().IsFMSAttached() == true) || tab == DriverTab){
        tab->Add(label, val).GetEntry();
   }
}