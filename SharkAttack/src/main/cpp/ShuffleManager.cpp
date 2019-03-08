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

//modify other onshuffles
//name values properly with string and setup (0) in variableinit
//go to whatever cpp and setup with skeleton
//call with  (name).set[thing]

void ShuffleManager::VariableInit(){
    testVal = ShuffleManager::DriverTab->Add("Ft-Sec-Left", 0.0).GetEntry();
    armEncoder = ShuffleManager::DriverTab->Add("New Arm Encoder", 0.0).GetEntry();
    // public NetworkTableEntry test2 = PreCompTab.Add("Ft-Sec-Right", 0).getEntry();
    // public NetworkTableEntry test = ArmWristTab.Add("Ft-Sec-Left", 0).getEntry();
    // public NetworkTableEntry test2 = VisionTab.Add("Ft-Sec-Right", 0).getEntry();
}

// void ShuffleManager::GetShuffleVariable(){
// //what goes in here?
// }

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, double val){
    if(frc::DriverStation::GetInstance().IsFMSAttached() == false || tab == DriverTab){ //negates everything on other tabs when FMS is connected
        // tab->Add(label, val).GetEntry();
        var.SetDouble(val);
    }    
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, const char* label, float val){
    if(frc::DriverStation::GetInstance().IsFMSAttached() == false || tab == DriverTab){
        tab->Add(label, val);
   }
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, const char* label, int val){
    if(frc::DriverStation::GetInstance().IsFMSAttached() == false || tab == DriverTab){
        tab->Add(label, val);
   }
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, const char* label, const char* val){
    
    if(frc::DriverStation::GetInstance().IsFMSAttached() == false || tab == DriverTab){
        tab->Add(label, val);
   }
}