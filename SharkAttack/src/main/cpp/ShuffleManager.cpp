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

void ShuffleManager::ShuffleInit(){ //variables were instantiated in .h, giving them values/initializing them here
    DriverTab = &frc::Shuffleboard::GetTab("DriverView");
   // PreCompTab = &frc::Shuffleboard::GetTab("Pre-Comp Check");
    ArmWristTab = &frc::Shuffleboard::GetTab("Arm&Wrist Debug");
    VisionTab = &frc::Shuffleboard::GetTab("Vision Testing");
    FourbarTab = &frc::Shuffleboard::GetTab("Fourbar Testing");
}

void ShuffleManager::VariableInit(){ //variables were instantiated in .h, giving them values/initializing them here
    leftDriveDriver = ShuffleManager::DriverTab->Add("Ft-Sec-Left", 0.0).GetEntry();
    rightDriveDriver = ShuffleManager::DriverTab->Add("Ft-Sec-Right", 0.0).GetEntry();
    // leftDrivePreComp = ShuffleManager::PreCompTab->Add("Ft-Sec-Left", 0.0).GetEntry();
    // rightDrivePreComp = ShuffleManager::PreCompTab->Add("Ft-Sec-Right", 0.0).GetEntry();
    currentDistanceInchesDriver = ShuffleManager::DriverTab->Add("Current Distance Inches", 0.0).GetEntry();    
    // speedErrorLeftPreComp = ShuffleManager::PreCompTab->Add("Speed Error Left", 0.0).GetEntry();
    // speedErrorRightPreComp = ShuffleManager::PreCompTab->Add("Speed Error Right", 0.0).GetEntry();
    armEncoderDriver = ShuffleManager::DriverTab->Add("Arm Encoder", 0.0).GetEntry();
    //armEncoderPreComp = ShuffleManager::PreCompTab->Add("Arm Encoder", 0.0).GetEntry();
    leftArmCurrentArmWrist = ShuffleManager::ArmWristTab->Add("Left Arm Curent", 0.0).GetEntry();
    rightArmCurrentArmWrist = ShuffleManager::ArmWristTab->Add("Right Arm Current", 0.0).GetEntry();
    wristEncoderDriver = ShuffleManager::DriverTab->Add("Wrist Encoder", 0.0).GetEntry();
    //wristEncoderPreComp = ShuffleManager::PreCompTab->Add("Wrist Encoder", 0.0).GetEntry();

    fourbarSpeedFourbar = ShuffleManager::FourbarTab->Add("FB Speed", 0.0).GetEntry();
}

// void ShuffleManager::GetShuffleVariable(){
// //what goes in here?
// }

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, double val){
    if(frc::DriverStation::GetInstance().IsFMSAttached() == true){ //negates everything on other tabs when FMS is connected
        if(tab == DriverTab){
             var.SetDouble(val);
        } //setDouble changes the 0.0 based on whatever parameter val is set to in the other cpp file
    }
    else if(frc::DriverStation::GetInstance().IsFMSAttached() == false){
        if(tab == VisionTab || tab == ArmWristTab || tab == FourbarTab){
            var.SetDouble(val);
        }
    }   
}

// void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, const char* label, float val){
//     if(frc::DriverStation::GetInstance().IsFMSAttached() == false || tab == DriverTab){
//         tab->Add(label, val);
//    }
// }

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, float val){
    if(frc::DriverStation::GetInstance().IsFMSAttached() == true){ //negates everything on other tabs when FMS is connected
        if(tab == DriverTab){
             var.SetDouble(val);
        } //setDouble changes the 0.0 based on whatever parameter val is set to in the other cpp file
    }
    else if(frc::DriverStation::GetInstance().IsFMSAttached() == false){
        if(tab == VisionTab || tab == ArmWristTab || tab == FourbarTab){
            var.SetDouble(val);
        }
    }   
}

// void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, int val){
//     if(frc::DriverStation::GetInstance().IsFMSAttached() == false || tab == DriverTab){
//         var.SetDouble((int)val); //check this, no getInt exists
//    }
// }

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, int val){
    if(frc::DriverStation::GetInstance().IsFMSAttached() == true){ //negates everything on other tabs when FMS is connected
        if(tab == DriverTab){
             var.SetDouble((int)val);
        } //setDouble changes the 0.0 based on whatever parameter val is set to in the other cpp file
    }
    else if(frc::DriverStation::GetInstance().IsFMSAttached() == false){
        if(tab == VisionTab || tab == ArmWristTab || tab == FourbarTab){
            var.SetDouble((int)val);
        }
    }   
}

// void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, const char* label, const char* val){
//     if(frc::DriverStation::GetInstance().IsFMSAttached() == false || tab == DriverTab){
//         tab->Add(label, val);
//    }
// }

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, const char* val){
    if(frc::DriverStation::GetInstance().IsFMSAttached() == true){ //negates everything on other tabs when FMS is connected
        if(tab == DriverTab){
             var.SetString(val);
        } //setDouble changes the 0.0 based on whatever parameter val is set to in the other cpp file
    }
    else if(frc::DriverStation::GetInstance().IsFMSAttached() == false){
        if(tab == VisionTab || tab == ArmWristTab || tab == FourbarTab){
            var.SetString(val);
        }
    }   
}

// void OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, bool val){
//     if(frc::DriverStation::GetInstance().IsFMSAttached() == true){ //negates everything on other tabs when FMS is connected
//         if(tab == DriverTab){
//              var.SetBoolean(val);
//         } //setDouble changes the 0.0 based on whatever parameter val is set to in the other cpp file
//     }
//     else if(frc::DriverStation::GetInstance().IsFMSAttached() == false){
//         if(tab == VisionTab || tab == ArmWristTab || tab == FourbarTab){
//             var.SetBoolean(val);
//         }
//     }   
// }
