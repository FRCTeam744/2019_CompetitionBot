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
    PreCompTab = &frc::Shuffleboard::GetTab("Pre-Comp Check");
    ArmWristTab = &frc::Shuffleboard::GetTab("Arm&Wrist Debug");
    VisionTab = &frc::Shuffleboard::GetTab("Vision Testing");
    FourbarTab = &frc::Shuffleboard::GetTab("Fourbar Testing");
}

void ShuffleManager::VariableInit(){ //variables were declared in .h, giving them values/initializing them here
    leftDriveDriver = ShuffleManager::DriverTab->Add("Ft-Sec-Left", 0.0).GetEntry();
    rightDriveDriver = ShuffleManager::DriverTab->Add("Ft-Sec-Right", 0.0).GetEntry();
    leftDrivePreComp = ShuffleManager::PreCompTab->Add("Ft-Sec-Left", 0.0).GetEntry();
    rightDrivePreComp = ShuffleManager::PreCompTab->Add("Ft-Sec-Right", 0.0).GetEntry();
    checkDriveTrainGearDriver = ShuffleManager::DriverTab->Add("In High Gear?", false).GetEntry(); //temp false 
    speedErrorLeftPreComp = ShuffleManager::PreCompTab->Add("Speed Error Left", 0.0).GetEntry();
    speedErrorRightPreComp = ShuffleManager::PreCompTab->Add("Speed Error Right", 0.0).GetEntry();    
    checkArmManualArmWrist = ShuffleManager::ArmWristTab->Add("Arm in Manual?", true).GetEntry();
    checkWristManualArmWrist = ShuffleManager::ArmWristTab->Add("Wrist in Manual?", true).GetEntry();
    checkWristManualDriver = ShuffleManager::DriverTab->Add("Wrist in Manual?", true).GetEntry();
    checkArmManualDriver = ShuffleManager::DriverTab->Add("Arm in Manual?", true).GetEntry();
    checkArmManualPreComp = ShuffleManager::PreCompTab->Add("Arm in Manual?", true).GetEntry();
    checkWristManualPreComp = ShuffleManager::PreCompTab->Add("Wrist in Manual?", true).GetEntry();
    

    armEncoderDriver = ShuffleManager::DriverTab->Add("Arm Encoder", 0.0).GetEntry();
    armEncoderPreComp = ShuffleManager::PreCompTab->Add("Arm Encoder", 0.0).GetEntry();
    leftArmCurrentArmWrist = ShuffleManager::ArmWristTab->Add("Left Arm Curent", 0.0).GetEntry();
    rightArmCurrentArmWrist = ShuffleManager::ArmWristTab->Add("Right Arm Current", 0.0).GetEntry();
    checkHatchGripperDriver = ShuffleManager::DriverTab->Add("Hatch Gripper Open?", false).GetEntry(); //temp false
    checkWristModeDriver = ShuffleManager::DriverTab->Add("In Ball Mode?", false).GetEntry();
    checkWristModeArmWrist = ShuffleManager::ArmWristTab->Add("In Ball Mode?", false).GetEntry();
    checkWristModePreComp = ShuffleManager::PreCompTab->Add("In Ball Mode?", false).GetEntry();
    wristEncoderDriver = ShuffleManager::DriverTab->Add("Wrist Encoder", 0.0).GetEntry();
    wristEncoderPreComp = ShuffleManager::PreCompTab->Add("Wrist Encoder", 0.0).GetEntry();
    targetWristPositionDegreesDriver = ShuffleManager::DriverTab->Add("Wrist Target Position", 0.0).GetEntry();

    
    headingVision = ShuffleManager::VisionTab->Add("Heading", 0.0).GetEntry();
    skewVision = ShuffleManager::VisionTab->Add("Skew", 0.0).GetEntry();
    currentDistanceInchesVision = ShuffleManager::VisionTab->Add("Current Distance Inches", 0.0).GetEntry();

    fourbarEncoderDriver = ShuffleManager::DriverTab->Add("FB Encoder", 0.0).GetEntry();
    checkRetractedTrippedDriver = ShuffleManager::DriverTab->Add("Retracted?", true).GetEntry();
    checkExtendedTrippedDriver = ShuffleManager::DriverTab->Add("Extended?", true).GetEntry();
    fourbarEncoderFourbar = ShuffleManager::FourbarTab->Add("FB Encoder", 0.0).GetEntry();
    checkExtendedTrippedFourbar = ShuffleManager::FourbarTab->Add("Retracted?", true).GetEntry();
    checkRetractedTrippedFourbar = ShuffleManager::FourbarTab->Add("Extended?", true).GetEntry();
    fourbarCurrentFourbar = ShuffleManager::FourbarTab->Add("FB Current", 0.0).GetEntry();
    fourbarRotationsSinceRetractFourbar = ShuffleManager::FourbarTab->Add("FB Rotations Since Retract", 0.0).GetEntry();
    fourbarRPMFourbar = ShuffleManager::FourbarTab->Add("FB RPM", 0.0).GetEntry();
    fourbarRotationsToHomeFourbar = ShuffleManager::FourbarTab->Add("FB Rotations to Home", 0.0).GetEntry();
    fourbarSpeedFourbar = ShuffleManager::FourbarTab->Add("FB Speed", 0.0).GetEntry();
}

// void ShuffleManager::GetShuffleVariable(){
// //what goes in here?
// }

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, double val){
    if(frc::DriverStation::GetInstance().IsFMSAttached() == false || tab == DriverTab){ //negates everything on other tabs when FMS is connected
        var.SetDouble(val); //setDouble changes the 0.0 based on whatever parameter val is set to in the other cpp file
    }    
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, float val){
    if(frc::DriverStation::GetInstance().IsFMSAttached() == false || tab == DriverTab){
        var.SetDouble((double)val);
   }
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, int val){
    if(frc::DriverStation::GetInstance().IsFMSAttached() == false || tab == DriverTab){
        var.SetDouble((int)val);
   }
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, const char* val){
    if(frc::DriverStation::GetInstance().IsFMSAttached() == false || tab == DriverTab){
        var.SetString(val);
   }
}
void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, bool val){
    if(frc::DriverStation::GetInstance().IsFMSAttached() == false || tab == DriverTab){
       var.SetBoolean(val);
   }
}