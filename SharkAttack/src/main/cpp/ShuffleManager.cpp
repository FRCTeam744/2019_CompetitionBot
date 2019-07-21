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

//printing methods are all in Robot.cpp/RobotPeriodic:
//drivetrain->PrintDriveShuffleInfo()
//fourbar->PrintFourbarShuffleInfo()
//arm->PrintArmShuffleInfo();

//GetTab("Tab_Name") is what creates a new tab for SB to display
void ShuffleManager::TabInit(){
    DriverTab = &frc::Shuffleboard::GetTab("DriverView");
    //PreCompTab = &frc::Shuffleboard::GetTab("Pre-Comp Check");
    ArmWristTab = &frc::Shuffleboard::GetTab("Arm&Wrist Debug");
    VisionTab = &frc::Shuffleboard::GetTab("Vision Testing");
    FourbarTab = &frc::Shuffleboard::GetTab("Fourbar Testing");
}

//Takes NetworkTableEntries from ShuffleManager.h and initializes them with placeholders based on needed variable type
//The proper variable will be passed in to the OnShfl in Drivetrain.cpp, Fourbar.cpp, or Arm.cpp in respective methods (see above)
//and updated by the various print methods being called in Robot.cpp

//NOTE: commented out variables are functional. The list is tailored for our drive team or debugging
//so the NetworkTable isn't cluttered with empty variables
void ShuffleManager::VariableInit(){
    // leftDriveVision = ShuffleManager::DriverTab->Add("Ft-Sec-Left", 0.0).GetEntry();
    // rightDriveVision = ShuffleManager::DriverTab->Add("Ft-Sec-Right", 0.0).GetEntry();
    // leftDriveDriver = ShuffleManager::DriverTab->Add("Ft-Sec-Left", 0.0).GetEntry();
    // rightDriveDriver = ShuffleManager::DriverTab->Add("Ft-Sec-Right", 0.0).GetEntry();
    checkDriveTrainGearDriver = ShuffleManager::DriverTab->Add("In High Gear?", true).GetEntry();
    // speedErrorLeftPreComp = ShuffleManager::PreCompTab->Add("Speed Error Left", 0.0).GetEntry();
    // speedErrorRightPreComp = ShuffleManager::PreCompTab->Add("Speed Error Right", 0.0).GetEntry();    
    checkArmManualArmWrist = ShuffleManager::ArmWristTab->Add("Arm in Manual?", true).GetEntry();
    checkWristManualArmWrist = ShuffleManager::ArmWristTab->Add("Wrist in Manual?", true).GetEntry();
    checkWristManualDriver = ShuffleManager::DriverTab->Add("Wrist in Manual?", true).GetEntry();
    checkArmManualDriver = ShuffleManager::DriverTab->Add("Arm in Manual?", true).GetEntry();
    // checkArmManualPreComp = ShuffleManager::PreCompTab->Add("Arm in Manual?", true).GetEntry();
    // checkWristManualPreComp = ShuffleManager::PreCompTab->Add("Wrist in Manual?", true).GetEntry();
    

    armEncoderDriver = ShuffleManager::DriverTab->Add("Arm Encoder", 0.0).GetEntry();
    //armEncoderPreComp = ShuffleManager::PreCompTab->Add("Arm Encoder", 0.0).GetEntry();
    leftArmCurrentArmWrist = ShuffleManager::ArmWristTab->Add("Left Arm Curent", 0.0).GetEntry();
    rightArmCurrentArmWrist = ShuffleManager::ArmWristTab->Add("Right Arm Current", 0.0).GetEntry();
    checkHatchGripperDriver = ShuffleManager::DriverTab->Add("Hatch Gripper Gripping?", false).GetEntry(); //temp false
    checkWristModeDriver = ShuffleManager::DriverTab->Add("In Ball Mode?", false).GetEntry();
    checkWristModeArmWrist = ShuffleManager::ArmWristTab->Add("In Ball Mode?", false).GetEntry();
    // checkWristModePreComp = ShuffleManager::PreCompTab->Add("In Ball Mode?", false).GetEntry();
    wristEncoderDriver = ShuffleManager::DriverTab->Add("Wrist Encoder", 0.0).GetEntry();
    // wristEncoderPreComp = ShuffleManager::PreCompTab->Add("Wrist Encoder", 0.0).GetEntry();
    targetWristPositionDegreesDriver = ShuffleManager::DriverTab->Add("Wrist Target Position", 0.0).GetEntry();
    targetArmPositionDegreesDriver = ShuffleManager::DriverTab->Add("Arm Target Position", 0.0).GetEntry();
    targetWristPositionDegreesArmWrist = ShuffleManager::ArmWristTab->Add("Wrist Target Position", 0.0).GetEntry();
    targetArmPositionDegreesArmWrist = ShuffleManager::ArmWristTab->Add("Arm Target Position", 0.0).GetEntry();
    // checkArmLimitSwitchArmWrist = ShuffleManager::ArmWristTab->Add("Arm Limit Switch Tripped?", 0.0).GetEntry();
    // checkArmLimitSwitchDriver = ShuffleManager::DriverTab->Add("Arm Limit Switch Tripped?", 0.0).GetEntry();
    // checkArmLimitSwitchPreComp = ShuffleManager::PreCompTab->Add("Arm Limit Switch Tripped?", 0.0).GetEntry();
    // checkWristLimitSwitchArmWrist = ShuffleManager::ArmWristTab->Add("Wrist Limit Switch Tripped?", 0.0).GetEntry();
    // checkWristLimitSwitchDriver = ShuffleManager::DriverTab->Add("Wrist Limit Switch Tripped?", 0.0).GetEntry();
    // checkWristLimitSwitchPreComp = ShuffleManager::PreCompTab->Add("Wrist Limit Switch Tripped?", 0.0).GetEntry();
    
    gyroYaw = ShuffleManager::DriverTab->Add("Gyro Yaw", 0.0).GetEntry();

    headingVision = ShuffleManager::VisionTab->Add("Heading", 0.0).GetEntry();
    skewVision = ShuffleManager::VisionTab->Add("Skew", 0.0).GetEntry();
    currentDistanceInchesVision = ShuffleManager::VisionTab->Add("Current Distance Inches", 0.0).GetEntry();
    //limeLightAngleErrorVision = ShuffleManager::VisionTab->Add("Angle Error", 0.0).GetEntry();
    //limeLightDistanceErrorVision = ShuffleManager::VisionTab->("Distance Error", 0.0).GetEntry();

    fourbarEncoderDriver = ShuffleManager::DriverTab->Add("FB Encoder", 0.0).GetEntry();
    checkRetractedTrippedDriver = ShuffleManager::DriverTab->Add("Retracted?", true).GetEntry();
    checkExtendedTrippedDriver = ShuffleManager::DriverTab->Add("Extended?", false).GetEntry();
    fourbarEncoderFourbar = ShuffleManager::FourbarTab->Add("FB Encoder", 0.0).GetEntry();
    checkExtendedTrippedFourbar = ShuffleManager::FourbarTab->Add("Retracted?", true).GetEntry();
    checkRetractedTrippedFourbar = ShuffleManager::FourbarTab->Add("Extended?", false).GetEntry();
    fourbarCurrentFourbar = ShuffleManager::FourbarTab->Add("FB Current", 0.0).GetEntry();
    fourbarRotationsSinceRetractFourbar = ShuffleManager::FourbarTab->Add("FB Rotations Since Retract", 0.0).GetEntry();
    fourbarRPMFourbar = ShuffleManager::FourbarTab->Add("FB RPM", 0.0).GetEntry();
    fourbarRotationsToHomeFourbar = ShuffleManager::FourbarTab->Add("FB Rotations to Home", 0.0).GetEntry();
    fourbarSpeedFourbar = ShuffleManager::FourbarTab->Add("FB Speed", 0.0).GetEntry();

    periodMatchTimeFMS = ShuffleManager::DriverTab->Add("Approx. Match Time", 0.0).GetEntry();
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, double val){
    //negates everything on other tabs when FMS is connected - bandwidth and Rio CPU issues can happen if duplicate variables on multiple tabs update every 200ms
    if(frc::DriverStation::GetInstance().IsFMSAttached() == true){
       if(tab == DriverTab){
        //setDouble changes the 0.0 based on whatever parameter val is set to in the other cpp file
        var.SetDouble(val);
       }
    }
    //functions as normal when in the lab or at demos
    else
    {
        var.SetDouble(val);
    }
}

//precaution in case floats are used
void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, float val){
    //negates everything on other tabs when FMS is connected - bandwidth and Rio CPU issues can happen if duplicate variables on multiple tabs update every 200ms
    if(frc::DriverStation::GetInstance().IsFMSAttached() == true){
       if(tab == DriverTab){
        //setDouble changes the 0.0 based on whatever parameter val is set to in the other cpp file
        //"setFloat" doesn't exist, so cast into double first
        var.SetDouble((double)val);
       }
    }
    //functions as normal when in the lab or at demos
    else
    {
        var.SetDouble((double)val);
    }
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, int val){
    //negates everything on other tabs when FMS is connected - bandwidth and Rio CPU issues can happen if duplicate variables on multiple tabs update every 200ms
    if(frc::DriverStation::GetInstance().IsFMSAttached() == true){
        if(tab == DriverTab){
        //setDouble changes the 0.0 based on whatever parameter val is set to in the other cpp file
        //"setInt" doesn't exist, so cast into double first
        var.SetDouble((double)val);
       }
    }
    //functions as normal when in the lab or at demos
    else
    {
        var.SetDouble((double)val);
    }
}

void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, const char* val){
    //negates everything on other tabs when FMS is connected - bandwidth and Rio CPU issues can happen if duplicate variables on multiple tabs update every 200ms
    if(frc::DriverStation::GetInstance().IsFMSAttached() == true){
       if(tab == DriverTab){
       //setString changes based on whatever parameter val is set to in the other cpp file
       var.SetString(val); 
       }
    }
    //functions as normal when in the lab or at demos
    else
    {
        var.SetString(val);
    }
}


void ShuffleManager::OnShfl(frc::ShuffleboardTab *tab, nt::NetworkTableEntry var, bool val){
    //negates everything on other tabs when FMS is connected - bandwidth and Rio CPU issues can happen if duplicate variables on multiple tabs update every 200ms
    if(frc::DriverStation::GetInstance().IsFMSAttached() == true){
       if(tab == DriverTab){
        //setBoolean changes the inital true or false from VariableInit() based on what val is set to in the other cpp file
        var.SetBoolean(val);
       }
    }
    //functions as normal when in the lab or at demos
    else
    {
        var.SetBoolean(val);
    }
}