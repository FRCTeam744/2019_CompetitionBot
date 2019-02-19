/*----------------------------------------------------------------------------------*/

//Methods for the OI class (any and all things driver station/operator interface)

/*----------------------------------------------------------------------------------*/

#include "OI.h"
#include "ShuffleManager.h"


OI* OI::s_instance = 0;


//Static Singleton Method
OI *OI::GetInstance()
{
    if (s_instance == 0)
    {
        s_instance = new OI();
    }
    return s_instance;
}

// Constructor
OI::OI()
{

    preferences = frc::Preferences::GetInstance();

    //Intantiates the controller objects for the drivers
    leftStick = new frc::Joystick(0);
    rightStick = new frc::Joystick(1);
    xbox = new frc::XboxController(2);

    isHighGear = true;
    isGripperClosed = true;

   

    //Caps the camera quality to allow for driver vision
    // camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
    // camera.SetResolution(resolutionWidth, resolutionHeight);
    // camera.SetFPS(framerate);
}

//Public Methods

bool OI::SwitchGears() {
    if (leftStick->GetRawButtonPressed(1)) {
        isHighGear = false;
        //frc::SmartDashboard::PutBoolean("isHighGear", false);
        ShuffleManager::OnShfl(ShuffleManager::Drivertab, "isHighGear", isHighGear);
        ShuffleManager::OnShfl(ShuffleManager::PreCompTab, "isHighGear", isHighGear);
        return true;
    }
    if (rightStick->GetRawButtonPressed(1)) {
        isHighGear = true;
        //frc::SmartDashboard::PutBoolean("isHighGear", true);
        ShuffleManager::OnShfl(ShuffleManager::Drivertab, "isHighGear", isHighGear);
        ShuffleManager::OnShfl(ShuffleManager::PreCompTab, "isHighGear", isHighGear);
        return true;
    }
    return false;
}

bool OI::GetIsHighGear()
{
    return isHighGear;
}

bool OI::SwitchGripper(){
    if (xbox->GetBumperPressed(RIGHT_HAND)){
        isGripperClosed = false;
        //frc::SmartDashboard::PutBoolean("isGripperClosed", false);
        ShuffleManager::OnShfl(ShuffleManager::Drivertab, "isGripperClosed", isGripperClosed);
        ShuffleManager::OnShfl(ShuffleManager::ArmWristtab, "isGripperClosed", isGripperClosed);
        return true;
    }
    if (xbox->GetBumperPressed(LEFT_HAND)){
        isGripperClosed = true;
        //frc::SmartDashboard::PutBoolean("isGripperClosed", true);
        ShuffleManager::OnShfl(ShuffleManager::Drivertab, "isGripperClosed", isGripperClosed);
        ShuffleManager::OnShfl(ShuffleManager::ArmWristtab, "isGripperClosed", isGripperClosed);
        return true;
    }
    return false;
}

bool OI::GetIsGripperClosed()
{
    return isGripperClosed;
}

double OI::GetArmInput()
{
    armPowerOutput = xbox->GetY(LEFT_HAND);
    frc::SmartDashboard::PutNumber("Xbox Y Left", armPowerOutput);
    //shuffleboard here
    if (armPowerOutput < 0.07 && armPowerOutput > -0.07){
        armPowerOutput = 0.0;
    }
    return armPowerOutput;
}

double OI::GetWristInput()
{
    wristPowerOutput = 0.0;
    wristPowerOutput = xbox->GetY(RIGHT_HAND);
    return wristPowerOutput;
}

//positive degree values
double OI::ArmPresetPickupCargo()
{
    return ARM_PICKUP_CARGO_PRESET_DEG;
}
double OI::ArmPresetLow()
{
    return ARM_PICKUP_LOW_DEG;
}
double OI::ArmPresetMid()
{
    return ARM_PICKUP_MID_DEG;
}
double OI::ArmPresetHigh()
{
    return ARM_PICKUP_HIGH_DEG;
}

//negative degree
double OI::ArmPresetNegPickupCargo()
{
    return ARM_PICKUP_NEG_CARGO_PRESET_DEG;
}
double OI::ArmPresetNegLow()
{
    return ARM_PICKUP_NEG_LOW_DEG;
}
double OI::ArmPresetNegMid()
{
    return ARM_PICKUP_NEG_MID_DEG;
}
double OI::ArmPresetNegHigh()
{
    return ARM_PICKUP_NEG_HIGH_DEG;
}

// void OI::PutOnShuffleboardInOI(){

//     // if (isInitialized == false)
//     // {
// //         frc::ShuffleboardTab &Drivertab = frc::Shuffleboard::GetTab("DriverView");
// //         //Drivertab.Add("Pi4", 3.14).GetEntry();
// //         // Drivertab.Add("Max Speed2", 1).WithWidget("Number Slider").GetEntry();
// //         frc::ShuffleboardTab &PreCompTab = frc::Shuffleboard::GetTab("Pre-Comp Check");
// //         frc::ShuffleboardTab &ArmWristtab = frc::Shuffleboard::GetTab("Arm&Wrist Debug");
// //         frc::ShuffleboardTab &Visiontab = frc::Shuffleboard::GetTab("Vision Testing");
// //         isInitialized = true;
// //     }
// }



//Joysticks natively give out negative values when going forward, so adding the negative corrects it
double OI::GetLeftDriveInput()
{
    return -(leftStick->GetY());
}

double OI::GetRightDriveInput()
{
    return -(rightStick->GetY());
}

void OI::PrintToSmartDashboard(double encoderValue)
{
    frc::SmartDashboard::PutNumber("Arm Encoder Value: ", encoderValue);
}


bool OI::GetFourbarExtend()
{
    return xbox->GetStartButton();
}

bool OI::GetFourbarRetract()
{
    return xbox->GetBackButton();
}

bool OI::GetFourbarHome(){
    return rightStick->GetRawButtonPressed(8);
}

bool OI::SetPresetToAButton()
{
    return xbox->GetAButton(); //ARM_PICKUP_LOW_DEG
}

bool OI::SetPresetToBButton()
{
    return xbox->GetBButton(); //ARM_PICKUP_MID_DEG
}

bool OI::SetPresetToXButton()
{
    return xbox->GetXButton(); //ARM_PICKUP_HIGH_DEG
}

bool OI::SetPresetToYButton()
{
    return xbox->GetYButton(); //ARM_PICKUP_CARGO_PRESET_DEG
}

bool OI::SetPresetToDPadUp()
{
    return xbox->GetPOV(0); //ARM_PICKUP_NEG_CARGO_PRESET_DEG
}

bool OI::SetPresetToDPadDown()
{
    return xbox->GetPOV(90); //ARM_PICKUP_NEG_MID_DEG
}

bool OI::SetPresetToDPadLeft()
{
    return xbox->GetPOV(180); //ARM_PICKUP_NEG_HIGH_DEG
}

bool OI::SetPresetToDPadRight()
{
    return xbox->GetPOV(270); //ARM_PICKUP_NEG_LOW_DEG
}

std::tuple<bool, std::string, double> OI::SetLimelight() {
    if (leftStick->GetRawButtonPressed(5)){
        return std::make_tuple(true, "ledMode", 0.0);
    }
    else if (leftStick->GetRawButtonPressed(10)){
        return std::make_tuple(true, "ledMode", 1.0);
    }
    else if (leftStick->GetRawButtonPressed(6)){
        return std::make_tuple(true, "camMode", 0.0);
    }
    else if (leftStick->GetRawButtonPressed(9)){
        return std::make_tuple(true, "camMode", 1.0);
    }
    
    return std::make_tuple(false, "", 0.0);
}

bool OI::LEDButtonPressed() {
    return leftStick->GetRawButton(2);
}

bool OI::AlsoLEDButtonPressed() {
    return rightStick->GetRawButton(2);
}