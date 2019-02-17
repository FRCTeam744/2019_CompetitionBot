/*----------------------------------------------------------------------------------*/

//Methods for the OI class (any and all things driver station/operator interface)

/*----------------------------------------------------------------------------------*/

#include "OI.h"
//#include "frc/smartdashboard/Smartdashboard.h"
//#include <frc/shuffleboard/Shuffleboard.h>
#include "Robot.h"

OI *OI::s_instance = 0;

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
    driveWithXbox = preferences->GetBoolean("driveWithXbox", false);

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
// void OI::SwitchDriveMode()
// {
//     if (xbox->GetStickButtonPressed(RIGHT_HAND))
//     {
//         if (!driveWithXbox)
//         {
//             driveWithXbox = true;
//             preferences->PutBoolean("driveWithXbox", true);
//         }
//         else
//         {
//             driveWithXbox = false;
//             preferences->PutBoolean("driveWithXbox", false);
//         }
//     }
// }

bool OI::SwitchGears()
{
    if (leftStick->GetRawButtonPressed(1))
    {
        isHighGear = false;
        frc::SmartDashboard::PutBoolean("isHighGear", false);
        return true;
    }
    if (rightStick->GetRawButtonPressed(1))
    {
        isHighGear = true;
        frc::SmartDashboard::PutBoolean("isHighGear", true);
        return true;
    }
    return false;
}

bool OI::GetIsHighGear()
{
    return isHighGear;
}

bool OI::SwitchGripper()
{
    if (xbox->GetBumper(RIGHT_HAND))
    {
        isGripperClosed = false;
        frc::SmartDashboard::PutBoolean("isGripperClosed", false);
        return true;
    }
    if (xbox->GetBumper(LEFT_HAND))
    {
        isGripperClosed = true;
        frc::SmartDashboard::PutBoolean("isGripperClosed", true);
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
    armPowerOutput = 0.0;
    armPowerOutput = xbox->GetY(LEFT_HAND) * ARM_POWER_SCALE;
    return armPowerOutput;
}

double OI::GetWristInput()
{
    wristPowerOutput = 0.0;
    wristPowerOutput = xbox->GetY(RIGHT_HAND) * WRIST_POWER_SCALE;
    return wristPowerOutput;
}

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

void OI::PutOnShuffleboard()
{
    if (isInitialized == false)
    {
        frc::ShuffleboardTab &Drivertab = frc::Shuffleboard::GetTab("DriverView");
        // Drivertab.Add("Pi4", 3.14).GetEntry();
        // Drivertab.Add("Max Speed2", 1).WithWidget("Number Slider").GetEntry();

        frc::ShuffleboardTab& PreCompTab = frc::Shuffleboard::GetTab("Pre-Comp Check");

        frc::ShuffleboardTab& ArmWristtab = frc::Shuffleboard::GetTab("Arm&Wrist Debug");

        frc::ShuffleboardTab& Visiontab = frc::Shuffleboard::GetTab("Vision Testing");
    isInitialized = true;
    }

    //    //test variables
    //     frc::SmartDashboard::PutBoolean("DriverView/bool", true);
    //     frc::SmartDashboard::PutBoolean("DriverView/bool2", true);
    //     frc::SmartDashboard::PutNumber("Smartdashboard/test5", 55);
    //     frc::SmartDashboard::PutNumber("Smartdashboard/test6", 6);
    //     frc::SmartDashboard::PutNumber("Smartdashboard/test7", 72);
    //     frc::SmartDashboard::PutBoolean("Smartdashboard/using gyro2?", false);
}

//Joysticks natively give out negative values when going forward, so adding the negative corrects it
// double OI::GetLeftDriveInput()
// {
//     if (driveWithXbox)
//     {
//         return -(xbox->GetY(LEFT_HAND));
//     }
//     else
//     {
//         return -(leftStick->GetY());
//     }
// }

// double OI::GetRightDriveInput()
// {
//     if (driveWithXbox)
//     {
//         return -(xbox->GetY(RIGHT_HAND));
//     }
//     else
//     {
//         return -(rightStick->GetY());
//     }
// }

void OI::PrintToSmartDashboard(double encoderValue)
{
    frc::SmartDashboard::PutNumber("Arm Encoder Value: ", encoderValue);
}

bool OI::GetFourbarExtend()
{
    return xbox->GetStartButtonPressed;
}

bool OI::GetFourbarRetract()
{
    return xbox->GetBackButtonPressed;
}

bool OI::SetPresetToAButton(){
    return xbox->GetAButton();
}

bool OI::SetPresetToBButton(){
    return xbox->GetBackButton();
}

bool OI::SetPresetToXButton(){
    return xbox->GetXButton();
}

bool OI::SetPresetToYButton(){
    return xbox->GetYButton();
}

bool OI::SetPresetToDPadUp(){
    return xbox->GetPOV(0);
}

bool OI::SetPresetToDPadDown(){
    return xbox->GetPOV(90);
}

bool OI::SetPresetToDPadLeft(){
    return xbox->GetPOV(180);
}

bool OI::SetPresetToDPadRight(){
    return xbox->GetPOV(270);
}


// void OI::SwitchLED_Mode(Drivetrain drivetrain) {

//     if (drivetrain.LimelightGet("ledMode") == 0){

//         drivetrain.LimelightPut("ledMode", 1);
//     }
//     else {
//         drivetrain.LimelightPut("ledMode", 0);
//     }
// }
