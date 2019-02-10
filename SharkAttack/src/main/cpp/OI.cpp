/*----------------------------------------------------------------------------------*/

//Methods for the OI class (any and all things driver station/operator interface)

/*----------------------------------------------------------------------------------*/

#include "OI.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include "Robot.h"

#define SMARTDASHBOARD



OI* OI::s_instance = 0;

OI* OI::getInstance() {
    if (s_instance == 0) {
        s_instance = new OI();
    }
    return s_instance;
}

// Constructor
OI::OI() {

    preferences = frc::Preferences::GetInstance();
    driveWithXbox = preferences->GetBoolean("driveWithXbox", false);

    //Intantiates the controller objects for the drivers
    leftStick = new frc::Joystick(0);
    rightStick = new frc::Joystick(1);
    xbox = new frc::XboxController(2);

    //Caps the camera quality to allow for driver vision
    // camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
    // camera.SetResolution(resolutionWidth, resolutionHeight);
    // camera.SetFPS(framerate);
}

//Public Methods
void OI::SwitchDriveMode(){
    if (xbox->GetStickButtonPressed(rightHand)){
        if (!driveWithXbox){
            driveWithXbox = true;
            preferences->PutBoolean("driveWithXbox", true);
        }
        else {
            driveWithXbox = false;
            preferences->PutBoolean("driveWithXbox", false);
        }
    }
}


void OI::PutOnShuffleboard(){

if(isInitialized == false){
    frc::ShuffleboardTab& Drivertab = frc::Shuffleboard::GetTab("DriverView");
    // Drivertab.Add("Pi4", 3.14).GetEntry();
    // Drivertab.Add("Max Speed2", 1).WithWidget("Number Slider").GetEntry();
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
double OI::GetLeftDriveInput(){
    if (driveWithXbox) {
        return -(xbox->GetY(leftHand));
    }
    else {
        return -(leftStick->GetY());
    }
}

double OI::GetRightDriveInput(){
    if (driveWithXbox) {
        return -(xbox->GetY(rightHand));
    }
    else {
        return -(rightStick->GetY());
    }
}

// void OI::SwitchLED_Mode(Drivetrain drivetrain) {

//     if (drivetrain.LimelightGet("ledMode") == 0){

//         drivetrain.LimelightPut("ledMode", 1);
//     }
//     else {
//         drivetrain.LimelightPut("ledMode", 0);
//     }
// }
