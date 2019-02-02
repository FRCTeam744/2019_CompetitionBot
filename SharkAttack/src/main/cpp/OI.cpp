/*----------------------------------------------------------------------------------*/

//Methods for the OI class (any and all things driver station/operator interface)

/*----------------------------------------------------------------------------------*/

#include "OI.h"

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

double OI::GetLeftDriveInput(){
    if (driveWithXbox) {
        xbox->GetY(leftHand);
    }
    else {
        leftStick->GetY();
    }
}

double OI::GetRightDriveInput(){
    if (driveWithXbox) {
        xbox->GetY(rightHand);
    }
    else {
        rightStick->GetY();
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
