/*----------------------------------------------------------------------------------*/

//Methods for the OI class (any and all things driver station/operator interface)

/*----------------------------------------------------------------------------------*/

#include "OI.h"

OI* OI::s_instance = 0;

//Static Singleton Method
OI* OI::GetInstance() {
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
    if (xbox->GetStickButtonPressed(RIGHT_HAND)){
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

void OI::SwitchGears(){
    if (xbox->GetBumperPressed(LEFT_HAND)){
        lowGear = true;
        highGear = false;
        frc::SmartDashboard::PutBoolean ("Low Gear", true);
        frc::SmartDashboard::PutBoolean("High Gear", false);
    }
    if (xbox->GetBumperPressed(RIGHT_HAND)){
        highGear = true;
        lowGear = false;
        frc::SmartDashboard::PutBoolean("High Gear", true);
        frc::SmartDashboard::PutBoolean("Low Gear", false);
    }
}

//Joysticks natively give out negative values when going forward, so adding the negative corrects it
double OI::GetLeftDriveInput(){
    if (driveWithXbox) {
        return -(xbox->GetY(LEFT_HAND));
    }
    else {
        return -(leftStick->GetY());
    }
}

double OI::GetRightDriveInput(){
    if (driveWithXbox) {
        return -(xbox->GetY(RIGHT_HAND));
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
