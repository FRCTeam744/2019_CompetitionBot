/*----------------------------------------------------------------------------------*/

//Methods for the OI class (any and all things driver station/operator interface)

/*----------------------------------------------------------------------------------*/

#include "OI.h"

//Public Methods
void OI::OI_Init() {

    preferences = frc::Preferences::GetInstance();
    driveWithXbox = preferences->GetBoolean("driveWithXbox", false);

    //Intantiates the controller objects for the drivers
    leftStick = new frc::Joystick(0);
    rightStick = new frc::Joystick(1);
    xbox = new frc::XboxController(2);

    //Caps the camera quality to allow for driver vision
    cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
    camera.SetResolution(160, 120);
    camera.SetFPS(30);
}

void OI::TurnOffLEDs() {
    if (xbox->GetAButtonPressed())
    {
        driveWithXbox = true;
        preferences->PutBoolean("drive with xbox", true);
        // table->PutNumber("camMode", 1.0);
    }
    if (xbox->GetBButtonPressed())
    {
        driveWithXbox = false;
        preferences->PutBoolean("drive with xbox", false);
        // table->PutNumber("camMode", 0.0);
    }
    if (xbox->GetXButtonPressed())
    {
        // arcadeDrive = true;
        // preferences->PutBoolean("arcade drive", true);
        table->PutNumber("ledMode", 1);
    }
    if (xbox->GetYButtonPressed())
    {
        // arcadeDrive = false;
        // preferences->PutBoolean("arcade drive", false);
        table->PutNumber("ledMode", 0);
    }
}