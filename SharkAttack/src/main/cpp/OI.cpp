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


void OI::PutOnShuffleboardTest(){
#if 1
    frc::SmartDashboard::PutBoolean("CatDog", true);
    frc::SmartDashboard::PutNumber("test1", 25);
    frc::SmartDashboard::PutNumber("test2", 55);
    frc::SmartDashboard::PutNumber("test3", 6);
    frc::SmartDashboard::PutNumber("Smartdashboard/test4", 72);
    frc::SmartDashboard::PutBoolean("using_gyro?", false);
#endif

#if 1
 //   frc::Shuffleboard::
 //frc::Shuffleboard::GetInstance().;
 
    //frc::Shuffleboard::GetInstance().

    frc::ShuffleboardTab& Drivertab = frc::Shuffleboard::GetTab("DriverView");
    //private NetworkTableEntry distanceEntry = tab.AddPersistent("Dist. to target", 0).getEntry();
    Drivertab.Add("Pi3", 3.14).GetEntry();
   // Drivertab.AddPersistent("pi2",3.14);

    Drivertab.AddPersistent("Smartdashboard/test4", 25);
    frc::SmartDashboard::PutBoolean("DriverView/bool", true);
    frc::SmartDashboard::PutBoolean("DriverView/bool2", true);
    frc::SmartDashboard::PutNumber("Smartdashboard/test5", 55);
    frc::SmartDashboard::PutNumber("Smartdashboard/test6", 6);
    frc::SmartDashboard::PutNumber("Smartdashboard/test7", 72);
    frc::SmartDashboard::PutBoolean("Smartdashboard/using gyro2?", false);
#endif

// frc::ShuffleboardTab& elevatorTab = frc::Shuffleboard::GetTab("Elevator");
//    // elevatorTab.Add("test", rightBack);

// // //making tab
// frc::ShuffleboardTab& testTab = frc::Shuffleboard::GetTab("Test1");
// testTab.Add("Max Speed", 1);
    


//     //encoder list test
//     // //frc::ShuffleboardLayout& encoders =
//     // frc::Shuffleboard::SelectTab("SmartDashboard");


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
