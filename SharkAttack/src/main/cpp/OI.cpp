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

    isHighGear = true;
    isGripperClosed = true;

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

bool OI::SwitchGears(){
    if (leftStick->GetRawButtonPressed(1)){
        isHighGear = false;
        frc::SmartDashboard::PutBoolean("isHighGear", false);
        return true;
    }
    if (rightStick->GetRawButtonPressed(1)){
        isHighGear = true;
        frc::SmartDashboard::PutBoolean("isHighGear", true);
        return true;
    }
    return false;
}

bool OI::GetIsHighGear() {
    return isHighGear;
}

bool OI::SwitchGripper(){
    if (xbox->GetBumperPressed(RIGHT_HAND)){
        isGripperClosed = false;
        frc::SmartDashboard::PutBoolean("isGripperClosed", false);
        return true;
    }
    if (xbox->GetBumperPressed(LEFT_HAND)){
        isGripperClosed = true;
        frc::SmartDashboard::PutBoolean("isGripperClosed", true);
        return true;
    }
    return false;
}

bool OI::GetIsGripperClosed() {
    return isGripperClosed;
}

double OI::GetArmInput() {
    armPowerOutput = 0.0;
    armPowerOutput = xbox->GetY(LEFT_HAND) * ARM_POWER_SCALE;
    return armPowerOutput;
}

double OI::GetWristInput() {
    wristPowerOutput = 0.0;
    wristPowerOutput = xbox->GetY(RIGHT_HAND) * WRIST_POWER_SCALE;
    return wristPowerOutput;
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

void OI::PrintToSmartDashboard(double encoderValue){
    frc::SmartDashboard::PutNumber("Arm Encoder Value: ", encoderValue);
}

bool OI::GetFourbarExtend(){
    return xbox->GetAButton();
}

bool OI::GetFourbarRetract(){
    return xbox->GetBButton();
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