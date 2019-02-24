/*----------------------------------------------------------------------------------*/

//Methods for the OI class (any and all things driver station/operator interface)

/*----------------------------------------------------------------------------------*/

#include "OI.h"

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

bool OI::GetAutoDriveForward()
{
    return leftStick->GetRawButton(8);
}

bool OI::SwitchGears()
{
    if (leftStick->GetRawButtonPressed(1))
    {
        isHighGear = false;
        //frc::SmartDashboard::PutBoolean("isHighGear", false);
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, "isHighGear", isHighGear);
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, "isHighGear", isHighGear);
        return true;
    }
    if (rightStick->GetRawButtonPressed(1))
    {
        isHighGear = true;
        //frc::SmartDashboard::PutBoolean("isHighGear", true);
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, "isHighGear", isHighGear);
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, "isHighGear", isHighGear);
        return true;
    }
    return false;
}

bool OI::GetIsHighGear()
{
    return isHighGear;
}

bool OI::GetVelocityTest()
{
    return leftStick->GetRawButton(11);
}

bool OI::SwitchGripper()
{
    if (xbox->GetBumperPressed(RIGHT_HAND))
    {
        isGripperClosed = false;
        //frc::SmartDashboard::PutBoolean("isGripperClosed", false);
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, "isGripperClosed", isGripperClosed);
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, "isGripperClosed", isGripperClosed);
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, "isGripperClosed", isGripperClosed);
        return true;
    }
    if (xbox->GetBumperPressed(LEFT_HAND))
    {
        isGripperClosed = true;
        //frc::SmartDashboard::PutBoolean("isGripperClosed", true);
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, "isGripperClosed", isGripperClosed);
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, "isGripperClosed", isGripperClosed);
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, "isGripperClosed", isGripperClosed);
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
    if (armPowerOutput < 0.07 && armPowerOutput > -0.07)
    {
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

double OI::GetIntakeIn()
{
    //ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->FourbarTab, "Left Trigger Output", xbox->GetTriggerAxis(LEFT_HAND));
    if (xbox->GetTriggerAxis(LEFT_HAND) > 0.05)
    {
        return xbox->GetTriggerAxis(LEFT_HAND);
    }
    return 0.0;
}

double OI::GetIntakeOut()
{
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->FourbarTab, "Right Trigger Output", xbox->GetTriggerAxis(RIGHT_HAND));
    if (xbox->GetTriggerAxis(RIGHT_HAND) > 0.05)
    {
        return xbox->GetTriggerAxis(RIGHT_HAND);
    }
    return 0.0;
}

// //positive degree values
// double OI::ArmPresetPickupCargo()
// {
//     return ARM_PICKUP_CARGO_PRESET_DEG;
// }
// double OI::ArmPresetLow()
// {
//     return ARM_PICKUP_LOW_DEG;
// }
// double OI::ArmPresetMid()
// {
//     return ARM_PICKUP_MID_DEG;
// }
// double OI::ArmPresetHigh()
// {
//     return ARM_PICKUP_HIGH_DEG;
// }

// //negative degree
// double OI::ArmPresetNegPickupCargo()
// {
//     return ARM_PICKUP_NEG_CARGO_PRESET_DEG;
// }
// double OI::ArmPresetNegLow()
// {
//     return ARM_PICKUP_NEG_LOW_DEG;
// }
// double OI::ArmPresetNegMid()
// {
//     return ARM_PICKUP_NEG_MID_DEG;
// }
// double OI::ArmPresetNegHigh()
// {
//     return ARM_PICKUP_NEG_HIGH_DEG;
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

bool OI::GetFourbarHome()
{
    return rightStick->GetRawButtonPressed(8);
}

double OI::GetTargetPosition()
{
    if (xbox->GetYButtonPressed())
    {
        targetArmPosition = ARM_PICKUP_HIGH_DEG;
    }

    if (xbox->GetBButtonPressed())
    {
        targetArmPosition = ARM_PICKUP_MID_DEG;
    }

    if (xbox->GetXButtonPressed())
    {
        targetArmPosition = ARM_PICKUP_LOW_DEG;
    }

    if (xbox->GetAButtonPressed())
    {
        targetArmPosition = ARM_PICKUP_CARGO_PRESET_DEG;
    }

    if (xbox->GetPOV(0) == 0)
    {
        targetArmPosition = ARM_PICKUP_NEG_HIGH_DEG;
    }

    if (xbox->GetPOV(0) == 270)
    {
        targetArmPosition = ARM_PICKUP_NEG_MID_DEG;
    }

    if (xbox->GetPOV(0) == 90)
    {
        targetArmPosition = ARM_PICKUP_NEG_LOW_DEG;
    }

    if (xbox->GetPOV(0) == 180)
    {
        targetArmPosition = ARM_PICKUP_NEG_CARGO_PRESET_DEG;
    }
    
    return targetArmPosition;
}
// bool OI::SetArmFrontHigh()
// {
//     return xbox->GetYButtonPressed(); //ARM_PICKUP_HIGH_DEG
// }

// bool OI::SetArmFrontMid()
// {
//     return xbox->GetBButtonPressed(); //ARM_PICKUP_MID_DEG
// }

// bool OI::SetArmFrontLow()
// {
//     return xbox->GetXButtonPressed(); //ARM_PICKUP_LOW_DEG
// }

// bool OI::SetArmFrontBallPickup()
// {
//     return xbox->GetAButtonPressed(); //ARM_PICKUP_CARGO_PRESET_DEG
// }

// bool OI::SetArmBackHigh()
// {
//     return (xbox->GetPOV(0) == 0); //ARM_PICKUP_NEG_HIGH_DEG
// }

// bool OI::SetArmBackMid()
// {
//     return (xbox->GetPOV(0) == 270); //ARM_PICKUP_NEG_MID_DEG
// }

// bool OI::SetArmBackLow()
// {
//     return (xbox->GetPOV(0) == 90); //ARM_PICKUP_NEG_LOW_DEG
// }

// bool OI::SetArmBackBallPickup()
// {
//     return (xbox->GetPOV(0) == 180); //ARM_PICKUP_NEG_CARGO_PRESET_DEG
// }

std::tuple<bool, std::string, double> OI::SetLimelight()
{
    if (leftStick->GetRawButtonPressed(5))
    {
        return std::make_tuple(true, "ledMode", 0.0);
    }
    else if (leftStick->GetRawButtonPressed(10))
    {
        return std::make_tuple(true, "ledMode", 1.0);
    }
    else if (leftStick->GetRawButtonPressed(6))
    {
        return std::make_tuple(true, "camMode", 0.0);
    }
    else if (leftStick->GetRawButtonPressed(9))
    {
        return std::make_tuple(true, "camMode", 1.0);
    }

    return std::make_tuple(false, "", 0.0);
}

bool OI::LEDButtonPressed()
{
    return leftStick->GetRawButton(2);
}

bool OI::AlsoLEDButtonPressed()
{
    return rightStick->GetRawButton(2);
}