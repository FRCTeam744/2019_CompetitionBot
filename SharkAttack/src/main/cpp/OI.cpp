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
    isInBallMode = false;
    targetArmPosition = NEUTRAL_ARM_POSITION;

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
        ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, "isGripperClosed", isGripperClosed);
        ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, "isGripperClosed_TEST", isGripperClosed);
        ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, "isGripperClosed_PRE", isGripperClosed);
        return true;
    }
    if (xbox->GetBumperPressed(LEFT_HAND))
    {
        isGripperClosed = true;
        //frc::SmartDashboard::PutBoolean("isGripperClosed", true);
        ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, "isGripperClosed", isGripperClosed);
        ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, "isGripperClosed_TEST", isGripperClosed);
        ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, "isGripperClosed_PRE", isGripperClosed);
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
    wristPowerOutput = xbox->GetY(RIGHT_HAND);
    return wristPowerOutput;
}

double OI::GetIntakeIn()
{
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->FourbarTab, "Left Trigger Output", xbox->GetTriggerAxis(LEFT_HAND));
    if (xbox->GetTriggerAxis(LEFT_HAND) > 0.05)
    {
        return xbox->GetTriggerAxis(LEFT_HAND);
    }
    return 0.0;
}

double OI::GetIntakeOut()
{
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->FourbarTab, "Right Trigger Output", xbox->GetTriggerAxis(RIGHT_HAND));
    if (xbox->GetTriggerAxis(RIGHT_HAND) > 0.05)
    {
        return xbox->GetTriggerAxis(RIGHT_HAND);
    }
    return 0.0;
}

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


//Returns the Target Position to the Arm
double OI::GetTargetPosition() 
{
    //Ball Placement and Pickup Presets
    if (isInBallMode){
        if (xbox->GetYButtonPressed())
        {
            targetArmPosition = FRONT_HIGH_BALL_POSITION;
        }

        if (xbox->GetBButtonPressed())
        {
            targetArmPosition = FRONT_MID_BALL_POSITION;
        }

        if (xbox->GetXButtonPressed())
        {
            targetArmPosition = FRONT_LOW_BALL_POSITION;
        }

        if (xbox->GetAButtonPressed())
        {
            targetArmPosition = FRONT_BALL_PICKUP_POSITION;
        }

        if (xbox->GetPOV(0) == DPAD_UP)
        {
            targetArmPosition = BACK_HIGH_BALL_POSITION;
        }

        if (xbox->GetPOV(0) == DPAD_LEFT)
        {
            targetArmPosition = BACK_MID_BALL_POSITION;
        }

        if (xbox->GetPOV(0) == DPAD_RIGHT)
        {
            targetArmPosition = BACK_LOW_BALL_POSITION;
        }

        if (xbox->GetPOV(0) == DPAD_DOWN)
        {
            targetArmPosition = BACK_BALL_PICKUP_POSITION;
        }
    }
    else {
        if (xbox->GetYButtonPressed())
        {
            targetArmPosition = FRONT_HIGH_HATCH_POSITION;
        
        }

        if (xbox->GetBButtonPressed())
        {
            targetArmPosition = FRONT_MID_HATCH_POSITION;
        }

        if (xbox->GetXButtonPressed())
        {
            targetArmPosition = FRONT_LOW_HATCH_POSITION;
        }

        if (xbox->GetAButtonPressed())
        {
            targetArmPosition = FRONT_LOW_HATCH_POSITION;
        }

        if (xbox->GetPOV(0) == DPAD_UP)
        {
            targetArmPosition = BACK_HIGH_HATCH_POSITION;
        }

        if (xbox->GetPOV(0) == DPAD_LEFT)
        {
            targetArmPosition = BACK_MID_HATCH_POSITION;
        }

        if (xbox->GetPOV(0) == DPAD_RIGHT)
        {
            targetArmPosition = BACK_LOW_HATCH_POSITION;
        }

        if (xbox->GetPOV(0) == DPAD_DOWN)
        {
            targetArmPosition = BACK_LOW_HATCH_POSITION;
        }
    }
    frc::SmartDashboard::PutNumber("Target Arm Position Degrees", targetArmPosition);
    return targetArmPosition;
}

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