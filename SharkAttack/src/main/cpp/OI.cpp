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

    isArmInManual = true;
    isWristInManual = true;
    isInCargoShipMode = false;
    // isArmInDefenseMode = false;

    //Caps the camera quality to allow for driver vision
    // camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
    // camera.SetResolution(resolutionWidth, resolutionHeight);
    // camera.SetFPS(framerate);

    armFFVoltage = preferences->GetDouble("armFFVoltage", 0.0);

    wasDPADRightPressed = false;
}

//Public Methods

bool OI::GetAutoDriveForward()
{
    return leftStick->GetRawButton(8);
}

bool OI::GetDriveByLimelight()
{
    return rightStick->GetRawButton(2);
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
    return leftStick->GetRawButton(2); //Was 11, changed by Robert
}

bool OI::SwitchGripper()
{
    if (isGripperClosed && xbox->GetBumper(RIGHT_HAND))
    {
        isGripperClosed = false;
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, "isGripperClosed", isGripperClosed);
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, "isGripperClosed_TEST", isGripperClosed);
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, "isGripperClosed_PRE", isGripperClosed);
        return true;
    }
    if (!isGripperClosed && !xbox->GetBumper(RIGHT_HAND))
    {
        isGripperClosed = true;
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, "isGripperClosed", isGripperClosed);
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->ArmWristTab, "isGripperClosed_TEST", isGripperClosed);
        // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, "isGripperClosed_PRE", isGripperClosed);
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
    if (armPowerOutput < ARM_DEADZONE && armPowerOutput > -ARM_DEADZONE)
    {
        armPowerOutput = 0.0;
    }
    else if (armPowerOutput >= ARM_DEADZONE)
    {
        armPowerOutput = 0.2 * ((armPowerOutput - ARM_DEADZONE) / (1.0 - ARM_DEADZONE)); //Scales power to 0.0-1.0
    }
    else if (armPowerOutput <= -ARM_DEADZONE)
    {
        armPowerOutput = 0.2 * ((armPowerOutput + ARM_DEADZONE) / (1.0 - ARM_DEADZONE)); //Scales power to 0.0-1.0
    }

    if (armPowerOutput != 0.0)
    {
        isArmInManual = true;
    }

    frc::SmartDashboard::PutNumber("Arm Control Output", armPowerOutput);
    return -armPowerOutput; //setting negative makes positive values move the arm forward
}

double OI::GetWristInput()
{
    wristPowerOutput = xbox->GetY(RIGHT_HAND);
    if (wristPowerOutput < WRIST_DEADZONE && wristPowerOutput > -WRIST_DEADZONE)
    {
        wristPowerOutput = 0.0;
    }
    else if (wristPowerOutput >= WRIST_DEADZONE)
    {
        wristPowerOutput = 0.2 * ((wristPowerOutput - WRIST_DEADZONE) / (1.0 - WRIST_DEADZONE)); //Scales power to 0.0-1.0
    }
    else if (armPowerOutput <= -WRIST_DEADZONE)
    {
        wristPowerOutput = 0.2 * ((wristPowerOutput + WRIST_DEADZONE) / (1.0 - WRIST_DEADZONE)); //Scales power to 0.0-1.0
    }

    if (wristPowerOutput != 0.0)
    {
        isWristInManual = true;
    }

    frc::SmartDashboard::PutNumber("Wrist Control Output", wristPowerOutput);
    return wristPowerOutput; 
}

double OI::GetIntakeInput()
{
    double intakeIn = xbox->GetTriggerAxis(LEFT_HAND);
    double intakeOut = xbox->GetTriggerAxis(RIGHT_HAND);
    if (intakeIn > 0.05)
    {
        return intakeIn;
    }
    else if (intakeOut > 0.05)
    {
        return -intakeOut;
    }
    else
    {
        return 0.0;
    }
}

bool OI::GetPlacingMode()
{
    if (xbox->GetStickButtonPressed(LEFT_HAND))
    {
        isInBallMode = false;
    }
    else if (xbox->GetStickButtonPressed(RIGHT_HAND))
    {
        isInBallMode = true;
    }
    return isInBallMode;
}

void OI::SetPlacingMode(bool isBallMode)
{
    isInBallMode = isBallMode;
}

bool OI::GetIsInBallPickup()
{
    return isInBallPickup;
}

bool OI::GetIsArmInManual()
{
    return isArmInManual;
}
bool OI::GetIsWristInManual()
{
    return isWristInManual;
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
    return rightStick->GetRawButtonPressed(16);
}

//Returns the Target Position to the Arm
double OI::GetTargetArmPosition()
{
    //Ball Placement and Pickup Presets
    if (isInBallMode)
    {
        if (xbox->GetYButtonPressed())
        {
            targetArmPosition = FRONT_HIGH_BALL_POSITION;
            isArmInManual = false;
            isWristInManual = false;
            isInBallPickup = false;
        }

        if (xbox->GetBButtonReleased())
        {
            targetArmPosition = FRONT_MID_BALL_POSITION;
            isArmInManual = false;
            isWristInManual = false;
            isInBallPickup = false;
        }

        if (xbox->GetBButton())
        {
            targetArmPosition = FRONT_CARGOSHIP_BALL_POSITION;
            isArmInManual = false;
            isWristInManual = false;
            isInBallPickup = false;
        }

        if (xbox->GetXButtonPressed())
        {
            targetArmPosition = FRONT_BALL_PICKUP_POSITION;
            isArmInManual = false;
            isWristInManual = false;
            isInBallPickup = true;
        }

        if (xbox->GetAButtonPressed())
        {
            targetArmPosition = FRONT_LOW_BALL_POSITION;
            isArmInManual = false;
            isWristInManual = false;
            isInBallPickup = false;
        }

        if (xbox->GetPOV(0) == DPAD_UP)
        {
            targetArmPosition = BACK_HIGH_BALL_POSITION;
            isArmInManual = false;
            isWristInManual = false;
            isInBallPickup = false;
        }

        if (xbox->GetPOV(0) == DPAD_LEFT)
        {
            targetArmPosition = BACK_BALL_PICKUP_POSITION;
            isArmInManual = false;
            isWristInManual = false;
            isInBallPickup = true;
        }
        

        //if DPAD RIGHT released
        if(xbox->GetPOV(0) != DPAD_RIGHT && wasDPADRightPressed) { 
            targetArmPosition = BACK_MID_BALL_POSITION;
            isArmInManual = false;
            isWristInManual = false;
            isInBallPickup = false;
        }
        if (xbox->GetPOV(0) == DPAD_RIGHT)
        {
            targetArmPosition = BACK_CARGOSHIP_BALL_POSITION;
            isArmInManual = false;
            isWristInManual = false;
            isInBallPickup = false;
        }
        wasDPADRightPressed = xbox->GetPOV(0) == DPAD_RIGHT;
        std::cout << "Was DPAD Right Pressed: " << wasDPADRightPressed << std::endl;

        if (xbox->GetPOV(0) == DPAD_DOWN)
        {
            targetArmPosition = BACK_LOW_BALL_POSITION;
            isArmInManual = false;
            isWristInManual = false;
            isInBallPickup = false;
        }

        if (xbox->GetBumperPressed(LEFT_HAND)){
            targetArmPosition = NEUTRAL_ARM_POSITION;
            isArmInManual = false;
            isWristInManual = false;
            isInBallPickup = false;
        }
    }
    else
    {

        isInBallPickup = false;

        if (xbox->GetYButtonPressed())
        {
            targetArmPosition = FRONT_HIGH_HATCH_POSITION;
            isArmInManual = false;
            isWristInManual = false;
        }

        if (xbox->GetBButtonPressed())
        {
            targetArmPosition = FRONT_MID_HATCH_POSITION;
            isArmInManual = false;
            isWristInManual = false;
        }

        if (xbox->GetAButtonPressed())
        {
            targetArmPosition = FRONT_LOW_HATCH_POSITION;
            isArmInManual = false;
            isWristInManual = false;
        }

        if (xbox->GetXButtonPressed())
        {
            targetArmPosition = NEUTRAL_ARM_POSITION;
            isArmInManual = false;
            isWristInManual = false;
        }

        if (xbox->GetPOV(0) == DPAD_UP)
        {
            targetArmPosition = BACK_HIGH_HATCH_POSITION;
            isArmInManual = false;
            isWristInManual = false;
        }

        if (xbox->GetPOV(0) == DPAD_RIGHT)
        {
            targetArmPosition = BACK_MID_HATCH_POSITION;
            isArmInManual = false;
            isWristInManual = false;
        }

        if (xbox->GetPOV(0) == DPAD_DOWN)
        {
            targetArmPosition = BACK_LOW_HATCH_POSITION;
            isArmInManual = false;
            isWristInManual = false;
        }

        if (xbox->GetPOV(0) == DPAD_LEFT)
        {
            targetArmPosition = NEUTRAL_ARM_POSITION;
            isArmInManual = false;
            isWristInManual = false;
        }

        if (xbox->GetBumperPressed(LEFT_HAND)){
            targetArmPosition = NEUTRAL_ARM_POSITION;
            isArmInManual = false;
            isWristInManual = false;
        }
    }
    frc::SmartDashboard::PutNumber("Target Arm Position Degrees", targetArmPosition);
    return targetArmPosition;
}

void OI::SetTargetArmPosition(double targetArmPos)
{
    targetArmPosition = targetArmPos;
}

bool OI::IsInCargoShipMode () {
    return xbox->GetBButton() || xbox->GetPOV(0) == DPAD_RIGHT;
}

double OI::GetTargetWristPosition()
{
    //Ball Placement and Pickup Presets
    if (isInBallMode)
    {
        if (xbox->GetYButtonPressed())
        {
            targetWristPosition = WRIST_BALL_PICKUP_FRONT_HIGH_DEG;
        }

        if (xbox->GetBButtonPressed())
        {
            targetWristPosition = WRIST_BALL_PICKUP_FRONT_MID_DEG;
        }

        if (xbox->GetXButtonPressed())
        {
            targetWristPosition = WRIST_NEUTRAL_DEG;
        }

        if (xbox->GetAButtonPressed())
        {
            targetWristPosition = WRIST_BALL_PICKUP_FRONT_CARGO_PRESET_DEG;
        }

        if (xbox->GetPOV(0) == DPAD_UP)
        {
            targetWristPosition = WRIST_BALL_PICKUP_BACK_HIGH_DEG;
        }

        if (xbox->GetPOV(0) == DPAD_LEFT)
        {
            targetWristPosition = WRIST_NEUTRAL_DEG;
        }

        if (xbox->GetPOV(0) == DPAD_RIGHT)
        {
            targetWristPosition = WRIST_BALL_PICKUP_BACK_MID_DEG;
        }

        if (xbox->GetPOV(0) == DPAD_DOWN)
        {
            targetWristPosition = WRIST_BALL_PICKUP_BACK_CARGO_PRESET_DEG;
        }
    }
    else
    {
        if (xbox->GetYButtonPressed())
        {
            targetWristPosition = WRIST_HATCH_PICKUP_FRONT_HIGH_DEG;
        }

        if (xbox->GetBButtonPressed())
        {
            targetWristPosition = WRIST_HATCH_PICKUP_FRONT_MID_DEG;
        }

        if (xbox->GetAButtonPressed())
        {
            targetWristPosition = WRIST_HATCH_PICKUP_FRONT_CARGO_PRESET_DEG;
        }

        if (xbox->GetXButtonPressed())
        {
            targetWristPosition = WRIST_NEUTRAL_DEG;
        }

        if (xbox->GetPOV(0) == DPAD_UP)
        {
            targetWristPosition = WRIST_HATCH_PICKUP_BACK_HIGH_DEG;
        }

        if (xbox->GetPOV(0) == DPAD_RIGHT)
        {
            targetWristPosition = WRIST_HATCH_PICKUP_BACK_MID_DEG;
        }

        if (xbox->GetPOV(0) == DPAD_DOWN)
        {
            targetWristPosition = WRIST_HATCH_PICKUP_BACK_CARGO_PRESET_DEG;
        }

        if (xbox->GetPOV(0) == DPAD_LEFT)
        {
            targetWristPosition = WRIST_NEUTRAL_DEG;
        }
    }
    frc::SmartDashboard::PutNumber("Target Wrist Position Degrees", targetWristPosition);
    return targetWristPosition;
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
        return std::make_tuple(true, "pipeline", 0.0);
    }
    else if (leftStick->GetRawButtonPressed(9))
    {
        return std::make_tuple(true, "pipeline", 1.0);
    }

    return std::make_tuple(false, "", 0.0);
}

bool OI::LEDButtonPressed()
{
    return leftStick->GetRawButton(3);
}

bool OI::AlsoLEDButtonPressed()
{
    return rightStick->GetRawButton(3);
}

bool OI::GetStopLLMove() {
    return leftStick->GetRawButton(12);
}

double OI::GetArmFFVoltage()
{

    double newArmFFVoltage = preferences->GetDouble("armFFVoltage", 0.0);

    if (armFFVoltage != newArmFFVoltage)
    {
        armFFVoltage = newArmFFVoltage;
    }

    return armFFVoltage;
}

bool OI::GetIsArmInDefenseMode(){
    // std::cout << "Button 7 working" << std::endl;
    if (leftStick->GetRawButton(9)){
        isArmInDefenseMode = false;
    }
    else if (leftStick->GetRawButton(10)){
        isArmInDefenseMode = true;
    }
    frc::SmartDashboard::PutBoolean("isArmInDefenseMode", isArmInDefenseMode);
    return isArmInDefenseMode;
}

void OI::SetArmWristInManual(bool isArmManual, bool isWristManual){
    isArmInManual = isArmManual;
    isWristInManual = isWristManual;
}