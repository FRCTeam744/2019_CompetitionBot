/*----------------------------------------------------------------------------------*/

//Methods for the LED class (any and all things LEDs/arduino)

/*----------------------------------------------------------------------------------*/

#include "LED.h"

LED *LED::s_instance = 0;

//Static Singleton Method
LED *LED::GetInstance()
{
    if (s_instance == 0)
    {
        s_instance = new LED();
    }
    return s_instance;
}

//Constructor
LED::LED()
{

    byte1 = new frc::DigitalOutput(6);
    byte2 = new frc::DigitalOutput(7);
    byte3 = new frc::DigitalOutput(8);
    byte4 = new frc::DigitalOutput(9);
    
    // try
    // {
    //     arduino = new frc::SerialPort(9600, frc::SerialPort::kUSB2);
    //     arduino->SetTimeout(0.01);
    // }
    // catch (const std::exception &e)
    // {
    //     frc::SmartDashboard::PutString("LED Data", "Error: Could not connect to arduino");
    // }
}

//Public Methods
void LED::LEDsOff()
{
    byte1->Set(true);
    byte2->Set(false);
    byte3->Set(false);
    byte4->Set(false);

    // if (arduino != NULL)
    // {
    //     arduino->Write("A", 1);
    // }
}

void LED::SwimmingShark()
{

    byte1->Set(false);
    byte2->Set(false);
    byte3->Set(false);
    byte4->Set(false);

    // if (arduino != NULL)
    // {
    //     arduino->Write("B", 1);
    // }
}

void LED::StartUp()
{

    byte1->Set(false);
    byte2->Set(true);
    byte3->Set(false);
    byte4->Set(false);

    // if (arduino != NULL)
    // {
    //     arduino->Write("C", 1);
    // }
}

void LED::StartUpBlue()
{
    byte1->Set(false);
    byte2->Set(false);
    byte3->Set(false);
    byte4->Set(false);

    // if (arduino != NULL)
    // {
    //     arduino->Write("D", 1);
    // }
}

void LED::StartUpRed()
{
    byte1->Set(false);
    byte2->Set(false);
    byte3->Set(false);
    byte4->Set(false);

    // if (arduino != NULL)
    // {
    //     arduino->Write("E", 1);
    // }
}

void LED::LiftOffBlue()
{
    byte1->Set(false);
    byte2->Set(false);
    byte3->Set(false);
    byte4->Set(false);
    // if (arduino != NULL)
    // {
    //     arduino->Write("M", 1);
    // }
}

void LED::LiftOffRed()
{
    byte1->Set(false);
    byte2->Set(false);
    byte3->Set(false);
    byte4->Set(false);

    // if (arduino != NULL)
    // {
    //     arduino->Write("N", 1);
    // }
}

void LED::ShutDown() {

    byte1->Set(true);
    byte2->Set(true);
    byte3->Set(false);
    byte4->Set(false);

    // if (arduino != NULL){
    //     arduino->Write("S", 1);
    // }
}

void LED::HatchOrBallMode(bool isInBallMode, bool isVisionTracking){
    
    if(isVisionTracking) {
        LEDsOff();
    }
    else if (!isInBallMode){
        byte1->Set(false);
        byte2->Set(false);
        byte3->Set(false);
        byte4->Set(false);
        // arduino->Write("W", 1);
    }
    else {
        byte1->Set(false);
        byte2->Set(false);
        byte3->Set(false);
        byte4->Set(false);
        // arduino->Write("X", 1);
    }
}

void LED::IsHatchOpen(bool isHatchGripperOpen, bool isVisionTracking){
    if(isVisionTracking){
        LEDsOff();
    }
    else if(!isHatchGripperOpen){
        byte1->Set(false);
        byte2->Set(false);
        byte3->Set(true);
        byte4->Set(false);
        // arduino->Write("G", 1);
    }
    else {
        byte1->Set(true);
        byte2->Set(false);
        byte3->Set(true);
        byte4->Set(false);
        // arduino->Write("R", 1);
    }
}