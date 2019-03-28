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
    try
    {
        arduino = new frc::SerialPort(9600, frc::SerialPort::kUSB2);
        arduino->SetTimeout(0.01);
    }
    catch (const std::exception &e)
    {
        frc::SmartDashboard::PutString("LED Data", "Error: Could not connect to arduino");
    }
}

//Public Methods
void LED::LEDsOff()
{
    if (arduino != NULL)
    {
        arduino->Write("A", 1);
    }
}

void LED::SwimmingShark()
{
    if (arduino != NULL)
    {
        arduino->Write("B", 1);
    }
}

void LED::StartUp()
{
    if (arduino != NULL)
    {
        arduino->Write("C", 1);
    }
}

void LED::StartUpBlue()
{
    if (arduino != NULL)
    {
        arduino->Write("D", 1);
    }
}

void LED::StartUpRed()
{
    if (arduino != NULL)
    {
        arduino->Write("E", 1);
    }
}

void LED::LiftOffBlue()
{
    if (arduino != NULL)
    {
        arduino->Write("M", 1);
    }
}

void LED::LiftOffRed()
{
    if (arduino != NULL)
    {
        arduino->Write("N", 1);
    }
}

void LED::ShutDown() {

    if (arduino != NULL){
        arduino->Write("S", 1);
    }
}

void LED::HatchOrBallMode(bool isInBallMode, bool isVisionTracking){
    if(isVisionTracking) {
        LEDsOff();
    }
    else if (!isInBallMode && arduino != NULL){
        arduino->Write("W", 1);
    }
    else if (arduino != NULL){
        arduino->Write("X", 1);
    }
}

void LED::IsHatchOpen(bool isHatchGripperOpen, bool isVisionTracking){
    if(isVisionTracking){
        LEDsOff();
    }
    else if(!isHatchGripperOpen){
        arduino->Write("G", 1);
    }
    else {
        arduino->Write("R", 1);
    }
}