/*----------------------------------------------------------------------------------*/

//Methods for the LED class (any and all things LEDs/arduino)

/*----------------------------------------------------------------------------------*/

#include "LED.h"

LED* LED::s_instance = 0;

//Static Singleton Method
LED* LED::GetInstance() {
    if (s_instance == 0) {
        s_instance = new LED();
    }
    return s_instance;
}

//Constructor
LED::LED() {
    try {
        arduino = new frc::SerialPort(9600, frc::SerialPort::kUSB);
        arduino->SetTimeout(0.001);

    }
    catch(const std::exception& e) {
        frc::SmartDashboard::PutString("LED Data", "Error: Could not connect to arduino");
    }

}

//Public Methods
void LED::LEDsOff() {
    arduino->Write("A", 1);
}

void LED::SwimmingShark() {
    arduino->Write("B", 1);
}

void LED::StartUp() {
    arduino->Write("C", 1);
}

void LED::StartUpBlue(){
    arduino->Write("D", 1);
}

void LED::StartUpRed(){
    arduino->Write("E", 1);
}

void LED::LiftOffBlue() {
    arduino->Write("M", 1);
}

void LED::LiftOffRed() {
    arduino->Write("N", 1);
}

void LED::ShutDown() {
    arduino->Write("S", 1);
}