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
    
}