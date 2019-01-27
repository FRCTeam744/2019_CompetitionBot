/*----------------------------------------------------------------------------------*/

//Use this to access and read encoder values as well as controller inputs

/*----------------------------------------------------------------------------------*/


//Inlcude all libraries that are necessary for reading input values
#include <frc/Joystick.h>
#include <frc/XboxController.h>


class OI {
    enum HAND {
        left,
        right
    };
    //Constants
    const frc::XboxController::JoystickHand leftHand = frc::XboxController::kLeftHand;
    const frc::XboxController::JoystickHand rightHand = frc::XboxController::kRightHand;

    //Instance Variables
    frc::Joystick *rightStick;
    frc::Joystick *leftStick;
    frc::XboxController *xbox;

    //Public Methods
    public : double getDriveValue(HAND hand){
        if(hand == left){

        }
    }

    
}