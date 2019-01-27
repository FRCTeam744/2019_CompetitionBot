/*----------------------------------------------------------------------------------*/

//Use this to access and read encoder values as well as controller inputs

/*----------------------------------------------------------------------------------*/


//Inlcude all libraries that are necessary for reading input values
#include "OI.h"

#include <frc/Joystick.h>
#include <frc/XboxController.h>

//Wish you were here babe <3
//It's been 3 and a half years, still waiting for you
//you'll always be in our <3's

class OI {
    enum HAND {
        left,
        right
    };
    //Constants


    //Instance Variables
    frc::Joystick *rightStick;
    frc::Joystick *leftStick;
    frc::XboxController *xbox;

    //Public Methods
    double getDriveValue(HAND hand){
        if(hand == left){
            
        }
    }
    void OI::SelectRobotDrive(){
        if (xbox->GetAButtonPressed())
        {
            driveWithXbox = true;
            preferences->PutBoolean("drive with xbox", true);
            // table->PutNumber("camMode", 1.0);
        }
        if (xbox->GetBButtonPressed())
        {
            driveWithXbox = false;
            preferences->PutBoolean("drive with xbox", false);
            // table->PutNumber("camMode", 0.0);
        }
        if (xbox->GetXButtonPressed())
        {
            // arcadeDrive = true;
            // preferences->PutBoolean("arcade drive", true);
            table->PutNumber("ledMode", 1);
        }
        if (xbox->GetYButtonPressed())
        {
            // arcadeDrive = false;
            // preferences->PutBoolean("arcade drive", false);
            table->PutNumber("ledMode", 0);
        }
    }
    
    
}