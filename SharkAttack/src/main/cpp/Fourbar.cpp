/*----------------------------------------------------------------------------------*/

//Methods for the Fourbar class (any and all things fourbar/climber)

/*----------------------------------------------------------------------------------*/

#include "Fourbar.h"


Fourbar* Fourbar::s_instance = 0;

//Static Singleton Method
Fourbar* Fourbar::GetInstance() {
  if (s_instance == 0){
    s_instance = new Fourbar();
  }
  return s_instance;
}

Fourbar::Fourbar() {
    fourbarExtender = new rev::CANSparkMax(50, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    fourbarSpeed = frc::SmartDashboard::GetNumber("fourbarSpeed", 0.1);

    fourbarEncoder = new rev::CANEncoder(*fourbarExtender);

    extended = new frc::DigitalInput(0);
    retracted = new frc::DigitalInput(1);

    retractedTripped = true;
    extendedTripped = false;
}

void Fourbar::ExtendOrRetract(bool extendBut, bool retractBut){

    if (extendBut && !retractBut) {
        if(extended->Get()){
            fourbarExtender->Set(fourbarSpeed);
            retractedTripped = false;
        } 
        // else if (!extended->Get()){
        //     fourbarExtender->Set(0.0);
        //     extendedTripped = true;
        //     retractedTripped = false;
        // }
        else {
            fourbarExtender->Set(0.0);
        }
    }
    else if(retractBut && !extendBut){
        if(retracted->Get()){
            fourbarExtender->Set(fourbarSpeed); 
            extendedTripped = false;
        }
        // else if (!retracted->Get()){
        //     // fourbarExtender->Set(-fourbarSpeed*0.01);
        //     retractedTripped = true;
        // }
        // else if(retracted->Get() && retractedTripped){
        //     fourbarExtender->Set(0.0);
        // }
        else{
            fourbarExtender->Set(0.0);
        }
    }
    else {
        fourbarExtender->Set(0.0);
    }
    
}

void Fourbar::UpdateFourbarSpeed() {
    
    frc::SmartDashboard::PutBoolean("extended switch", !extended->Get());
    frc::SmartDashboard::PutBoolean("retracted switch", !retracted->Get());

    newSpeed = frc::SmartDashboard::GetNumber("fourbarSpeed", 0.1);
    if (fourbarSpeed != newSpeed){
        fourbarSpeed = newSpeed;
    }
}

void Fourbar::PrintClimberRPM(){

    frc::SmartDashboard::PutNumber("Fourbar Current", fourbarExtender->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Fourbar RPM", fourbarEncoder->GetVelocity());
}