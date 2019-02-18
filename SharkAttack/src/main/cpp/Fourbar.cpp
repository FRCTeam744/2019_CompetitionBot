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
    fourbarExtender = new rev::CANSparkMax(50, rev::CANSparkMax::MotorType::kBrushless);
    fourbarSpeed = frc::SmartDashboard::GetNumber("fourbarSpeed", 0.1);

    fourbarEncoder = new rev::CANEncoder(*fourbarExtender);

    extended = new frc::DigitalInput(0);
    retracted = new frc::DigitalInput(1);

    isPastRetracted = true;
    isPastExtended = false;
    isHoming = false;

    rotationsSinceRetractTripped = 0.0;
}

void Fourbar::ExtendOrRetract(bool extendBut, bool retractBut){

    if (extendBut && !retractBut) {
        isHoming = false;
        if(!IsExtendedTripped() && !isPastExtended){
            fourbarExtender->Set(fourbarSpeed);
            isPastRetracted = false;
        } 
        else if (IsExtendedTripped() && !isPastExtended){
            fourbarExtender->Set(0.0);
            isPastExtended = true;
        }
        else {
            fourbarExtender->Set(0.0);
        }
    }
    else if(retractBut && !extendBut){
        isHoming = false;
        if(!IsRetractedTripped() && !isPastRetracted){
            fourbarExtender->Set(-fourbarSpeed); 
            isPastExtended = false;
        }
        else if (IsRetractedTripped() && !isPastRetracted){
            fourbarExtender->Set(0.0);
            isPastRetracted = true;
        }
        else{
            fourbarExtender->Set(0.0);
        }
    }
    else if (isHoming){}
    else {
        fourbarExtender->Set(0.0);
    }
    
}

void Fourbar::FourbarHome(bool homingBut){
    if ((homingBut || isHoming) && !IsRetractedTripped() && !isPastRetracted){
        isHoming = true;
        fourbarExtender->Set(-0.1);
    }
    else if (IsRetractedTripped() && !isPastRetracted && isHoming) {
        isPastRetracted = true;
        rotationsAtRetractTripped = fourbarEncoder->GetPosition();
        fourbarExtender->Set(-0.02);
    }
    else if (-rotationsSinceRetractTripped < ROTATIONS_TO_HOME && isHoming){
        rotationsSinceRetractTripped = fourbarEncoder->GetPosition() - rotationsAtRetractTripped;
        fourbarExtender->Set(-0.02);
    }
    else {
        isHoming = false;
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

    frc::SmartDashboard::PutNumber("Fourbar Encoder Position", fourbarEncoder->GetPosition());
    frc::SmartDashboard::PutNumber("Fourbar Rotations To Home", ROTATIONS_TO_HOME);
    frc::SmartDashboard::PutNumber("Fourbar Current", fourbarExtender->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Fourbar RPM", fourbarEncoder->GetVelocity());
}

//Private Methods
bool Fourbar::IsExtendedTripped(){
    return !retracted->Get();
}

bool Fourbar::IsRetractedTripped(){
    return !extended->Get();
}