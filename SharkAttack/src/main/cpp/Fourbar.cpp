/*----------------------------------------------------------------------------------*/

//Methods for the Fourbar class (any and all things fourbar/climber)

/*----------------------------------------------------------------------------------*/

#include "Fourbar.h"
#include "ShuffleManager.h"

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

    extended = new frc::DigitalInput(1);
    retracted = new frc::DigitalInput(0);

    isPastRetracted = true;
    isPastExtended = false;
    isHoming = false;

    rotationsSinceRetractTripped = 0.0;
}

void Fourbar::ExtendOrRetract(bool extendBut, bool retractBut){

    if (IsExtendedTripped()){
        isPastRetracted = false;
    }
    if (IsRetractedTripped()) {
        isPastExtended = false;
    }

    if (extendBut && !retractBut) {
        isHoming = false;
        if(!IsExtendedTripped() && !isPastExtended){
            fourbarExtender->Set(1.0);
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
            fourbarExtender->Set(-1.0); 
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
        fourbarExtender->Set(-0.2);
    }
    else if (IsRetractedTripped() && !isPastRetracted && isHoming) {
        isPastRetracted = true;
        rotationsAtRetractTripped = fourbarEncoder->GetPosition();
        fourbarExtender->Set(-0.05);
    }
    else if (-rotationsSinceRetractTripped < ROTATIONS_TO_HOME && isHoming){
        rotationsSinceRetractTripped = fourbarEncoder->GetPosition() - rotationsAtRetractTripped;
        fourbarExtender->Set(-0.05);
        frc::SmartDashboard::PutNumber("MM Since Retract Tripped", rotationsSinceRetractTripped * MM_PER_REVOLUTIONS);
    }
    else {
        isHoming = false;
    }
}

void Fourbar::UpdateFourbarSpeed() {
    
    // frc::SmartDashboard::PutBoolean("extended switch", IsExtendedTripped());
    // frc::SmartDashboard::PutBoolean("retracted switch", IsRetractedTripped());
//    if(isInitialized == false){
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->FourbarTab, "Extended Switch", IsExtendedTripped());
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->FourbarTab, "Retracted Switch", IsRetractedTripped());
  //      isInitialized = true;
  //  }

    newSpeed = frc::SmartDashboard::GetNumber("fourbarSpeed", 0.1);
    if (fourbarSpeed != newSpeed){
        fourbarSpeed = newSpeed;
    }
}

void Fourbar::PrintClimberRPM(){
//    if(isInitialized == false){
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->FourbarTab, "Fourbar Rotations Counted", rotationsSinceRetractTripped);
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->FourbarTab, "Fourbar Encoder Position", fourbarEncoder->GetPosition());
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->FourbarTab, "Fourbar Rotations To Home", ROTATIONS_TO_HOME);
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->FourbarTab, "Fourbar Current", fourbarExtender->GetOutputCurrent());
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, "Fourbar Current_PRE", fourbarExtender->GetOutputCurrent());
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->FourbarTab, "Fourbar RPM", fourbarEncoder->GetVelocity());
    // ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->PreCompTab, "Fourbar RPM_PRE", fourbarEncoder->GetVelocity());
//        isInitialized = true;
//    }
    // frc::SmartDashboard::PutNumber("Fourbar Rotations Counted", rotationsSinceRetractTripped);
    // frc::SmartDashboard::PutNumber("Fourbar Encoder Position", fourbarEncoder->GetPosition());
    // frc::SmartDashboard::PutNumber("Fourbar Rotations To Home", ROTATIONS_TO_HOME);
    // frc::SmartDashboard::PutNumber("Fourbar Current", fourbarExtender->GetOutputCurrent());
    // frc::SmartDashboard::PutNumber("Fourbar RPM", fourbarEncoder->GetVelocity());
}

//Private Methods
bool Fourbar::IsExtendedTripped(){
    return !retracted->Get();
}

bool Fourbar::IsRetractedTripped(){
    return !extended->Get();
}