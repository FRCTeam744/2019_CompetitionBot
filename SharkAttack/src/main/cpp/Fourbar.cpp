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
    //initialize NEO motor and current limit
    fourbarExtender = new rev::CANSparkMax(50, rev::CANSparkMax::MotorType::kBrushless);
    fourbarExtender->SetSmartCurrentLimit(60);

    //initialize testing speed, no longer used
    fourbarSpeed = frc::SmartDashboard::GetNumber("fourbarSpeed", 0.1);
    
    //initialize encoder built into NEO motor
    fourbarEncoder = new rev::CANEncoder(*fourbarExtender);

    //initialize magnetic read sensors
    extended = new frc::DigitalInput(1);
    retracted = new frc::DigitalInput(0);

    //initialize state tracking variables
    isPastRetracted = true;
    isPastExtended = false;
    isHoming = false;

    rotationsSinceRetractTripped = 0.0;
}

void Fourbar::ExtendOrRetract(bool extendBut, bool retractBut){

    //update state variables
    if (IsExtendedTripped()){
        isPastRetracted = false;
    }
    if (IsRetractedTripped()) {
        isPastExtended = false;
    }

    //if wanting to extend,  otherwise
    if (extendBut && !retractBut) {
        isHoming = false;
        //while not past extended, extend
        if(!IsExtendedTripped() && !isPastExtended){
            fourbarExtender->Set(EXTEND_SPEED);
            isPastRetracted = false;
        } 
        //is past extended, stop and set isPastExtended
        else if (IsExtendedTripped() && !isPastExtended){
            fourbarExtender->Set(0.0);
            isPastExtended = true;
        }
        //otherwise, stop (unlikely you ever get here)
        else {
            fourbarExtender->Set(0.0);
        }
    }
    //if wanting to retract
    else if(retractBut && !extendBut){
        isHoming = false;
        //while not past retracted limit switch, retract
        if(!IsRetractedTripped() && !isPastRetracted){
            fourbarExtender->Set(RETRACT_SPEED); 
            isPastExtended = false;
        }
        //if past retracted limit switch, stop retracting
        else if (IsRetractedTripped() && !isPastRetracted){
            fourbarExtender->Set(0.0);
            isPastRetracted = true;
        }
        //otherwise, stop (unlikely you ever get here)
        else{
            fourbarExtender->Set(0.0);
        }
    }
    //otherwise, do nothing
    else if (isHoming){}
    else {
        fourbarExtender->Set(0.0);
    }
    
}

void Fourbar::FourbarHome(bool homingBut){
    //if homing is wanted, and you haven't tripped nor past retracted, retract at pre limit switch speed
    if ((homingBut || isHoming) && !IsRetractedTripped() && !isPastRetracted){
        isHoming = true;
        fourbarExtender->Set(HOMING_PRE_LIMIT_SWITCH_SPEED);
    }
    //if at the rising edge of the limit switch, note encoder position, and change retracting speed to be slower 
    else if (IsRetractedTripped() && !isPastRetracted && isHoming) {
        isPastRetracted = true;
        rotationsAtRetractTripped = fourbarEncoder->GetPosition();
        fourbarExtender->Set(HOMING_POST_LIMIT_SWITCH_SPEED);
    }
    //while not fully homed, retracted until the desired millimeters is reached 
    else if (-rotationsSinceRetractTripped < ROTATIONS_TO_HOME && isHoming){
        rotationsSinceRetractTripped = fourbarEncoder->GetPosition() - rotationsAtRetractTripped;
        fourbarExtender->Set(HOMING_POST_LIMIT_SWITCH_SPEED);
        frc::SmartDashboard::PutNumber("MM Since Retract Tripped", rotationsSinceRetractTripped * MM_PER_REVOLUTIONS);
    }
    //when desired millimeters is reached, set isHoming to false, thus stopping the motor
    else {
        isHoming = false;
    }
}

void Fourbar::UpdateFourbarSpeed() {
    newSpeed = frc::SmartDashboard::GetNumber("fourbarSpeed", 0.1);
    if (fourbarSpeed != newSpeed){
        fourbarSpeed = newSpeed;
    }
}

void Fourbar::PrintFourbarShuffleInfo(){
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->checkExtendedTrippedDriver, IsExtendedTripped());
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->checkRetractedTrippedDriver, IsRetractedTripped());
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->DriverTab, ShuffleManager::GetInstance()->fourbarEncoderDriver, fourbarEncoder->GetPosition());
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->FourbarTab, ShuffleManager::GetInstance()->fourbarEncoderFourbar, fourbarEncoder->GetPosition());
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->FourbarTab, ShuffleManager::GetInstance()->fourbarRPMFourbar, fourbarEncoder->GetVelocity());
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->FourbarTab, ShuffleManager::GetInstance()->fourbarCurrentFourbar, fourbarExtender->GetOutputCurrent());
    ShuffleManager::GetInstance()->OnShfl(ShuffleManager::GetInstance()->FourbarTab, ShuffleManager::GetInstance()->fourbarRotationsToHomeFourbar, ROTATIONS_TO_HOME);
}

//Private Methods
bool Fourbar::IsExtendedTripped(){
    return !retracted->Get();
}

bool Fourbar::IsRetractedTripped(){
    return !extended->Get();
}