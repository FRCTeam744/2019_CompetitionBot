#include "Fourbar.h"


Fourbar* Fourbar::s_instance = 0;

//Static Singleton Method
Fourbar* Fourbar::GetInstance() {
  if (s_instance == 0){
    s_instance = new Fourbar();
  }
  return s_instance;
}

Fourbar::Fourbar()
{
    fourbarExtender = new rev::CANSparkMax(36, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    fourbarSpeed = 0.1;
}

void Fourbar::ExtendBar(bool inputBut) {
    if(inputBut == true){
        fourbarExtender->Set(fourbarSpeed);
    } else {
        fourbarExtender->Set(0.0);
    }
    
}

void Fourbar::RetractBar(bool inputBut) {
    if(inputBut == true){
        fourbarExtender->Set(-fourbarSpeed);
    } else {
        fourbarExtender->Set(0.0);
    }
}

void Fourbar::UpdateFourbarSpeed(double newSpeed) {
    
    if (fourbarSpeed != newSpeed){
        fourbarSpeed = newSpeed;
    }
}