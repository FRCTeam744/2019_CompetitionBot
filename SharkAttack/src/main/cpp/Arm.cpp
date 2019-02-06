/*----------------------------------------------------------------------------------*/

//Methods for the Arm class (any and all things arm/wrist/placing)

/*----------------------------------------------------------------------------------*/

#include "Arm.h"

Arm* Arm::s_instance = 0;

//Static Singleton Method
Arm* Arm::GetInstance() {  
  if (s_instance == 0){
    s_instance = new Arm();
  }
  return s_instance;  
}

//Constructor
Arm::Arm() {
  //Initialize arm motors
  arm1 = new rev::CANSparkMax(30, BRUSHLESS);
  arm2 = new rev::CANSparkMax(31, BRUSHLESS);
  wrist = new rev::CANSparkMax(32, BRUSHLESS);
  intake = new rev::CANSparkMax(33, BRUSHLESS);

  //Set inverted
  arm1->SetInverted(false);
  arm2->SetInverted(true);
  wrist->SetInverted(false);
  intake->SetInverted(false);

  //Set to brake or coast
  arm1->SetIdleMode(BRAKE);
  arm2->SetIdleMode(BRAKE);
  wrist->SetIdleMode(BRAKE);
  intake->SetIdleMode(BRAKE);

}

//Public Methods
void Arm::ManualRotateArm(double input){

  arm1->Set(input/2);
  arm2->Set(input/2);
}

void Arm::ManualRotateWrist(double input){

  wrist->Set(input/2);
}

void Arm::Intake(bool buttonIsPressed){

  intake->Set(INTAKE_SPEED);
}

void AutoRotateArm(double position){
  
  
}

void AutoRotateWrist(double position){

}