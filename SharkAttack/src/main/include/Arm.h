/*----------------------------------------------------------------------------------*/

//Header file for Arm.cpp

/*----------------------------------------------------------------------------------*/

#pragma once
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/PowerDistributionPanel.h>
#include "math.h"
#include <iostream>

#include "ShuffleManager.h"

class Arm
{
public:
  static Arm *GetInstance();

  void ManualRotateArm(double input);
  void ManualRotateWrist(double input);
  void RunIntake(double in, double out);
  //one button per preset : total of 8 buttons
  //2 buttons to switch between ball and panel
  //struct for different wrist and panel
  double GetArmEncoderValue();
  double GetWristEncoderValue();
  void MoveArmToPosition(double targetPosition); //Degrees
  void MoveWristToPosition(double wristCurrentPosition, double armCurrentPosition);                      //Degrees
  void CheckHatchGripper(bool isClosed);
  void SetArmToBrake();
  void SetArmToCoast();

  void ManualCalibrateArm();

  void PrintArmInfo();

private:
  static Arm *s_instance;
  Arm();

  bool GetArmLimitSwitch();
  bool GetWristLimitSwitch();

  //Private Objects
  rev::CANSparkMax *leftArm, *rightArm, *leftWrist, *rightWrist;
  VictorSPX *intake;

  rev::CANEncoder *armEncoder;
  rev::CANEncoder *wristEncoder;

  frc::DigitalInput *armLimitSwitch;
  frc::DigitalInput *wristLimitSwitch;

  frc::DoubleSolenoid *hatchGripper;

  frc::PowerDistributionPanel *pdp;

  bool wasArmLimitSwitchTripped;
  bool wasWristLimitSwitchTripped;

  bool hasBall = false;
  bool isArmInManual;
  double previousTargetPosition;

  //CAN Motor IDs
  const int LEFT_ARM_ID = 42; //42 is actual, was changed for testing. Change back
  const int RIGHT_ARM_ID = 43;
  const int LEFT_WRIST_ID = 44;
  const int RIGHT_WRIST_ID = 45; //45 is actual, was changed for testing. Change back
  const int INTAKE_ID = 46;

  const int INTAKE_PDP_PORT = 10;
  const double INTAKE_MAX_CURRENT = 40.0;

  const int ARM_CURRENT_LIMIT = 40;
  const int WRIST_CURRENT_LIMIT = 10;

  //Tunables
  const double HOLD_BALL_SPEED = 0.05;
  const double WRIST_ADJUSTER = 400;
  const double DANGER_ZONE_LIMIT = 50;
  const double CALIBRATION_SPEED = 3000.0;
  const double LIMIT_SWITCH_OFFSET = 5.0;

  //Arm PID Values
  const double ARM_MAX_POWER = 0.5;

  const double FFGAIN_ARM = 0.0625; //
  const double PGAIN_ARM = 0.01;


  //USE THIS TO SET CONVERSTION FACTOR FOR ENCODER TO READ IN DEGREES OF ARM ROTATION
  const double ARM_GEAR_RATIO = 81.0;
  const double DEGREES_PER_REVOLUTION = 360.0;
  const double DEGREES_PER_MOTOR_ROTATION = DEGREES_PER_REVOLUTION / ARM_GEAR_RATIO;

  //Constants
  const rev::CANSparkMax::MotorType BRUSHLESS = rev::CANSparkMax::MotorType::kBrushless;
  const rev::CANSparkMax::MotorType BRUSHED = rev::CANSparkMax::MotorType::kBrushed;
  const rev::CANSparkMax::IdleMode COAST = rev::CANSparkMax::IdleMode::kCoast;
  const rev::CANSparkMax::IdleMode BRAKE = rev::CANSparkMax::IdleMode::kBrake;
};