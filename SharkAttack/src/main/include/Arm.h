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
  void RunIntake(double input);
  double GetArmEncoderValue();
  double GetWristEncoderValue();
  void MoveArmToPosition(double targetPosition);                                    //Degrees
  void MoveWristToPosition( double wristTargetPosition); //Degrees
  void CheckHatchGripper(bool isClosed);
  void SetArmToBrake();
  void SetArmToCoast();

  void ManualCalibrateArm();

  void PrintArmInfo();

  double GetMAX_FF_GAIN();
  void SetMAX_FF_GAIN(double ArmFFVoltage);

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
  rev::CANPIDController *armPID;

  frc::DigitalInput *armLimitSwitch;
  frc::DigitalInput *wristLimitSwitch;

  frc::DoubleSolenoid *hatchGripper;

  frc::PowerDistributionPanel *pdp;

  //Instance Variables
  bool wasArmLimitSwitchTripped;
  bool wasWristLimitSwitchTripped;

  bool hasBall = false;
  bool isArmInManual;
  bool isWristInManual;
  double previousTargetPosition;
  double previousTargetWristPosition;

  double FFVoltage = 0.0;

  //CAN Motor IDs
  const int LEFT_ARM_ID = 42;    //42 is actual, was changed for testing. Change back
  const int RIGHT_ARM_ID = 43;   //43 is actual
  const int LEFT_WRIST_ID = 44;  //44 is actual
  const int RIGHT_WRIST_ID = 45; //45 is actual, was changed for testing. Change back
  const int INTAKE_ID = 46;

  const int INTAKE_PDP_PORT = 10;
  const double INTAKE_MAX_CURRENT = 40.0;

  const int ARM_CURRENT_LIMIT = 40;
  const int WRIST_CURRENT_LIMIT = 10;

  //Tunables
  const double HOLD_BALL_SPEED = 0.05;
  const double DANGER_ZONE_LIMIT = 20;
  const double CALIBRATION_SPEED = 3000.0;
  const double LIMIT_SWITCH_OFFSET = 5.0;

  //Wrist PID Values
  const double WRIST_P_GAIN = 0.025; //Was .06 when experiencing issues

  //Arm PID Values
  const double MAX_FF_GAIN = 0.60; //Volts required to hold arm at 90 degrees
  const double ARM_FF_GAIN = (1.15/360);
  const double P_GAIN_ARM = 0.0017;
  const double I_GAIN_ARM = 0.000034;
  const double I_ZONE_ARM = 2;
  const double D_GAIN_ARM = P_GAIN_ARM * 200; //Rule of thumb for DGain is to multiply P by 20
  const double MAX_POWER_ARM = 1.0;
  const double MIN_POWER_ARM = -1.0;

  const double MAX_VEL_ARM = 90.0; //Was 60 //Everything here is Degrees/Sec
  const double MIN_VEL_ARM = 0.0;
  const double MAX_ACCEL_ARM = 40.0; //Was 20
  const double ALLOWED_ERROR_ARM = 2.0; //Was 1.0

  //USE THIS TO SET CONVERSTION FACTOR FOR ENCODER TO READ IN DEGREES OF ARM ROTATION
  const double ARM_GEAR_RATIO = 74.97;
  const double DEGREES_PER_ARM_REVOLUTION = 360.0;
  const double SECONDS_PER_MINUTE = 60.0;

  //This is the PositionConversionFactor
  const double DEGREES_PER_MOTOR_ROTATION = DEGREES_PER_ARM_REVOLUTION / ARM_GEAR_RATIO;

  //THis is the VelocityConversionFactor
  const double RPM_TO_DEGREES_PER_SECOND = DEGREES_PER_MOTOR_ROTATION / SECONDS_PER_MINUTE;

  //Constants
  const rev::CANSparkMax::MotorType BRUSHLESS = rev::CANSparkMax::MotorType::kBrushless;
  const rev::CANSparkMax::MotorType BRUSHED = rev::CANSparkMax::MotorType::kBrushed;
  const rev::CANSparkMax::IdleMode COAST = rev::CANSparkMax::IdleMode::kCoast;
  const rev::CANSparkMax::IdleMode BRAKE = rev::CANSparkMax::IdleMode::kBrake;
};