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
#include <string>


#include "ShuffleManager.h"

class Arm
{
public:
  static Arm *GetInstance();

  void ManualRotateArm(double input);
  void ManualRotateWrist(double input);
  void RunIntake(double input);
  void MoveArmToPosition(double targetPosition, bool isInBallMode, bool isInBallPickup); //Degrees
  void CheckHatchGripper(bool isClosed);
  void SetArmToBrake();
  void SetArmToCoast();

  void UpdateArmAndWristInManual(bool arm, bool wrist);

  void ManualCalibrateArm();

  void PrintArmInfo();
  void PrintArmInfotoConsole();


  double GetMAX_FF_GAIN();
  void SetMAX_FF_GAIN(double ArmFFVoltage);

private:
  static Arm *s_instance;
  Arm();

  void MoveWristToPosition(double wristTargetPosition); //Degrees
  double FindWristFinalPosition(bool isGoingToBack, bool isInBallMode, bool isInBallPickup);
  bool GetArmLimitSwitch();
  bool GetWristLimitSwitch();

  //Private Objects
  rev::CANSparkMax *leftArm, *rightArm, *leftWrist;
  VictorSPX *intake;

  rev::CANEncoder *armEncoder;
  rev::CANEncoder *wristEncoder;
  rev::CANPIDController *armPID;
  rev::CANPIDController *wristPID;

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

  int compPrintCount = 0;
  int printCount = 0;

  bool isArmInBack;
  bool isArmSwitchingSides;
  bool isArmMoving;
  bool isWristMoving;
  bool isInHatchMode = true;
  bool isHatchGripperClosed = true;

  bool areWheelsVeryDown;
  bool willArmEnterDZ;
  bool areWheelsUp;
  bool isArmInDZ;
  bool isArmGoingToBack;
  double currentArmPos;
  double currentWristPos;

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

  //Wrist Positions
  const int WRIST_BALL_FRONT = -65;
  const int WRIST_HATCH_FRONT = 90;
  const int WRIST_BALL_BACK = 65;
  const int WRIST_HATCH_BACK = -90;
  const int WRIST_NEUTRAL = 0;
  const int WRIST_BALL_PICKUP_FRONT = -190;
  const int WRIST_BALL_PICKUP_BACK = 190;

  //Tunables
  const double HOLD_BALL_SPEED = 0.05;
  const double CALIBRATION_SPEED = 3000.0;
  const double LIMIT_SWITCH_OFFSET = 5.0;

  //ARM "NO ROTATE ZONE" ANGLE
  const double ARM_DANGERZONE = 30.0;
  const double ARM_CHECKPOINT = 36.0;

  const double WRIST_HATCH_LIMIT = 90;

  //Arm PID Values
  const double MAX_MOTOR_OUTPUT = 0.5; //percent output
  const double TIME_TO_MAX_MOTOR_OUTPUT = 0.5; //secs
  const double RAMP_RATE = TIME_TO_MAX_MOTOR_OUTPUT/MAX_MOTOR_OUTPUT;

  const double MAX_FF_GAIN = 0.67; //Volts required to hold arm at 90 degrees
  const double ARM_FF_GAIN = 0;
  const double P_GAIN_ARM = 0.01;
  const double I_GAIN_ARM = 0;
  const double I_ZONE_ARM = 0;
  const double D_GAIN_ARM = P_GAIN_ARM * 20.0 * 8.0; //8.0 = 2^3 //Rule of thumb for DGain is to multiply P by 20, then keep doubling it (we doubled it three times in this case)
  const double MAX_POWER_ARM = MAX_MOTOR_OUTPUT;
  const double MIN_POWER_ARM = -MAX_MOTOR_OUTPUT;

  //Wrist PID Values
  const double P_GAIN_WRIST = 0.025;
  const double D_GAIN_WRIST = P_GAIN_WRIST * 20; //2 = 2^1 //Rule of thumb for DGain is to multiply P by 20, then keep doubling it (we doubled it once in this case)

  //USE THIS TO SET CONVERSTION FACTOR FOR ENCODER TO READ IN DEGREES OF ARM ROTATION
  const double ARM_GEAR_RATIO = 90.0;
  const double WRIST_GEAR_RATIO = 81.0;
  const double DEGREES_PER_ARM_REVOLUTION = 360.0;
  const double SECONDS_PER_MINUTE = 60.0;

  //This is the PositionConversionFactor
  const double ARM_DEGREES_PER_MOTOR_ROTATION = DEGREES_PER_ARM_REVOLUTION / ARM_GEAR_RATIO;
  const double WRIST_DEGREES_PER_MOTOR_ROTATION = DEGREES_PER_ARM_REVOLUTION / WRIST_GEAR_RATIO;

  //This is the VelocityConversionFactor
  const double ARM_RPM_TO_DEGREES_PER_SECOND = ARM_DEGREES_PER_MOTOR_ROTATION / SECONDS_PER_MINUTE;
  const double WRIST_RPM_TO_DEGREES_PER_SECOND = WRIST_DEGREES_PER_MOTOR_ROTATION / SECONDS_PER_MINUTE;

  //Constants
  const rev::CANSparkMax::MotorType BRUSHLESS = rev::CANSparkMax::MotorType::kBrushless;
  const rev::CANSparkMax::MotorType BRUSHED = rev::CANSparkMax::MotorType::kBrushed;
  const rev::CANSparkMax::IdleMode COAST = rev::CANSparkMax::IdleMode::kCoast;
  const rev::CANSparkMax::IdleMode BRAKE = rev::CANSparkMax::IdleMode::kBrake;
};