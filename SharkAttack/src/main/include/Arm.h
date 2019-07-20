/*----------------------------------------------------------------------------------*/

//Header file for Arm.cpp

/*----------------------------------------------------------------------------------*/

/** doxygen example
    @brief Returns best target based on current planks.
    @param num_Robots Number of robots in game.
    @return The robot with best current plank.
    Loops through the robots in current state to find the robot with the highest
    value plank. Returns this robot if found, otherwise returns an empty Robot.
*/

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

  /**
    @brief Controls the arm from a manual input, without PID control
    @param manualArmPower The amount of power in percent output to set the arm motors to
    When in the arm is in manual mode, this method is responsible for taking the motor input value 
    and setting it to the arm motors.  
  */
  void ManualRotateArm(double manualArmPower);

  /**
    @brief Controls the wrist from a manual input, without PID control
    @param manualWristPower The amount of power in percent output to set the wrist motor to
    When in the wrist is in manual mode, this method is responsible for taking the motor input value 
    and setting it to the wrist motor.
  */
  void ManualRotateWrist(double manualWristPower);

  /**
    @brief run the ball intake motors
    @param intakeSpeed The amount of power in percent output to set the ball intake motor to
    Sets the ball intake motors to the given input speed.
  */
  void RunIntake(double intakeSpeed);
  /**
    @brief 
    @param targetPosition
    @param isInBallMode
    @param isInBallPickup
    @param isInCargoShipMode
    
    
  */
  void MoveArmToPosition(double targetPosition, bool isInBallMode, bool isInBallPickup, bool isInCargoShipMode); //Degrees
  void SetDesiredHatchGripperState(bool wantClosed);
  void SetArmToBrake();
  void SetArmToCoast();
  void SetToMatchMode();
  double GetCurrentArmPosition();

  void UpdateArmAndWristInManual(bool arm, bool wrist);

  void PrintArmShuffleInfo();
  void ToggleDefenseMode(bool isArmInDefenseMode);

  bool GetIsGripperGripped();

private:
  static Arm *s_instance;
  Arm();

  
  void MoveWristToPosition(double wristTargetPosition); //Degrees
  double FindWristFinalPosition(bool isGoingToBack, bool isInBallMode, bool isInBallPickup, bool isInCargoShipMode);
  
  /**
    @brief Opens hatch gripper
  */
  void OpenHatchGripper();
  
  /**
    @brief Closes hatch gripper
  
  */
  void CloseHatchGripper();

  //Private Objects
  rev::CANSparkMax *leftArm, *rightArm, *wrist;
  VictorSPX *intake;

  rev::CANEncoder *armEncoder;
  rev::CANEncoder *wristEncoder;
  rev::CANPIDController *armPID;
  rev::CANPIDController *wristPID;

  frc::DoubleSolenoid *hatchGripper;

  //Instance Variables - state of arm/wrist
  bool hasBall = false;
  bool isArmInManual;
  bool isWristInManual;
  double previousTargetPosition;
  double previousTargetWristPosition;
  double wristTargetPositionShuffle;
  double armTargetPositionShuffle;

  double FFVoltage = 0.0;

  int compPrintCount = 0;
  int printCount = 0;

  bool isArmInBack;
  bool isArmSwitchingSides;
  bool isInHatchMode = true;
  bool isHatchGripperClosed = true;
  bool wantHatchGripperClosed = true;

  bool wasInBallMode = false;

  bool isArmInDefenseMode = false;

  bool areWheelsVeryDown;
  bool willArmEnterDZ;
  bool areWheelsUp;
  bool isArmInDZ;
  bool isArmGoingToBack;
  double currentArmPos;
  double currentWristPos;

  //----------------------------------CONSTANTS-------------------------------------------
  //CAN Motor IDs
  const int LEFT_ARM_ID = 42;
  const int RIGHT_ARM_ID = 43;
  const int LEFT_WRIST_ID = 44;
  const int RIGHT_WRIST_ID = 45; 
  const int INTAKE_ID = 46;

  //Gripper Solenoid IDs
  const int SOLENOID_FORWARD = 2;
  const int SOLENOID_REVERSE = 3;

  //Arm/Wrist Current Limits
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
  const int WRIST_CARGO_SHIP_FRONT = -130;
  const int WRIST_CARGO_SHIP_BACK = 130;

  //Tunables
  const double HOLD_BALL_SPEED = 0.05;
  const double CALIBRATION_SPEED = 3000.0;
  const double LIMIT_SWITCH_OFFSET = 5.0;

  //ARM "NO ROTATE ZONE" ANGLE
  const double ARM_DANGERZONE = 22;
  const double ARM_CHECKPOINT = 36.0;

  const double WRIST_HATCH_LIMIT = 90;

  //Arm PID Values
  const double MAX_MOTOR_OUTPUT_FIELD = 0.5;  //percent output
  const double MAX_MOTOR_OUTPUT_PIT = 0.5;    //percent output - was .35/.45
  const double TIME_TO_MAX_MOTOR_OUTPUT = 0.5; //secs
  const double RAMP_RATE_FIELD = TIME_TO_MAX_MOTOR_OUTPUT / MAX_MOTOR_OUTPUT_FIELD;
  const double RAMP_RATE_PIT = TIME_TO_MAX_MOTOR_OUTPUT / MAX_MOTOR_OUTPUT_PIT;

  const double MAX_FF_GAIN = 0.67; //Volts required to hold arm at 90 degrees
  const double ARM_FF_GAIN = 0;
  const double P_GAIN_ARM = 0.01;
  const double I_GAIN_ARM = 0;
  const double I_ZONE_ARM = 0;
  const double D_GAIN_ARM = P_GAIN_ARM * 20.0 * 8.0; //8.0 = 2^3 //Rule of thumb for DGain is to multiply P by 20, then keep doubling it (we doubled it three times in this case)
  const double P_GAIN_ARM_DEFENSE = 0.1;
  const double I_GAIN_ARM_DEFENSE = 0;
  const double I_ZONE_ARM_DEFENSE = 0;
  const double D_GAIN_ARM_DEFENSE = P_GAIN_ARM_DEFENSE * 20.0 * 8.0; //8.0 = 2^3 //Rule of thumb for DGain is to multiply P by 20, then keep doubling it (we doubled it three times in this case)
  const double MAX_POWER_ARM_FIELD = MAX_MOTOR_OUTPUT_FIELD;
  const double MIN_POWER_ARM_FIELD = -MAX_MOTOR_OUTPUT_FIELD;
  const double MAX_POWER_ARM_PIT = MAX_MOTOR_OUTPUT_PIT;
  const double MIN_POWER_ARM_PIT = -MAX_MOTOR_OUTPUT_PIT;

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