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

  /**
    @brief Controls the arm from a manual input, without PID control.
    @param manualArmPower The amount of power in percent output to set the arm motors to.
    
    When in the arm is in manual mode, this method is responsible for taking the motor input value 
    and setting it to the arm motors.  
  */
  void ManualRotateArm(double manualArmPower);

  /**
    @brief Controls the wrist from a manual input, without PID control.
    @param manualWristPower The amount of power in percent output to set the wrist motor to.

    When in the wrist is in manual mode, this method is responsible for taking the motor input value 
    and setting it to the wrist motor.
  */
  void ManualRotateWrist(double manualWristPower);

  /**
    @brief run the ball intake motors.
    @param intakeSpeed The amount of power in percent output to set the ball intake motor to.
    
    Sets the ball intake motors to the given input speed.
  */
  void RunIntake(double intakeSpeed);

  /**
    @brief high level movement of the arm/wrist system together to the desired position.
    @param targetPosition desired arm position in degrees. (0 is stowed, positive is forward, negative is backwards)
    @param isInBallMode whether or not the arm/wrist is in ball or hatch mode
    @param isInBallPickup whether or not the arm/wrist is in ball pickup mode
    @param isInCargoShipMode whether or not the arm/wrist is in cargo ship mode

    Keeps track of the wrist and arm together to ensure safe movement of the entire system from position to position.
    For example, the wrist is moved to the neautral position if the arm will pass through the danger zone,
    the arm will go to a position outside the dangerzone if the wrist is not in a neutral position yet and the arm wants to pass through,
    the gripper is updated for swinging through the dangerzone, etc, etc.
  */
  void MoveArmToPosition(double targetPosition, bool isInBallMode, bool isInBallPickup, bool isInCargoShipMode); //Degrees
  
  /**
    @brief setter method for the desired gripper state
    @param wantClosed the desired state of the gripper

    Setter method for the desired gripper state. true is for closed/gripped, false if for open/released
  */
  void SetDesiredHatchGripperState(bool wantClosed);

  /**
    @brief getter method for the current arm position in degrees.
    @return the current arm position in degrees

    Getter method for the current arm position in degrees.
  */
  double GetCurrentArmPosition();

  /**
    @brief setter method for whether or not the arm and wrist are in manual or auto
    @param arm Arm manual/auto state. true to set to manual, false to set to auto
    @param wrist Wrist manual/auto state. true to set to manual, false to set to auto
 
    Setter method for whether or not the arm and wrist are in manual or auto
  */
  void UpdateArmAndWristInManual(bool arm, bool wrist);

  /**
    @brief Print arm related shuffleboard info.

    Print arm related shuffleboard info. Many commented out and commented in things for easy changes.
  */
  void PrintArmShuffleInfo();

  /**
    @brief setter method for arm in defense mode.
    @param wantsDefenseMode whether or not you want defense mode. True for wanting defense mode. False for not wanting defense mode.

    Setter method for arm in defense mode.
  */
  void ToggleDefenseMode(bool wantsDefenseMode);

  /**
    @brief getter method for whether or not the gripper is currently open or closed (gripped or released).
    @return true if the gripper is gripper/closed, false if it's released/open.

    Getter method for whether or not the gripper is currently open or closed (gripped or released).
  */
  bool GetIsGripperGripped();

private:
  static Arm *s_instance;
  Arm();

  /**
    @brief Set wrist position reference for the wristPID controller.
    @param wristTargetPosition The degrees of the desired wrist position, relative to the robot frame.

    Set wrist position reference for the wristPID controller, taking into account that the wrist cannot 
    move beyond a certain angle relative to the arm.
  */
  void MoveWristToPosition(double wristTargetPosition); //Degrees

  /**
    @brief Figure out the final desired wrist position depending on the mode of the arm/wrist system
    @param isGoingToBack True if the final arm position in the back of the robot. False otherwise. 
    @param isInBallMode True if in ball mode. False if in hatch mode.
    @param isInBallPickup True if wanting to pickup the ball. False otherwise.
    @param isInCargoShipMode True if in cargo ship mode. False otherwise.

    Depending on these four modes, the final wrist will be in one of the several preset positions.
    For example, if in hatch mode and going to the front, 
    the final wrist will be to face the hatch gripper to the front of the robot. 
  */
  double FindWristFinalPosition(bool isGoingToBack, bool isInBallMode, bool isInBallPickup, bool isInCargoShipMode);
  
  /**
    @brief Opens hatch gripper

    Opens hatch gripper and sets isHatchGripperClosed member variable to keep track of gripper state.
  */
  void OpenHatchGripper();
  
  /**
    @brief Closes hatch gripper

    Closes hatch gripper and sets isHatchGripperClosed member variable to keep track of gripper state.
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
  bool isArmInManual;
  bool isWristInManual;
  double wristTargetPositionShuffle;
  double armTargetPositionShuffle;

  double FFVoltage = 0.0;

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

  //Arm/Wrist Current Limits - amps
  const int ARM_CURRENT_LIMIT = 40;
  const int WRIST_CURRENT_LIMIT = 10;

  //Wrist Positions - degrees
  const int WRIST_BALL_FRONT = -65;
  const int WRIST_HATCH_FRONT = 90;
  const int WRIST_BALL_BACK = 65;
  const int WRIST_HATCH_BACK = -90;
  const int WRIST_NEUTRAL = 0;
  const int WRIST_BALL_PICKUP_FRONT = -190;
  const int WRIST_BALL_PICKUP_BACK = 190;
  const int WRIST_CARGO_SHIP_FRONT = -130;
  const int WRIST_CARGO_SHIP_BACK = 130;

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