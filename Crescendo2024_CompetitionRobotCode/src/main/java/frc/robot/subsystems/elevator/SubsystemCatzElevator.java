// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.Utils.LoggedTunableNumber;
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.SubsystemCatzIntake;

public class SubsystemCatzElevator extends SubsystemBase {
  //instance instantiation
  private static SubsystemCatzElevator instance = new SubsystemCatzElevator();

  //io block
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  //-------------------------------------------------------------------------------------
  // Elevator Constants
  //-------------------------------------------------------------------------------------
  private static final double REV_SWITCH_POS = 0.0; 
  
  private static final double ElEVATOR_REV_TO_INCHES = 0.0;

  private static final double MAXPLANETARY_GEAR_RATIO = 4.0 * 4.0;

  private static final double ELEVATOR_SPOOL_DIA_PULL_UP_INCHES = 0.7; //TBD on the name of the variable
  private static final double ELEVATOR_SPOOL_DIA_PULL_DN_INCHES = 1.4; //TBD on the name of the variable


  private static final double ELEVATOR_GEAR_RATIO    = MAXPLANETARY_GEAR_RATIO;


    private static final double ELEVATOR_MANUAL_STEP_SIZE = 0.5;

 //-----------------------------------------------------------------------------------------------
  //  Elevator CLOSED loop defs and variables
  //-----------------------------------------------------------------------------------------------

  private static final double ELEVATOR_kS = 0.0;
  private static final double ELEVATOR_kG = 0.8;
  private static final double ELEVATOR_kV = 0.0;
  private double m_ffVolts = 0.0;

  private double m_targetPositionRev = 0.0;
  private double m_elevatorPercentOutput = 0.0;
  private double elevatorVelocityMtrRPS = 0.0;
  
 //-----------------------------------------------------------------------------------------------
  //  Elevator position defs & variables
  //-----------------------------------------------------------------------------------------------
  public static final double ELEVATOR_STOW           = 0.0;
  public static final double ELEVATOR_GROUND_PICKUP  = 0.0;
  public static final double ELEVATOR_AMP_SCORE_DN   = 0.0;
  public static final double ELEVATOR_AMP_TRANSITION = 50.0;
  public static final double ELEVATOR_SOURCE_PICKUP  = 60.0;
  public static final double ELEVATOR_AMP_SCORE      = 90.0;
  public static final double ELEVATOR_SCORE_TRAP     = 110.0;

  private static final double ELEVATOR_NULL_POSITION = -999.0;


  //-------------------------------------------------------------------------------------
  // Elevator Variables
  //-------------------------------------------------------------------------------------

  private double intakeClearanceAngle;

  private boolean m_elevatorInPos = false;

  private ElevatorFeedforward elevatorFeedforward;

  private static ElevatorControlState currentElevatorState;
  public static enum ElevatorControlState {
    AUTO,
    FULL_MANUAL,
    SEMI_MANUAL
  }

  private static enum ElevatorDirection {
    UP,DOWN
  }

  private static ElevatorDirection currentElevatorDirection = ElevatorDirection.UP;

  private boolean m_elevatorIntakeInSafetyZone = false;


  private SubsystemCatzElevator() {
    switch (CatzConstants.currentMode) {
      case REAL: io = new ElevatorIOReal();
                 System.out.println("Elevator Configured for Real");
      break;

      case REPLAY: io = new ElevatorIOReal() {};
                   System.out.println("Elevator Configured for Replayed simulation");
      break;

      case SIM:
      default: io = null;
               System.out.println("Elevator Unconfigured");
      break;
    }

    elevatorFeedforward = new ElevatorFeedforward(ELEVATOR_kS, 
                                                  ELEVATOR_kG, 
                                                  ELEVATOR_kV);
  }

  // Get the singleton instance of the elevator Subsystem
  public static SubsystemCatzElevator getInstance() {
      return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/inputs", inputs);


    //elevator control calculations
    m_ffVolts = elevatorFeedforward.calculate(0.0); //calculating while disabled for advantage scope logging

    if(DriverStation.isDisabled()) {
      io.setElevatorPercentOutput(0);
      currentElevatorState = ElevatorControlState.FULL_MANUAL;
      m_elevatorPercentOutput = 0.0;
    }
    else {
      if(inputs.bottomSwitchTripped) {
        io.setSelectedSensorPosition(0.0);
      }

      if((currentElevatorState == ElevatorControlState.AUTO  ||
          currentElevatorState == ElevatorControlState.SEMI_MANUAL)) {

        if((m_targetPositionRev != ELEVATOR_NULL_POSITION)) {

          if(m_elevatorIntakeInSafetyZone == false) {
            if(currentElevatorDirection == ElevatorDirection.DOWN) {
              if(SubsystemCatzIntake.getInstance().getWristAngle() > intakeClearanceAngle) {
                m_elevatorIntakeInSafetyZone = true;
                        System.out.println("E-G");

              } 
            } else {
              if(SubsystemCatzIntake.getInstance().getWristAngle() < intakeClearanceAngle) {
                m_elevatorIntakeInSafetyZone = true;
                 System.out.println("E-F");

              }     
            }
          }

          if(m_elevatorIntakeInSafetyZone == true) {
            io.setElevatorPosition(m_targetPositionRev, 
                                    m_ffVolts, 
                                    inputs.bottomSwitchTripped);

            if(inputs.elevatorPositionError < 0.2) {
              m_elevatorInPos = true;
            } 
          }
        }
      } else {
        io.setElevatorPercentOutput(m_elevatorPercentOutput);
      }
    }
  

    Logger.recordOutput("elevator/targetRev", m_targetPositionRev);
    Logger.recordOutput("elevator/PercentOut", m_elevatorPercentOutput);
    Logger.recordOutput("elevator/elevatorin safety", m_elevatorIntakeInSafetyZone);
    Logger.recordOutput("elevator/GOing up", currentElevatorDirection.toString());
  }


  //-------------------------------------------------------------------------------------
  // Elevator Access Methods
  //-------------------------------------------------------------------------------------
  public void updateTargetPositionElevator(CatzMechanismPosition targetPosition) {
    // System.out.println("EUP" + targetPosition.getElevatorTargetRev());
    m_elevatorInPos = false;
    currentElevatorState = ElevatorControlState.AUTO;

    m_elevatorIntakeInSafetyZone = false;
    m_targetPositionRev = targetPosition.getElevatorTargetRev();

    if(m_targetPositionRev == ELEVATOR_STOW ||
       m_targetPositionRev == ELEVATOR_GROUND_PICKUP ||
       m_targetPositionRev == ELEVATOR_AMP_SCORE_DN) {
                // System.out.println("E-Z");
      //-------------------------------------------------------------------------------------
      //  If elevator is below the stow clearance. No change
      //  If elevator is above the stow clearance. No change
      //----------------------------------------------------------------------------------
      currentElevatorDirection = ElevatorDirection.DOWN;
      intakeClearanceAngle = SubsystemCatzIntake.INTAKE_GROUND_PICKUP_DEG;
      // System.out.println(SubsystemCatzIntake.getInstance().getWristAngle());
      if(SubsystemCatzIntake.getInstance().getWristAngle() > SubsystemCatzIntake.INTAKE_GROUND_PICKUP_DEG) {
        
        // System.out.println("E-A");
           //-------------------------------------------------------------------------------------
          //  intake is above bumpers
          //----------------------------------------------------------------------------------
          m_elevatorIntakeInSafetyZone = true;
      }
    } else if(m_targetPositionRev == ELEVATOR_SOURCE_PICKUP ||
              m_targetPositionRev == ELEVATOR_AMP_TRANSITION) {        
        // System.out.println("E-B");

      currentElevatorDirection = ElevatorDirection.UP;
      intakeClearanceAngle = SubsystemCatzIntake.INTAKE_MIN_ELEV_CLEARANCE_DEG;
      if(SubsystemCatzIntake.getInstance().getWristAngle() < SubsystemCatzIntake.INTAKE_MIN_ELEV_CLEARANCE_DEG) {
        // System.out.println("E-C");

        //-------------------------------------------------------------------------------------
        //  intake is in front of elevator
        //----------------------------------------------------------------------------------
        m_elevatorIntakeInSafetyZone = true;
      }
    } else if(m_targetPositionRev == ELEVATOR_AMP_SCORE) {
        // System.out.println("E-D");

      currentElevatorDirection = ElevatorDirection.UP;
      intakeClearanceAngle = SubsystemCatzIntake.INTAKE_TRANSITION_CHECK_DEG;
      if(SubsystemCatzIntake.getInstance().getWristAngle() < SubsystemCatzIntake.INTAKE_TRANSITION_CHECK_DEG) {
        //-------------------------------------------------------------------------------------
        //  intake is in front of elevator
        //----------------------------------------------------------------------------------
        m_elevatorIntakeInSafetyZone = true;
      } 
    } else {
      System.out.println("Invalid elevator target Angle");
    }

  }

  public void setElevatorSemiManualPwr(double output) {
    m_elevatorInPos = false;
    currentElevatorState = ElevatorControlState.SEMI_MANUAL;
    m_targetPositionRev = inputs.elevatorPosRev * (output * ELEVATOR_MANUAL_STEP_SIZE);
   }

  public void setElevatorPercentOutput(double percentOutput) {
    m_elevatorInPos = false;
    currentElevatorState = ElevatorControlState.FULL_MANUAL;
    this.m_elevatorPercentOutput = percentOutput/5;
  }

  public void setElevatorOff() {
    
  }

  // ----------------------------------------------------------------------------------
  // Elevator getters
  // ----------------------------------------------------------------------------------
  public double getElevatorRevPos() {
    return inputs.elevatorPosRev;
  }

  private ElevatorControlState getElevatorState() {
    return currentElevatorState;
  }

  public boolean getElevatorInPos() {
    return m_elevatorInPos;
  }

 }