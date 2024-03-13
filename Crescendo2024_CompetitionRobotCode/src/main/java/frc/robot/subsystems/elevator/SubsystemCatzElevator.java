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
  public static double REV_SWITCH_POS = 0.0; //dummy
  public static double FWD_SWITCH_POS = 5.0; //dummy
  
  private static final double ElEVATOR_REV_TO_INCHES = 0.0;

  private static final double ElEVATOR_DRIVEN_GEAR = 42;
  private static final double ELEVATOR_DRIVING_GEAR = 10;

  private static final double ELEVATOR_GEAR_RATIO    = ElEVATOR_DRIVEN_GEAR/ELEVATOR_DRIVING_GEAR;

  private static final double ELEVATOR_NULL_POSITION = -999.0;

  private static final double ELEVATOR_MANUAL_STEP_SIZE = 0.5;

  //crash check constants
  private static final double INTAKE_WAIT_THRESHOLD_ANGLE = 60;
  private static final double ELEVATOR_THRESHOLD_REV      = 20;

  private static final double ELEVATOR_kS = 0.0;
  private static final double ELEVATOR_kG = 0.8;
  private static final double ELEVATOR_kV = 0.0;

 //-----------------------------------------------------------------------------------------------
  //  Elevator position defs & variables
  //-----------------------------------------------------------------------------------------------
  public static final double ELEVATOR_STOW           = 0.0;
  public static final double ELEVATOR_AMP_TRANSITION = 35.0;
  public static final double ElEVATOR_SOURCE_PICKUP  = 40.0;
  public static final double ELEVATOR_SCORE_AMP      = 60.0;
  public static final double ELEVATOR_SCORE_TRAP     = 66.0;

  //-------------------------------------------------------------------------------------
  // Elevator Variables
  //-------------------------------------------------------------------------------------
  private double m_newPositionRev = 0.0;
  private double m_elevatorPercentOutput = 0.0;
  private double m_finalffVolts = 0.0;
  private double elevatorVelocityMTRRPS = 0.0;

  private boolean m_elevatorInPos = false;

  private ElevatorFeedforward elevatorFeedforward;

  private static ElevatorState currentElevatorState;
  public static enum ElevatorState {
    AUTO,
    FULL_MANUAL,
    SEMI_MANUAL
  }

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
    if(inputs.bottomSwitchTripped) {
      io.setSelectedSensorPosition(0.0);
    }

    //elevator control calculations
    //elevatorVelocityMTRRPS = (currentRotations - previousRotations)/0.02;
    m_finalffVolts = elevatorFeedforward.calculate(0.0);

    if(DriverStation.isDisabled()) {
      io.setElevatorPercentOutput(0);
      currentElevatorState = ElevatorState.FULL_MANUAL;
      m_elevatorPercentOutput = 0.0;
    }
    else {
      
      if(m_elevatorIntakeInSafetyZone == false) {
        if(SubsystemCatzIntake.getInstance().getWristAngle() < SubsystemCatzIntake.INTAKE_STOW_ELEV_CLEARED_DEG) {
          m_elevatorIntakeInSafetyZone = true;
        } 
      }

      if(m_elevatorIntakeInSafetyZone == true) {
        if((m_newPositionRev != ELEVATOR_NULL_POSITION) && 
                (currentElevatorState == ElevatorState.AUTO  ||
                 currentElevatorState == ElevatorState.SEMI_MANUAL)) {

            io.setElevatorPosition(m_newPositionRev, 
                                   m_finalffVolts, 
                                   inputs.bottomSwitchTripped);
            if(inputs.elevatorPositionError < 5) {
              m_elevatorInPos = true;
            } 
        } else {
          io.setElevatorPercentOutput(m_elevatorPercentOutput);
        }
      }
    }

    Logger.recordOutput("elevator/targetRev", m_newPositionRev);
    Logger.recordOutput("elevator/PercentOut", m_elevatorPercentOutput);
    Logger.recordOutput("elevator/elevatorin safety", m_elevatorIntakeInSafetyZone);
  }


  //-------------------------------------------------------------------------------------
  // Elevator Access Methods
  //-------------------------------------------------------------------------------------
  public void updateTargetPositionElevator(CatzMechanismPosition targetPosition) {
    m_elevatorInPos = false;
    currentElevatorState = ElevatorState.AUTO;
    m_elevatorIntakeInSafetyZone = false;
    System.out.println("in update elevator");
    m_newPositionRev = targetPosition.getElevatorTargetRev();

    if(m_newPositionRev < SubsystemCatzIntake.INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV) {
      m_elevatorIntakeInSafetyZone = true;
      //-------------------------------------------------------------------------------------
      //  If elevator is below the stow clearance. No change
      //  If elevator is above the stow clearance. No change
      //----------------------------------------------------------------------------------

     } else if(m_newPositionRev > SubsystemCatzIntake.INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV){//m_newPositionRev == ELEVATOR_SCORE_AMP ||
    //           m_newPositionRev == ELEVATOR_AMP_TRANSITION ||
    //           m_newPositionRev == ELEVATOR_SCORE_TRAP ||
    //           m_newPositionRev == ElEVATOR_SOURCE_PICKUP ||
    //           m_newPositionRev == ELEVATOR_SCORE_AMP) {
                System.out.println("J");
      //-------------------------------------------------------------------------------------
      //  If elevator position is above the stow clearance. Assume intake is in position
      //  If elevator position is below the stow clearance. intake is not is position
      //----------------------------------------------------------------------------------

      if(inputs.elevatorPosRev > SubsystemCatzIntake.INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV) {
          m_elevatorIntakeInSafetyZone = true;
      }
    }
  }

  public void setElevatorSemiManualPwr(double output) {
    m_elevatorInPos = false;
    currentElevatorState = ElevatorState.SEMI_MANUAL;
    m_newPositionRev = inputs.elevatorPosRev * (output * ELEVATOR_MANUAL_STEP_SIZE);
   }

  public void setElevatorPercentOutput(double percentOutput) {
    m_elevatorInPos = false;
    System.out.println("in manual");
    currentElevatorState = ElevatorState.FULL_MANUAL;
    this.m_elevatorPercentOutput = percentOutput/5;
  }

  // ----------------------------------------------------------------------------------
  // Elevator getters
  // ----------------------------------------------------------------------------------
  public double getElevatorRevPos() {
    return inputs.elevatorPosRev;
  }

  private ElevatorState getElevatorState() {
    return currentElevatorState;
  }

  public boolean getElevatorInPos() {
    return m_elevatorInPos;
  }

 }