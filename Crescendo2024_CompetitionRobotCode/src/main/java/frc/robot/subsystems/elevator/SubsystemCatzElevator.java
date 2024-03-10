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
  public static final double ELEVATOR_POS_STOW = 0.0;
  public static final double ElEVATOR_POS_AMP_SCORE_AMP    = 60.0;//8;
  public static final double ELEVATOR_POS_AMP_TRANSITION = 35.0;

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

  //-------------------------------------------------------------------------------------
  // Elevator Variables
  //-------------------------------------------------------------------------------------
  private double m_newPositionRev = 0.0;
  private double m_elevatorPercentOutput = 0.0;
  private double m_finalffVolts = 0.0;
  private double m_finalPIDVolts = 0.0;
  private double m_finalVoltage = 0.0;
  private double elevatorVelocityMTRRPS = 0.0;
  private double currentRotations = 0.0;
  private double previousRotations = 0.0;

  private ElevatorFeedforward elevatorFeedforward;
  private CatzMechanismPosition m_targetPosition;

  private static ElevatorState currentElevatorState;
  public static enum ElevatorState {
    AUTO,
    FULL_MANUAL,
    WAITING,
    SEMI_MANUAL,
    IN_POSITION
  }

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
    elevatorVelocityMTRRPS = (currentRotations - previousRotations)/0.02;
    m_finalffVolts = elevatorFeedforward.calculate(0.0);

    if(DriverStation.isDisabled()) {
      io.setElevatorPercentOutput(0);
      currentElevatorState = ElevatorState.FULL_MANUAL;
      m_elevatorPercentOutput = 0.0;
    }
    else {
      
      if(currentElevatorState == ElevatorState.WAITING) {
          if(SubsystemCatzIntake.getInstance().getWristAngle() < INTAKE_WAIT_THRESHOLD_ANGLE) {
            updateTargetPositionElevator(m_targetPosition);
          } 
      } else if((m_newPositionRev != ELEVATOR_NULL_POSITION) && 
                (currentElevatorState == ElevatorState.AUTO  ||
                 currentElevatorState == ElevatorState.SEMI_MANUAL ||
                 currentElevatorState == ElevatorState.IN_POSITION)) {

            io.setElevatorPosition(m_newPositionRev, 
                                   m_finalffVolts, 
                                   inputs.bottomSwitchTripped);
            if(inputs.elevatorPositionError < 5) {
              currentElevatorState = ElevatorState.IN_POSITION;
            } 
      } else {
        io.setElevatorPercentOutput(m_elevatorPercentOutput);
      }
    }

    Logger.recordOutput("elevator/targetRev", m_newPositionRev);
    Logger.recordOutput("elevator/PercentOut", m_elevatorPercentOutput);
  }


  //-------------------------------------------------------------------------------------
  // Elevator Access Methods
  //-------------------------------------------------------------------------------------
  public void updateTargetPositionElevator(CatzMechanismPosition targetPosition) {
    this.m_targetPosition = targetPosition;

    //set new target position for elevator
    m_newPositionRev = m_targetPosition.getElevatorTargetRev();

    //checks the package for if the intake is trying to get into a position that may collide with the elevator
    if(targetPosition.getIntakePivotTargetAngle() > INTAKE_WAIT_THRESHOLD_ANGLE &&
       getElevatorRevPos() > ELEVATOR_THRESHOLD_REV) {
      currentElevatorState = ElevatorState.WAITING;
      
    } else {
      currentElevatorState = ElevatorState.AUTO;
    }

  }

  public void setElevatorSemiManualPwr(double output) {
    currentElevatorState = ElevatorState.SEMI_MANUAL;
    m_newPositionRev = inputs.elevatorPosRev * (output * ELEVATOR_MANUAL_STEP_SIZE);
   }

  public void setElevatorPercentOutput(double percentOutput) {
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

  public ElevatorState getElevatorState() {
    return currentElevatorState;
  }

 }