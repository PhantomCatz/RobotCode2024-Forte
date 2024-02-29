// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.ElevatorConstants;
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

  //elevator constants
  private static final double ElEVATOR_REV_TO_INCHES = 0.0;
  private static final double ELEVATOR_GEAR_RATIO    = 0.0;

  private static final double ELEVATOR_NULL_POSITION = -999.0;

  private static final double ELEVATOR_MANUAL_STEP_SIZE = 0.5;

  private static final double INTAKE_WAIT_THRESHOLD_ANGLE = 60;

  private static final double ELEVATOR_kP = 0.6;
  private static final double ELEVATOR_kI = 0.0;
  private static final double ELEVATOR_kD = 0.0;

  private static final double ELEVATOR_kS = 0.0;
  private static final double ELEVATOR_kG = 0.6;
  private static final double ELEVATOR_kV = 0.0;


  //elevator variables
  private double m_newPositionRev = 0.0;
  private double m_elevatorPercentOutput = 0.0;
  private double m_finalffVolts = 0.0;
  private double m_finalPIDVolts = 0.0;
  private double m_finalVoltage = 0.0;
  private double elevatorVelocityMTRRPS = 0.0;
  private double currentRotations = 0.0;
  private double previousRotations = 0.0;

  private ElevatorFeedforward elevatorFeedforward;
  private PIDController elevatorPID;
  private CatzMechanismPosition m_targetPosition;

  LoggedTunableNumber kgElevatorTunning = new LoggedTunableNumber("kgElevatorTunning", 0.0);

  private static ElevatorState currentElevatorState;
  private static enum ElevatorState {
    AUTO,
    FULL_MANUAL,
    WAITING,
    SEMI_MANUAL
  }

  public SubsystemCatzElevator() {
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

    elevatorFeedforward = new ElevatorFeedforward(ELEVATOR_kS, ELEVATOR_kG, ELEVATOR_kV);
    elevatorPID         = new PIDController(ELEVATOR_kP, ELEVATOR_kI, ELEVATOR_kD);
  }

  // Get the singleton instance of the elevator Subsystem
  public static SubsystemCatzElevator getInstance() {
      return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/inputs", inputs);

    elevatorVelocityMTRRPS = (currentRotations - previousRotations)/0.02;

    if(DriverStation.isDisabled()) {
      io.setElevatorPercentOutput(0);
    }
    else {

      if(currentElevatorState == ElevatorState.WAITING) {
          if(SubsystemCatzIntake.getInstance().getWristAngle() < INTAKE_WAIT_THRESHOLD_ANGLE) {
            currentElevatorState = ElevatorState.AUTO;

          } 
      } else if((m_newPositionRev != ELEVATOR_NULL_POSITION) && 
                (currentElevatorState == ElevatorState.AUTO  ||
                 currentElevatorState == ElevatorState.SEMI_MANUAL)) {
            System.out.println("In elev auto");
            m_finalffVolts = elevatorFeedforward.calculate(0.0);
            m_finalPIDVolts = elevatorPID.calculate(inputs.elevatorPosRev, m_newPositionRev);
            m_finalVoltage = m_finalPIDVolts + m_finalffVolts;
            io.setElevatorVoltage(m_finalVoltage);

      } else {
        io.setElevatorPercentOutput(m_elevatorPercentOutput);

      }
    }

    Logger.recordOutput("elevator/targetRev", m_newPositionRev);
    Logger.recordOutput("elevator/PercentOut", m_elevatorPercentOutput);
  }

  public void updateElevatorTargetRev(CatzMechanismPosition targetPosition) {
    //set new target position for elevator
      m_newPositionRev = targetPosition.getElevatorTargetRev();
    //checks the package for if the intake is trying to get into a position that may collide with the elevator
    if(targetPosition.getIntakePivotTargetAngle() > INTAKE_WAIT_THRESHOLD_ANGLE) {
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
    currentElevatorState = ElevatorState.FULL_MANUAL;
    this.m_elevatorPercentOutput = percentOutput/10;
  }
  // ----------------------------------------------------------------------------------
  // Elevator getters
  // ----------------------------------------------------------------------------------

  public double getElevatorRevPos() {
    return inputs.elevatorPosRev;
  }

}