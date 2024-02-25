// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.ElevatorConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;

public class SubsystemCatzElevator extends SubsystemBase {
  //instance instantiation
  private static SubsystemCatzElevator instance = new SubsystemCatzElevator();

  //io block
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  //elevator constants
  private double m_newPositionRev;

  private double m_elevatorPercentOutput;

  private static ElevatorState currentElevatorState;
  private static enum ElevatorState {
    AUTO,
    FULL_MANUAL,
    SEMI_MANUAL
  }

  public SubsystemCatzElevator() {
            switch (CatzConstants.currentMode) {
            case REAL: io = 
                    new ElevatorIOReal();
                break;
            case SIM : io = null;
                break;
            default : io = 
                    new ElevatorIOReal() {};
                break;
        }
  }

  // Get the singleton instance of the elevator Subsystem
  public static SubsystemCatzElevator getInstance() {
      return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/inputs", inputs);
    if(DriverStation.isDisabled()) {
      io.setElevatorPercentOutput(0);
    }
    else if((m_newPositionRev != -999) && currentElevatorState == ElevatorState.AUTO) {
      io.setElevatorPosition(m_newPositionRev);
      System.out.println("in auto");
    } else {
      io.setElevatorPercentOutput(m_elevatorPercentOutput);
    }
/* 
    if(inputs.forwardSwitchTripped){
      io.setSelectedSensorPosition(ElevatorConstants.FWD_SWITCH_POS);
    }
    if(inputs.reverseSwitchTripped){
      io.setSelectedSensorPosition(ElevatorConstants.REV_SWITCH_POS);
    }
    */
  }

  public void updateElevatorTargetRev(double targetPos) {
    currentElevatorState = ElevatorState.AUTO;
    m_newPositionRev = targetPos;
  }

  public void setElevatorPercentOutput(double percentOutput) {
    currentElevatorState = ElevatorState.FULL_MANUAL;
    this.m_elevatorPercentOutput = percentOutput/10;
  }
}