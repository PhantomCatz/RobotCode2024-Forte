// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;


public class SubsystemCatzElevator extends SubsystemBase {
  
  private final ElevatorIO io;
  private static SubsystemCatzElevator instance;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private CatzMechanismPosition m_newPosition;

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

  //Run every 20 ms
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/inputs", inputs);

    double targetEncPos;

    if(DriverStation.isDisabled()) {
      io.setElevatorPercentOutput(0);
    }
    else if(m_newPosition != null) {
      targetEncPos = m_newPosition.getElevatorTargetEncPos();
      io.setElevatorPosition(targetEncPos);
      Logger.recordOutput("targetEncElevator", targetEncPos);
    }

    io.exampleAccessMethod(0);
  }

  public void setNewPos(CatzMechanismPosition newPosition) {
    this.m_newPosition = newPosition;
  }

  // Get the singleton instance of the ClimbSubsystem
  public static SubsystemCatzElevator getInstance() {
      return instance;
  }

}
