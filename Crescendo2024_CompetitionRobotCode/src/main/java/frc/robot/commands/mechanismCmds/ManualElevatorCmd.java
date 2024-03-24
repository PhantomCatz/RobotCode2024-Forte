// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;

public class ManualElevatorCmd extends Command {
  private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
  Supplier<Double> m_supplierLeftY;

  public ManualElevatorCmd(Supplier<Double> supplierLeftY) {
    addRequirements(elevator);

    this.m_supplierLeftY = supplierLeftY;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

      elevator.setElevatorPercentOutput(m_supplierLeftY.get());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
