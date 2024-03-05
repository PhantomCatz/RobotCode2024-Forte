// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;

public class ManualElevatorCmd extends Command {
  private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
  Supplier<Double> m_supplierRightY;
  Supplier<Boolean> m_supplierLeftJoyStickPressed;


  private double pressCounter;

  public ManualElevatorCmd(Supplier<Double> supplierRightY, Supplier<Boolean> supplierLeftJoyStickPressed) {
    addRequirements(elevator);

    this.m_supplierRightY = supplierRightY;
    this.m_supplierLeftJoyStickPressed = supplierLeftJoyStickPressed;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(m_supplierLeftJoyStickPressed.get()) {
      pressCounter = 1;
    }

    if(pressCounter == 1) {
      elevator.setElevatorSemiManualPwr(m_supplierRightY.get());
    } else {
      elevator.setElevatorSemiManualPwr(m_supplierRightY.get());
    }  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
