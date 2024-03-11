// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.intake.SubsystemCatzIntake;

public class IntakeManualCmd extends Command {
  SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  Supplier<Double> m_supplierLeftJoyY;
  Supplier<Boolean> m_supplierLeftJoyStickPressed;

  public IntakeManualCmd(Supplier<Double> supplierLeftJoyY, Supplier<Boolean> supplierLeftJoyStickPressed) {
    this.m_supplierLeftJoyY = supplierLeftJoyY;
    this.m_supplierLeftJoyStickPressed = supplierLeftJoyStickPressed;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
      intake.pivotSemiManual(m_supplierLeftJoyY.get());
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
