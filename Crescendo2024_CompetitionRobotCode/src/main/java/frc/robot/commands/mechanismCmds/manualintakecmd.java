// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.intake.SubsystemCatzIntake;

public class ManualIntakeCmd extends Command {
  SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  Supplier<Double> supplierLeftJoyY;
  public ManualIntakeCmd(Supplier<Double> supplierLeftJoyY) {
    this.supplierLeftJoyY = supplierLeftJoyY;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
