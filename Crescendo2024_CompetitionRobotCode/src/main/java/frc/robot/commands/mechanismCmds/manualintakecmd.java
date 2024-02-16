// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.intake.SubsystemCatzIntake;

public class ManualIntakeCmd extends Command {
  SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  Supplier<Double> supplierLeftJoyX;
  /** Creates a new manualintakecmd. */
  public ManualIntakeCmd(Supplier<Double> supplierLeftJoyX) {
    this.supplierLeftJoyX = supplierLeftJoyX;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.pivotFullManual(supplierLeftJoyX.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
