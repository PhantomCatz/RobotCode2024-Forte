// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.intake.SubsystemCatzIntake;

public class ManualIntakecmd extends Command {
  SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  Supplier<Double> supplierLeftJoyY;
  /** Creates a new manualintakecmd. */
  public ManualIntakecmd(Supplier<Double> supplierLeftJoyY) {
    this.supplierLeftJoyY = supplierLeftJoyY;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(supplierLeftJoyY.get() > 0.1) {
      System.out.println("going to 10");
      intake.updateIntakeTargetPosition(10);
    } else if (supplierLeftJoyY.get() < -0.1) {
      intake.updateIntakeTargetPosition(-10);
      System.out.println("going to -10");

    } else {
      intake.updateIntakeTargetPosition(0);
    }
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
