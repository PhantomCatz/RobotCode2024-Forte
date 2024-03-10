// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.intake.SubsystemCatzIntake;

public class ManualIntakeCmd extends Command {
  SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  Supplier<Double> m_supplierLeftJoyY;
  Supplier<Boolean> m_supplierLeftJoyStickPressed;

  private int pressCounter = 0;
  public ManualIntakeCmd(Supplier<Double> supplierLeftJoyY, Supplier<Boolean> supplierLeftJoyStickPressed) {
    this.m_supplierLeftJoyY = supplierLeftJoyY;
    this.m_supplierLeftJoyStickPressed = supplierLeftJoyStickPressed;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    pressCounter = 0;
  }

  @Override
  public void execute() {
    // if(m_supplierLeftJoyStickPressed.get()) {
    //   pressCounter = 1;
    // }

    // if(pressCounter == 1) {
    //   intake.pivotFullManual(m_supplierLeftJoyY.get());
   // } else {
      intake.pivotSemiManual(m_supplierLeftJoyY.get());
      
    //}
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
