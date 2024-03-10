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
  Supplier<Boolean> m_supplierLeftJoyStickPressed;


  private double pressCounter;

  public ManualElevatorCmd(Supplier<Double> supplierLeftY, Supplier<Boolean> supplierLeftJoyStickPressed) {
    addRequirements(elevator);

    this.m_supplierLeftY = supplierLeftY;
    this.m_supplierLeftJoyStickPressed = supplierLeftJoyStickPressed;
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
    //   elevator.setElevatorSemiManualPwr(m_supplierLeftY.get());
    // } else {
     // System.out.println(m_supplierLeftY.get());
      elevator.setElevatorPercentOutput(m_supplierLeftY.get());
    //}  
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
