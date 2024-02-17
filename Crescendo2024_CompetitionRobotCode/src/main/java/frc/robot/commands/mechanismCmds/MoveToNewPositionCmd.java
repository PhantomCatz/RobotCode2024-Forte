// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;

public class MoveToNewPositionCmd extends Command {
  
  SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
  SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();

  private CatzMechanismPosition m_newPosition;

  public MoveToNewPositionCmd(CatzMechanismPosition newPosition) {
    m_newPosition = newPosition;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    System.out.println("new mechanism set cmd");
    intake.updateIntakeTargetPosition(m_newPosition.getIntakePivotTargetAngle());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
