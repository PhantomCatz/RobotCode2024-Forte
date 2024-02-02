// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StateMachineCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;

public class MoveToNewPositionCmd extends Command {
  /** Creates a new MoveToNewPositionCmd. */
  SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();

  CatzMechanismPosition m_newPosition;
  public MoveToNewPositionCmd(CatzMechanismPosition newPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_newPosition = newPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setNewPos(m_newPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
