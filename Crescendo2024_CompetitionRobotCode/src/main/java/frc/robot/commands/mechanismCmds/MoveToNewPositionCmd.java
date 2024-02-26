// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.ManipulatorMode;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;

public class MoveToNewPositionCmd extends Command {
  
  //subsystem declaration
  private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
  private SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  private SubsystemCatzShooter shooter = SubsystemCatzShooter.getInstance();


  private CatzMechanismPosition m_newPosition;
  private ManipulatorMode       m_manipulatorMode;

  //logic variables
  private int iterationCounter = 0;

  public MoveToNewPositionCmd(CatzMechanismPosition newPosition, ManipulatorMode newManipulatorMode) {
    m_manipulatorMode = newManipulatorMode;
    m_newPosition = newPosition;
    addRequirements(intake, elevator);
  }

  @Override
  public void initialize() {
    intake.updateIntakeTargetPosition(m_newPosition.getIntakePivotTargetAngle());
    elevator.updateElevatorTargetRev(m_newPosition.getElevatorTargetRev());
  }

  @Override
  public void execute() {
      iterationCounter++;
    if(m_newPosition == CatzConstants.CatzMechanismConstants.NOTE_POS_HANDOFF && iterationCounter == 100) {
      intake.setRollerState(2);
      shooter.updateLoadState(1);
    }

    if(m_newPosition == CatzMechanismConstants.NOTE_POS_HANDOFF && iterationCounter == 200) {
      intake.setRollerState(0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
