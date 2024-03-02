// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzMechanismPosition;
// import frc.robot.subsystems.elevator.SubsystemCatzElevator;
// import frc.robot.subsystems.intake.SubsystemCatzIntake;
// import frc.robot.subsystems.shooter.SubsystemCatzShooter;

public class MoveToNewPositionCmd extends Command {
  
  //subsystem declaration
  // private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
  // private SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  // private SubsystemCatzShooter shooter = SubsystemCatzShooter.getInstance();


  private CatzMechanismPosition m_newPosition;

  //logic variables
  private int iterationCounter = 0;

  public MoveToNewPositionCmd(CatzMechanismPosition newPosition) {
    m_newPosition = newPosition;
    // addRequirements(intake);
  }

  @Override
  public void initialize() {
    System.out.println("new mechanism set cmd");
    // intake.updateIntakeTargetPosition(m_newPosition.getIntakePivotTargetAngle());
    // elevator.updateElevatorTargetRev(m_newPosition.getElevatorTargetRev());
  }

  @Override
  public void execute() {
      iterationCounter++;
    if(m_newPosition == CatzConstants.CatzMechanismConstants.NOTE_POS_HANDOFF && iterationCounter == 150) {
      // intake.setRollerState(2);
      // shooter.updateLoadState(1);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
