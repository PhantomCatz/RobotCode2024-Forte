// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.turret.SubsystemCatzTurret;

public class ScoreTrapCmd extends Command {
  private SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  private SubsystemCatzShooter shooter = SubsystemCatzShooter.getInstance();
  private SubsystemCatzTurret turret = SubsystemCatzTurret.getInstance();
  private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
  /** Creates a new ScoreTrapCmd. */
  public ScoreTrapCmd() {
    addRequirements(intake, elevator, shooter, turret);
  }

  @Override
  public void initialize() {
    runMechanismSetpoints(CatzMechanismConstants.SCORING_TRAP_PRESET);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  //factory for updating all mechanisms with the packaged target info associated with the new postion
  private void runMechanismSetpoints(CatzMechanismPosition pose) {
    intake.updateAutoTargetPositionIntake(pose.getIntakePivotTargetAngle());
    elevator.updateTargetPositionElevator(pose.getElevatorTargetRev());
    turret.updateTargetPositionTurret(pose);
  }
}
