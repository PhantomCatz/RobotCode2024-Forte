// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.elevator.SubsystemCatzElevator.ElevatorState;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.intake.SubsystemCatzIntake.IntakeRollerState;
import frc.robot.subsystems.intake.SubsystemCatzIntake.IntakeState;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterServoState;
import frc.robot.subsystems.turret.SubsystemCatzTurret;
import frc.robot.subsystems.turret.SubsystemCatzTurret.TurretState;

public class ScoreAmpCmd extends Command {
  private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
  private SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  private SubsystemCatzShooter shooter = SubsystemCatzShooter.getInstance();
  private SubsystemCatzTurret turret = SubsystemCatzTurret.getInstance();

  private static Timer intakeNoteTimer = new Timer();

  public ScoreAmpCmd() {
    addRequirements(intake, elevator, shooter, turret);
  }

  @Override
  public void initialize() {
    runMechanismSetpoints(CatzMechanismConstants.SCORING_AMP);
    intakeNoteTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(areMechanismsInPosition()) {
      intake.setRollerState(IntakeRollerState.ROLLERS_OUT_FULL_EJECT);
      intakeNoteTimer.start();
    }

    if(intakeNoteTimer.hasElapsed(2) && intake.getIntakeBeamBreakBroken()) {
      runMechanismSetpoints(CatzMechanismConstants.POS_STOW);
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

  //factory for updating all mechanisms with the packaged target info associated with the new postion
  private void runMechanismSetpoints(CatzMechanismPosition pose) {
    intake.updateIntakeTargetPosition(pose);
    elevator.updateElevatorTargetPosition(pose);
    shooter.updateShooterTargetPosition(pose);
    turret.updateTurretTargetPosition(pose);
  }

  private boolean areMechanismsInPosition() {
    return (intake.getIntakeState() == IntakeState.IN_POSITION && 
            turret.getTurretState() == TurretState.IN_POSITION &&
            shooter.getShooterServoState() == ShooterServoState.IN_POSITION &&
            elevator.getElevatorState() == ElevatorState.IN_POSITION);
  }
}