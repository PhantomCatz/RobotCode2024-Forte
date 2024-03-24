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

public class MoveToAmpTransition extends Command {
    //subsystem declaration
  private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
  private SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  private SubsystemCatzShooter shooter = SubsystemCatzShooter.getInstance();
  private SubsystemCatzTurret turret = SubsystemCatzTurret.getInstance();
  /** Creates a new MoveToAmpTransition. */
  public MoveToAmpTransition() {
    addRequirements(intake, elevator, turret, shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    runMechanismSetpoints(CatzMechanismConstants.AMP_TRANSITION_PRESET);    //run initial sepoint

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

  //factory for updating all mechanisms with the packaged target info associated with the new postion
  private void runMechanismSetpoints(CatzMechanismPosition pose) {

    intake  .updateAutoTargetPositionIntake(pose.getIntakePivotTargetAngle());
    elevator.updateTargetPositionElevator(pose.getElevatorTargetRev());
    shooter .updateTargetPositionShooter (pose);
    turret  .updateTargetPositionTurret  (pose);
  }

  private boolean areMechanismsInPosition() {
    boolean intakeState   = intake.getIntakeInPos(); 
    boolean turretState   = turret.getTurretInPos();
    boolean shooterState  = shooter.getShooterServoInPos();
    boolean elevatorState = elevator.getElevatorInPos();
    // System.out.println("i " + intakeState + "t " + turretState + "s " + shooterState + "e " +elevatorState);
    return(intakeState && turretState && shooterState && elevatorState);
  }
}
