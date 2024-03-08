// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.RobotContainer.NoteDestination;
import frc.robot.RobotContainer.NoteSource;
import frc.robot.Robot.manipulatorMode;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.elevator.SubsystemCatzElevator.ElevatorState;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.intake.SubsystemCatzIntake.IntakeRollerState;
import frc.robot.subsystems.intake.SubsystemCatzIntake.IntakeState;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterLoadState;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterNoteState;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterServoState;
import frc.robot.subsystems.turret.SubsystemCatzTurret;
import frc.robot.subsystems.turret.SubsystemCatzTurret.TurretState;

public class IntakeMoveToHandoffPoseCmd extends Command {
  
  //subsystem declaration
  private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
  private SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  private SubsystemCatzShooter shooter = SubsystemCatzShooter.getInstance();
  private SubsystemCatzTurret turret = SubsystemCatzTurret.getInstance();

  //target Pose declaration
  private CatzMechanismPosition m_targetRobotPoseStart;
  private CatzMechanismPosition m_targetRobotPoseEnd;

  //destination targets
  private NoteDestination m_noteDestination;
  private NoteSource m_noteSource;

  /** This cmd should not be used for transfering handoff positions */
  public IntakeMoveToHandoffPoseCmd(NoteDestination noteDestination, NoteSource noteSource) {
    this.m_noteDestination = noteDestination;
    this.m_noteSource = noteSource;

    addRequirements(intake, elevator, turret, shooter);
  }

  @Override
  public void initialize() {

    switch(m_noteDestination) {
      case SPEAKER: 
        m_targetRobotPoseEnd = CatzMechanismConstants.HANDOFF_SHOOTER;
      break;

      case HOARD:
      case TRAP:
      case AMP:
      default: //amp
        m_targetRobotPoseEnd = CatzMechanismConstants.HANDOFF_AMP_PREP;
      break;
    }

    switch(m_noteSource) {
      case INTAKE_GROUND:
        m_targetRobotPoseStart = CatzMechanismConstants.INTAKE_GROUND;
        intake.setRollerState(IntakeRollerState.ROLLERS_IN);
      break;

      case INTAKE_SOURCE:
        m_targetRobotPoseStart = CatzMechanismConstants.INTAKE_SOURCE;
        intake.setRollerState(IntakeRollerState.ROLLERS_IN);
      break;

      default: 
        //invalid command...should've used switch handoff positions cmd
        m_targetRobotPoseStart = CatzMechanismConstants.POS_STOW;
      break;
    }
    //run setpoint 1
    runMechanismSetpoints(m_targetRobotPoseStart);
  }
  @Override
  public void execute() {
    if(m_noteSource == NoteSource.INTAKE_GROUND ||
       m_noteSource == NoteSource.INTAKE_SOURCE) {
      //when the the rollers stop intaking due to beambreak
      if(intake.getIntakeBeamBreakBroken()) {
        runMechanismSetpoints(m_targetRobotPoseEnd);
        //run load motors in when the note destination is speaker
        if(m_noteDestination == NoteDestination.SPEAKER) {
          shooter.setShooterLoadState(ShooterLoadState.LOAD_IN);
        } else {
          //keep note in intake if destination is anything else
        }
      }
      //when the mechanisms have all reached their end position after collecting 
      if(areMechanismsInPosition()) {
        if(m_noteDestination == NoteDestination.SPEAKER) {
          //set intake rollers to eject to handoff
          intake.setRollerState(IntakeRollerState.ROLLERS_OUT_HANDOFF_EJECT);
          //set rollers off once note is in place
          if(shooter.getShooterNoteState() == ShooterNoteState.NOTE_IN_POSTION) {
            intake.setRollerState(IntakeRollerState.ROLLERS_OFF);
          }
        } else {
          //keep note in intake
        }
      }
     }
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

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
