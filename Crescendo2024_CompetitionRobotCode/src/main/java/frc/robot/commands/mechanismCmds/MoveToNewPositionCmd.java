// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.NoteDestination;
import frc.robot.CatzConstants.NoteSource;
import frc.robot.Robot.manipulatorMode;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.elevator.SubsystemCatzElevator.ElevatorState;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.intake.SubsystemCatzIntake.IntakeState;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterServoState;
import frc.robot.subsystems.turret.SubsystemCatzTurret;
import frc.robot.subsystems.turret.SubsystemCatzTurret.TurretState;

public class MoveToNewPositionCmd extends Command {
  
  //subsystem declaration
  private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
  private SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  private SubsystemCatzShooter shooter = SubsystemCatzShooter.getInstance();
  private SubsystemCatzTurret turret = SubsystemCatzTurret.getInstance();


  private CatzMechanismPosition m_targetRobotPose;
  private Supplier<NoteDestination> m_supplierNoteDestination;
  private NoteDestination m_manipulatorMode;
  private NoteDestination       m_previousManipulatorMode;

  private Supplier<NoteSource> m_noteSource;

  private 

  //logic variables
  private static int iterationCounter = 0;

  public MoveToNewPositionCmd(Supplier<NoteDestination> noteDestination) {
    m_supplierNoteDestination = targetRobotPose;
    m_targetRobotPose = targetRobotPose;


    addRequirements(intake, elevator, turret, shooter);
  }
  public MoveToNewPositionCmd(Supplier<NoteSource> noteSource) {
    m_noteSource = noteSource;
    m_targetRobotPose = targetRobotPose;


    addRequirements(intake, elevator, turret, shooter);
  }

  @Override
  public void initialize() {
    //if in speaker mode...run all the transformations of the new catzposition into a speaker config if applicable
    if(m_supplierNoteDestination.get() == NoteDestination.SPEAKER) {
      if(m_targetRobotPose == CatzMechanismConstants.NOTE_SCORING_AMP) {
      }
    } 

    switch(NoteDestination) {
      case SPEAKER: 
      m_targetRobotPoseEnd   = CatzMechanismConstants.NOTE_POS_SHOOTER_HANDOFF;
      if(m_noteSource.get() == NoteSource.INTAKE_SOURCE) {
        m_targetRobotPoseStart = CatzMechanismConstants.NOTE_POS_INTAKE_SOURCE;
      } else { //default to ground
        m_targetRobotPoseStart = CatzMechanismConstants.NOTE_POS_INTAKE_GROUND;
      }

    }

    runMechanismSetpoints();
  }

  
  @Override
  public void execute() {
  m_manipulatorMode = m_supplierNoteDestination.get();
    if(m_previousManipulatorMode != m_manipulatorMode) {
      //if in speaker mode...run all the transformations of the new catzposition into a speaker config if applicable
    if(m_supplierNoteDestination.get() == NoteDestination.SPEAKER) {
      if(m_targetRobotPose == CatzMechanismConstants.NOTE_SCORING_AMP) {
        //m_newPosition = null;
      }
    } 
      runMechanismSetpoints();
    }


    m_previousManipulatorMode = m_manipulatorMode;
  }

  //factory for updating all mechanisms with the packaged target info associated with the new postion
  private void runMechanismSetpoints() {
    intake.updateIntakeTargetPosition(m_targetRobotPose);
    elevator.updateElevatorTargetPosition(m_targetRobotPose);
    shooter.updateShooterTargetPosition(m_targetRobotPose);
    turret.updateTurretTargetPosition(m_targetRobotPose);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return (turret.getTurretState()        == TurretState.IN_POSITION &&
            shooter.getShooterServoState() == ShooterServoState.IN_POSITION &&
            elevator.getElevatorState()    == ElevatorState.IN_POSITION &&
            intake.getIntakeState()        == IntakeState.IN_POSITION);
  }
}
