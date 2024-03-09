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
  private CatzMechanismPosition m_targetMechPoseStart;
  private CatzMechanismPosition m_targetMechPoseEnd;


  //destination targets
  private NoteDestination m_noteDestination;
  private NoteSource m_noteSource;

  private boolean m_targetMechPoseStartReached = false;
  private boolean m_targetMechPoseEndReached   = false;


  /** This cmd should not be used for transfering handoff positions */
  public IntakeMoveToHandoffPoseCmd(NoteDestination noteDestination, NoteSource noteSource) {
    this.m_noteDestination = noteDestination;
    this.m_noteSource = noteSource;

    addRequirements(intake, elevator, turret, shooter);
  }

  @Override
  public void initialize() {
    m_targetMechPoseStartReached = false;
    m_targetMechPoseEndReached   = false;

    switch(m_noteSource) {
      case INTAKE_GROUND:
        m_targetMechPoseStart = CatzMechanismConstants.INTAKE_GROUND;

        if(m_noteDestination == NoteDestination.HOARD ||
           m_noteDestination == NoteDestination.SPEAKER) {

            m_targetMechPoseEnd = CatzMechanismConstants.HANDOFF_SHOOTER;
            intake.setRollerState(IntakeRollerState.ROLLERS_IN_GROUND);
        } else if(m_noteDestination == NoteDestination.AMP)  {
            m_targetMechPoseEnd = CatzMechanismConstants.POS_AMP_TRANSITION;
        }
      break;

      case INTAKE_SOURCE:
        m_targetMechPoseStart = CatzMechanismConstants.INTAKE_SOURCE;

        if(m_noteDestination == NoteDestination.HOARD ||
           m_noteDestination == NoteDestination.SPEAKER) {

            m_targetMechPoseEnd = CatzMechanismConstants.HANDOFF_SHOOTER;
            intake.setRollerState(IntakeRollerState.ROLLERS_IN_SOURCE);
        } else if(m_noteDestination == NoteDestination.AMP) {
          m_targetMechPoseEnd = m_targetMechPoseStart;
        }      
      break;

      case FROM_INTAKE:
        m_targetMechPoseStart = CatzMechanismConstants.HANDOFF_SHOOTER;

        if(m_noteDestination == NoteDestination.HOARD ||
           m_noteDestination == NoteDestination.SPEAKER) {

          m_targetMechPoseEnd = CatzMechanismConstants.HANDOFF_SHOOTER;
        } else if(m_noteDestination == NoteDestination.AMP) {

          m_targetMechPoseEnd = CatzMechanismConstants.POS_AMP_TRANSITION;
        }
      
      break;

      case FROM_SHOOTER:
        m_targetMechPoseStart = CatzMechanismConstants.HANDOFF_SHOOTER;

        if(m_noteDestination == NoteDestination.AMP) {
            m_targetMechPoseEnd = CatzMechanismConstants.POS_AMP_TRANSITION;
        } 
     
      break;
        
      default: 
        //invalid command...should have used switch handoff positions cmd
        m_targetMechPoseStart = CatzMechanismConstants.POS_STOW;
      break;
    }

    runMechanismSetpoints(m_targetMechPoseStart);    //run initial sepoint
  }

  
  @Override
  public void execute() {
  boolean mechInPos = false;

    if(m_noteSource == NoteSource.INTAKE_GROUND ||
       m_noteSource == NoteSource.INTAKE_SOURCE) {

      //when the the rollers stop intaking due to beambreak
      if(intake.getIntakeBeamBreakBroken()) {
        if(m_targetMechPoseStartReached == false) { 
          
          runMechanismSetpoints(m_targetMechPoseEnd);

          if(m_noteDestination == NoteDestination.SPEAKER) {
            shooter.setShooterLoadState(ShooterLoadState.LOAD_IN);
          }

          m_targetMechPoseStartReached = true; //reached start postion and start for end position
        }
      }

      //when the mechanisms have all reached their end position after collecting 
      if(m_targetMechPoseStartReached == true &&
         m_targetMechPoseEndReached   == false) {

        mechInPos = areMechanismsInPosition();
        if(mechInPos) {

          if(m_noteDestination == NoteDestination.SPEAKER) {
             intake.setRollerState(IntakeRollerState.ROLLERS_OUT_SHOOTER_HANDOFF);

            if(shooter.getShooterNoteState() == ShooterNoteState.NOTE_IN_POSTION) {
              intake.setRollerState(IntakeRollerState.ROLLERS_OFF);
              m_targetMechPoseEndReached = true;
            } 
          } else {
            //keep note in intake
          }
        }
      }
    } else if(m_noteSource == NoteSource.FROM_SHOOTER) {
      //when the the rollers stop intaking due to beambreak
      if(m_targetMechPoseStartReached == false) {
        if(areMechanismsInPosition()) {
          intake.setRollerState(IntakeRollerState.ROLLERS_IN_SOURCE);
          shooter.setShooterLoadState(ShooterLoadState.LOAD_OUT);
          m_targetMechPoseStartReached = true;
                        System.out.println("A");
        }
      }

      //when the mechanisms have all reached their end position after collecting 
      if(m_targetMechPoseStartReached && 
         m_targetMechPoseEndReached == false) {

        if(intake.getIntakeBeamBreakBroken()) { 

          intake.setRollerState(IntakeRollerState.ROLLERS_OFF);   

          if(m_noteDestination == NoteDestination.AMP) {
            runMechanismSetpoints(m_targetMechPoseEnd);
            m_targetMechPoseEndReached = true;
          } else {
            //keep note in intake
          }
        }
      } 
    } else if(m_noteSource == NoteSource.FROM_INTAKE) {
      //when the the rollers stop intaking due to beambreak
      if(m_targetMechPoseStartReached == false) {
        if(areMechanismsInPosition()) {
          intake.setRollerState(IntakeRollerState.ROLLERS_OUT_SHOOTER_HANDOFF);
          shooter.setShooterLoadState(ShooterLoadState.LOAD_IN);
          m_targetMechPoseStartReached = true;
        }
      }

      //when the mechanisms have all reached their end position after collecting 
      if(m_targetMechPoseStartReached && 
         m_targetMechPoseEndReached == false) {

        if(shooter.shooterLoadBeamBrkBroken()) { 
          intake.setRollerState(IntakeRollerState.ROLLERS_OFF);   
        }
      } 
    }


  }//ground to amp transition
  //amp transition to amp

  //

  //factory for updating all mechanisms with the packaged target info associated with the new postion
  private void runMechanismSetpoints(CatzMechanismPosition pose) {

    intake  .updateTargetPositionIntake  (pose);
    elevator.updateTargetPositionElevator(pose);
    shooter .updateTargetPositionShooter (pose);
    turret  .updateTargetPositionTurret  (pose);
  }

  private boolean areMechanismsInPosition() {
    boolean intakeState   = intake.getIntakeState()        == IntakeState.IN_POSITION; 
    boolean turretState   = turret.getTurretState()        == TurretState.IN_POSITION;
    boolean shooterState  = shooter.getShooterServoState() == ShooterServoState.IN_POSITION;
    boolean elevatorState = elevator.getElevatorState()    == ElevatorState.IN_POSITION;

    return(intakeState && turretState && shooterState && elevatorState);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
