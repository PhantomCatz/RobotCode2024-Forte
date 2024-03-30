// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.CatzConstants.NoteDestination;
import frc.robot.CatzConstants.NoteSource;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterState;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterNoteState;
import frc.robot.subsystems.turret.SubsystemCatzTurret;

public class MoveToPresetHandoffCmd extends Command {
  
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

  private Timer transferToShooter  = new Timer();


  public MoveToPresetHandoffCmd(NoteDestination noteDestination, NoteSource noteSource) {
    this.m_noteDestination = noteDestination;
    this.m_noteSource = noteSource;

    addRequirements(intake, elevator, turret, shooter);
  }

  @Override
  public void initialize() {
    intake.setWasIntakeInAmpScoring(false); // flag for determining whether to move to a transition state during sequencing

    // System.out.println("Handoff " + m_noteDestination.toString());
    // System.out.println(m_noteSource.toString());
    m_targetMechPoseStartReached = false;
    m_targetMechPoseEndReached   = false;

    switch(m_noteSource) {
      case INTAKE_GROUND:
        m_targetMechPoseStart = CatzMechanismConstants.INTAKE_GROUND_PRESET;
        intake.setRollersIn();

        if(m_noteDestination == NoteDestination.HOARD ||
           m_noteDestination == NoteDestination.SPEAKER) {

            m_targetMechPoseEnd = CatzMechanismConstants.STOW_PRESET;
            // System.out.println("Ground speaker");
        } else if(m_noteDestination == NoteDestination.AMP)  {

            m_targetMechPoseEnd = CatzMechanismConstants.PREP_FOR_AMP_PRESET;
            // System.out.println("Ground AMP");
        }
      break;

      case INTAKE_SOURCE:
        m_targetMechPoseStart = CatzMechanismConstants.INTAKE_SOURCE_PRESET;

        if(m_noteDestination == NoteDestination.HOARD ||
           m_noteDestination == NoteDestination.SPEAKER) {

            m_targetMechPoseEnd = CatzMechanismConstants.STOW_PRESET;
            intake.setRollersIntakeSource();
            // System.out.println(" Source Speaker");
        } else if(m_noteDestination == NoteDestination.AMP) {
          m_targetMechPoseEnd = m_targetMechPoseStart;
                        // System.out.println("Source Amp");
        }      
      break;

      case FROM_INTAKE:
        m_targetMechPoseStart = CatzMechanismConstants.STOW_PRESET;

        if(m_noteDestination == NoteDestination.HOARD ||
           m_noteDestination == NoteDestination.SPEAKER) {
            // System.out.println("Intake Speaker");

          m_targetMechPoseEnd = CatzMechanismConstants.STOW_PRESET;
        } else if(m_noteDestination == NoteDestination.AMP) {
            // System.out.println("Intake Amp");
          m_targetMechPoseEnd = CatzMechanismConstants.PREP_FOR_AMP_PRESET;
        }
      
      break;

      case FROM_SHOOTER:
        m_targetMechPoseStart = CatzMechanismConstants.STOW_PRESET;

        if(m_noteDestination == NoteDestination.AMP) {
            m_targetMechPoseEnd = CatzMechanismConstants.PREP_FOR_AMP_PRESET;
            // System.out.println("Shooter Amp");

        } 
     
      break;
        
      default: 
        //invalid command...should have used switch handoff positions cmd
       // m_targetMechPoseStart = CatzMechanismConstants.HOME;
      break;
    }

    runMechanismSetpoints(m_targetMechPoseStart);    //run initial sepoint
  }

  
  @Override
  public void execute() {
    boolean mechInPos = false;
    areMechanismsInPosition();

    if(m_noteSource == NoteSource.INTAKE_GROUND ||
       m_noteSource == NoteSource.INTAKE_SOURCE) {

      //when the the rollers stop intaking due to beambreak
      if(intake.getIntakeBeamBreakBroken()) {
        if(m_targetMechPoseStartReached == false) { 
          
          runMechanismSetpoints(m_targetMechPoseEnd);

          if(m_noteDestination == NoteDestination.SPEAKER) {
            shooter.setShooterState(ShooterState.LOAD_IN);
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
             intake.setRollersOutakeHandoff();
            //System.out.print("Outtaking");
            if(shooter.getShooterNoteState() == ShooterNoteState.NOTE_IN_POSTION) {
            //System.out.print("Note in position");
              intake.setRollersOff();
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
          intake.setRollersIntakeSource();
          shooter.setShooterState(ShooterState.LOAD_OUT);
          m_targetMechPoseStartReached = true;
        }
      }

      //when the mechanisms have all reached their end position after collecting 
      if(m_targetMechPoseStartReached && 
         m_targetMechPoseEndReached == false) {

        if(intake.getIntakeBeamBreakBroken()) { 

          intake.setRollersOff(); 

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
            intake.setRollersOutakeHandoff();
          shooter.setShooterState(ShooterState.LOAD_IN);
          m_targetMechPoseStartReached = true;
        }
      }

      //when the mechanisms have all reached their end position after collecting 
      if(m_targetMechPoseStartReached && 
         m_targetMechPoseEndReached == false) {

        if(shooter.shooterLoadBeamBrkBroken()) { 
          intake.setRollersOff();  
        }
      } 
    }


  }

  //factory for updating all mechanisms with the packaged target info associated with the new postion
  private void runMechanismSetpoints(CatzMechanismPosition pose) {

    intake  .updateAutoTargetPositionIntake(pose.getIntakePivotTargetAngle());
    elevator.updateTargetPositionElevator  (pose.getElevatorTargetRev());
    shooter .updateTargetPositionShooter   (pose);
    turret  .updateTargetPositionTurret    (pose);
  }

  private boolean areMechanismsInPosition() {
    boolean intakeState   = intake.getIntakeInPos(); 
    boolean turretState   = turret.getTurretInPos();
    boolean elevatorState = elevator.getElevatorInPos();
    // System.out.println("i " + intakeState + "t " + turretState + "e " + elevatorState);
    return(intakeState && turretState && elevatorState);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_targetMechPoseEndReached;
  }
}
