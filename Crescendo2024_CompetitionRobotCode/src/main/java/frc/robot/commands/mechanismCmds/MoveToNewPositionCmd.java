// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.ManipulatorMode;
import frc.robot.Robot.manipulatorMode;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
 import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.turret.SubsystemCatzTurret;

public class MoveToNewPositionCmd extends Command {
  
  //subsystem declaration
  private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
  private SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  private SubsystemCatzShooter shooter = SubsystemCatzShooter.getInstance();
  private SubsystemCatzTurret turret = SubsystemCatzTurret.getInstance();


  private CatzMechanismPosition m_newPosition;
  private ManipulatorMode       m_manipulatorMode;
  private ManipulatorMode       m_previousManipulatorMode;

  //logic variables
  private static int iterationCounter = 0;

  public MoveToNewPositionCmd(CatzMechanismPosition newPosition, ManipulatorMode newManipulatorMode) {
    m_manipulatorMode = newManipulatorMode;
    m_newPosition = newPosition;


    addRequirements(intake, elevator, turret, shooter);
  }

  @Override
  public void initialize() {
    //if in speaker mode...run all the transformations of the new catzposition into a speaker config if applicable
    if(m_manipulatorMode == ManipulatorMode.SPEAKER) {
       
      if(m_newPosition == CatzMechanismConstants.NOTE_POS_HANDOFF_AMP_PREP) {
        m_newPosition = CatzMechanismConstants.NOTE_POS_HANDOFF_SPEAKER_PREP;
      }
    }
    runMechanismSetpoints();
  }

  
  @Override
  public void execute() {
    if(m_previousManipulatorMode != m_manipulatorMode) {
      //if in speaker mode...run all the transformations of the new catzposition into a speaker config if applicable
      if(m_manipulatorMode == ManipulatorMode.SPEAKER) {
        
        if(m_newPosition == CatzMechanismConstants.NOTE_POS_HANDOFF_AMP_PREP) {
          m_newPosition = CatzMechanismConstants.NOTE_POS_HANDOFF_SPEAKER_PREP;
        }
      }
      runMechanismSetpoints();
    }


    m_previousManipulatorMode = m_manipulatorMode;
  }

  //factory for updating all mechanisms with the packaged target info associated with the new postion
  public void runMechanismSetpoints() {
    intake.updateIntakeTargetPosition(m_newPosition);
    elevator.updateElevatorTargetPosition(m_newPosition);
    shooter.updateShooterTargetPosition(m_newPosition);
    turret.updateTurretTargetPosition(m_newPosition);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
