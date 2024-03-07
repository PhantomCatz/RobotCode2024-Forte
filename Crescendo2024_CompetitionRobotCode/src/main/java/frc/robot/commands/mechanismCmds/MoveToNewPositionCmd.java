// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.ManipulatorMode;
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


  private CatzMechanismPosition m_newPosition;
  private Supplier<ManipulatorMode> supplierManipulatorMode;
  private ManipulatorMode m_manipulatorMode;
  private ManipulatorMode       m_previousManipulatorMode;

  //logic variables
  private static int iterationCounter = 0;

  public MoveToNewPositionCmd(CatzMechanismPosition newPosition, Supplier<ManipulatorMode> newManipulatorMode) {
    supplierManipulatorMode = newManipulatorMode;
    m_newPosition = newPosition;


    addRequirements(intake, elevator, turret, shooter);
  }

  @Override
  public void initialize() {
    //if in speaker mode...run all the transformations of the new catzposition into a speaker config if applicable
    if(supplierManipulatorMode.get() == ManipulatorMode.SPEAKER) {
      if(m_newPosition == CatzMechanismConstants.NOTE_SCORING_AMP) {
       // m_newPosition = null;
      }
    } 
    runMechanismSetpoints();
  }

  
  @Override
  public void execute() {
  m_manipulatorMode = supplierManipulatorMode.get();
    if(m_previousManipulatorMode != m_manipulatorMode) {
      //if in speaker mode...run all the transformations of the new catzposition into a speaker config if applicable
    if(supplierManipulatorMode.get() == ManipulatorMode.SPEAKER) {
      if(m_newPosition == CatzMechanismConstants.NOTE_SCORING_AMP) {
        //m_newPosition = null;
      }
    } 
      runMechanismSetpoints();
    }


    m_previousManipulatorMode = m_manipulatorMode;
  }

  //factory for updating all mechanisms with the packaged target info associated with the new postion
  private void runMechanismSetpoints() {
    intake.updateIntakeTargetPosition(m_newPosition);
    elevator.updateElevatorTargetPosition(m_newPosition);
    shooter.updateShooterTargetPosition(m_newPosition);
    turret.updateTurretTargetPosition(m_newPosition);
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
