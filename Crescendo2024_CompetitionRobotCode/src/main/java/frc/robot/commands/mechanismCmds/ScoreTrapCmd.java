// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.climb.SubsystemCatzClimb;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.elevator.SubsystemCatzElevator.ElevatorState;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.intake.SubsystemCatzIntake.IntakeState;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterServoState;
import frc.robot.subsystems.turret.SubsystemCatzTurret;
import frc.robot.subsystems.turret.SubsystemCatzTurret.TurretState;

public class ScoreTrapCmd extends Command {
  private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
  private SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  private SubsystemCatzShooter shooter = SubsystemCatzShooter.getInstance();
  private SubsystemCatzTurret turret = SubsystemCatzTurret.getInstance();  
  private SubsystemCatzClimb climb = SubsystemCatzClimb.getInstance();


  Supplier<Boolean> m_supplierPovUP;
  Supplier<Boolean> m_supplierPovDN;
  Supplier<Boolean> m_supplierPovLT;
  Supplier<Boolean> m_supplierPovRT;

  
  public ScoreTrapCmd(Supplier<Boolean> PovUp, 
                      Supplier<Boolean> PovDn, 
                      Supplier<Boolean> PovLt, 
                      Supplier<Boolean> PovRt) {
    m_supplierPovUP = PovUp;
    m_supplierPovDN = PovDn;
    m_supplierPovLT = PovLt;
    m_supplierPovRT = PovRt;

    addRequirements(elevator, intake, shooter, turret, climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    runMechanismSetpoints(CatzMechanismConstants.POS_STOW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_supplierPovUP.get()) {
      climb.setLeftClimbPercentOutput(0.2);
      climb.setRightClimbPercentOutput(0.2);
    } else if(m_supplierPovDN.get()) {
      climb.setLeftClimbPercentOutput(-0.2);
      climb.setRightClimbPercentOutput(-0.2);
    } else if(m_supplierPovLT.get()) {
      climb.setLeftClimbPercentOutput(0.2);
    } else if(m_supplierPovRT.get()) {
      climb.setRightClimbPercentOutput(0.2);
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
    intake.updateTargetPositionIntake(pose);
    elevator.updateTargetPositionElevator(pose);
    shooter.updateTargetPositionShooter(pose);
    turret.updateTargetPositionTurret(pose);
  }

  private boolean areMechanismsInPosition() {
    return (intake.getIntakeState() == IntakeState.IN_POSITION && 
            turret.getTurretState() == TurretState.IN_POSITION &&
            shooter.getShooterServoState() == ShooterServoState.IN_POSITION &&
            elevator.getElevatorState() == ElevatorState.IN_POSITION);
  }
}
