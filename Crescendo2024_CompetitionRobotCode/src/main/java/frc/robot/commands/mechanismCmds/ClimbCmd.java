// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.climb.SubsystemCatzClimb;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.turret.SubsystemCatzTurret;

public class ClimbCmd extends Command {
  private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
  private SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  private SubsystemCatzShooter shooter = SubsystemCatzShooter.getInstance();
  private SubsystemCatzTurret turret = SubsystemCatzTurret.getInstance();  
  private SubsystemCatzClimb climb = SubsystemCatzClimb.getInstance();


  Supplier<Double> m_supplierXboxLeftY;
  Supplier<Double> m_supplierXboxRightY;


  
  public ClimbCmd(Supplier<Double> supplierXboxleftY, Supplier<Double> supplierXboxRightY) {
    m_supplierXboxLeftY = supplierXboxleftY;
    m_supplierXboxRightY = supplierXboxRightY;


    addRequirements(elevator, intake, shooter, turret, climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("in clmb");
    climb.setClimbModeEnabled(true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_supplierXboxLeftY.get()) > 0.1 ){
      climb.setLeftClimbPercentOutput(-m_supplierXboxLeftY.get()/2);
    }
    else{
      climb.setLeftClimbPercentOutput(0.0);
    }
    if(Math.abs(m_supplierXboxRightY.get()) > 0.1 ){
      climb.setRightClimbPercentOutput(m_supplierXboxRightY.get()/2);
    }
    else{
      climb.setRightClimbPercentOutput(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.setClimbModeEnabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //factory for updating all mechanisms with the packaged target info associated with the new postion
  private void runMechanismSetpoints(CatzMechanismPosition pose) {
    intake.updateAutoTargetPositionIntake(pose.getIntakePivotTargetAngle());
    elevator.updateTargetPositionElevator(pose);
    shooter.updateTargetPositionShooter(pose);
    turret.updateTargetPositionTurret(pose);
  }

  private boolean areMechanismsInPosition() {
    return (intake.getIntakeInPos() && 
            turret.getTurretInPos() &&
            shooter.getShooterServoInPos() &&
            elevator.getElevatorInPos());
  }
}
