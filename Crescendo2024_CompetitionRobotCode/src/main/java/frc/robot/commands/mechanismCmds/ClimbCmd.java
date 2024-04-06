// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.RobotMode;
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
    climb.setClimbModeEnabled(true);
    turret.cmdTurretDegree(SubsystemCatzTurret.HOME_POSITION_DEG);
    shooter.setServoManualHold(SubsystemCatzShooter.SERVO_MAX_POS);
    intake.updateAutoTargetPositionIntake(SubsystemCatzIntake.INTAKE_AMP_SCORE_DN_DEG);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Left Climb hook control
    if(Math.abs(m_supplierXboxLeftY.get()) > 0.1 ){
      if(CatzConstants.currentRobotMode == RobotMode.CLIMB) {
        climb.setLeftClimbPercentOutput(-Math.abs(m_supplierXboxLeftY.get()/2));
      } else {
        climb.setLeftClimbPercentOutput(-m_supplierXboxLeftY.get()/2);
      }
    }
    else{
      climb.setLeftClimbPercentOutput(0.0);
    }

    //Right Climb Hook Control
    if(Math.abs(m_supplierXboxRightY.get()) > 0.1 ){
      if(CatzConstants.currentRobotMode == RobotMode.CLIMB) {
        climb.setRightClimbPercentOutput(-Math.abs(m_supplierXboxRightY.get()/2));
      } else {
        climb.setRightClimbPercentOutput(-m_supplierXboxRightY.get()/2);
      }
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

}
