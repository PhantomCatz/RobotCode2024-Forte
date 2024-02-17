// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.RobotContainer;
import frc.robot.CatzConstants.ShooterConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.Utils.LoggedTunableNumber;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;


public class SubsystemCatzShooter extends SubsystemBase {
  
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static SubsystemCatzShooter instance = new SubsystemCatzShooter();
  //Pivot constants and variables
  private CatzMechanismPosition m_targetPosition;


  public SubsystemCatzShooter() {

    switch (CatzConstants.currentMode) {
      case REAL: io = new ShooterIOReal();
                 System.out.println("Shooter Configured for Real");
      break;

      case REPLAY: io = new ShooterIOReal() {};
                   System.out.println("Shooter Configured for Replayed Simulation");
      break;

      case SIM:
      default: io = null;
               System.out.println("Current Mode Unconfigured");
      break;
    }
  }



  // Get the singleton instance of the ShooterSubsystem
  public static SubsystemCatzShooter getInstance() {
      return instance;
  }



  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/shooterinputs ", inputs);

    if(DriverStation.isDisabled()) {
      io.loadDisabled();
      io.setShooterDisabled();
    } else {
    }
  }

  //-------------------------------------------Flywheel and Feed methods------------------------------------------

  public Command cmdShooterEnabled() {
    return run(()->shooterEnabled());
  }

  public Command cmdShooterDisabled() {
    return run(()->shooterDisabled());
  }

  public void shooterEnabled() {
    io.setShooterEnabled();
    io.feedForward();
  }

  public void shooterDisabled() {
    io.setShooterDisabled();
    io.feedDisabled();
  }

  public Command setFeedMotor() {
    return run(()->io.feedForward());
  }

  public Command setFeedMotorDisabled() {
    return run(()->io.setShooterDisabled());
  }

  //-------------------------------------------Load methods------------------------------------------
  
  public Command setLoadReverse() {
      return run(()->io.loadReverse());
  }

  public Command shootNote() {
    return run(()->io.loadForward());
  }
}
