// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;


public class SubsystemCatzShooter extends SubsystemBase {
  
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static SubsystemCatzShooter instance = new SubsystemCatzShooter();
  //Pivot constants and variables
  //private CatzMechanismPosition m_targetPosition;


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
    Logger.processInputs("Shooter/shooterinputs", inputs);

    if(DriverStation.isDisabled()) {
      io.loadDisabled();
      io.setShooterDisabled();
    }
  }

  //-------------------------------------------Flywheel Commands------------------------------------------

  public Command cmdShooterEnabled() {
    return run(()->io.setShooterEnabled());
  }

  public Command cmdShooterDisabled() {
    return run(()->io.setShooterDisabled());
  }

  //-------------------------------------------Load Commands------------------------------------------

  public Command shootNote() {
    return run(()->io.loadForward());
  }
  public Command loadBackward() {
    return run(()->io.loadBackward());
  }

  //-------------------------------------------Servo Commands------------------------------------------
  public Command setServoPowerExtend() {
    return run(()->io.setServoPower(1));
  }

  public Command setServoPowerRetract() {
    return run(()->io.setServoPower(0.0));
  }


}
