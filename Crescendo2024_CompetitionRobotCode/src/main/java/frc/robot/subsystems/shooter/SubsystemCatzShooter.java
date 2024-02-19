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
  //shooter constants and variables
  private static int currentLoaderMode;
  //private CatzMechanismPosition m_targetPosition;

  private static final int SHOOTER_OFF = 0;
  private static final int LOAD_IN = 1;
  private static final int LOAD_OUT = 2;
  private static final int LOAD_OUT_WITH_BEAMBREAK_CHECK = 3;
  private static final int LOAD_IN_WITH_BRAMBREAK_CHECK = 4;



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

    //if we are shooting in...perform the state maneuver
    if(currentLoaderMode == LOAD_IN) { 
        //if both beam breaks broken go into s
        if(inputs.shooterBackBeamBreakBroken && inputs.shooterFrontBeamBreakBroken) {
          currentLoaderMode = LOAD_OUT_WITH_BEAMBREAK_CHECK; //go to reverse with beambreak check state
        } else if(inputs.shooterBackBeamBreakBroken) { 
          currentLoaderMode = LOAD_IN_WITH_BRAMBREAK_CHECK;
        }
    }


    if(DriverStation.isDisabled()) {
      io.loadDisabled();
      io.setShooterDisabled();
    } else {
      if(currentLoaderMode == LOAD_IN) {
        io.loadForward();
      } else if(currentLoaderMode == LOAD_OUT) {
        io.loadBackward();  
      } else if(currentLoaderMode == LOAD_OUT_WITH_BEAMBREAK_CHECK) {
        if(!inputs.shooterFrontBeamBreakBroken) {
          io.loadDisabled();
          currentLoaderMode = 0; 
        } else {
          io.loadBackward();
        }
      } else if(currentLoaderMode == LOAD_IN_WITH_BRAMBREAK_CHECK) {
        if(inputs.shooterFrontBeamBreakBroken) {
          io.loadDisabled();
          currentLoaderMode = 0;
        } else {
          io.loadForward();
        }
      } else {
        io.loadDisabled();
      }
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

  public Command loadFoward() {
    return run(()->currentLoaderMode = 1);
  }

  public Command loadDisabled() {
    return run(()->currentLoaderMode = 0);
  }
  public Command loadBackward() {
    return run(()->currentLoaderMode = 2);
  }

  //-------------------------------------------Servo Commands------------------------------------------
  public Command setServoPowerExtend() {
    return run(()->io.setServoPower(1));
  }

  public Command setServoPowerRetract() {
    return run(()->io.setServoPower(0.0));
  }


}
