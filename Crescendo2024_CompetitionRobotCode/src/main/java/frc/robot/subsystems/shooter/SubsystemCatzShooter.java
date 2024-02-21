// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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

  private static final int LOAD_OFF = 0;
  private static final int LOAD_IN = 1;
  private static final int LOAD_OUT = 2;
  private static final int FINE_TUNE = 3;
  private static final int LOAD_IN_DONE = 4;
  private static final int WAIT_FOR_NOTE_TO_SETTLE = 5;

  private static final boolean BEAM_IS_BROKEN  = true;
  private static final boolean BEAM_IS_NOT_BROKEN = false;

  private boolean desiredBeamBreakState;

  private int iterationCounter;

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

      switch(currentLoaderMode) { 
        case LOAD_IN:
          io.loadNote();
          currentLoaderMode = LOAD_IN_DONE;
        break;

        case LOAD_IN_DONE:
          if(inputs.shooterBackBeamBreakBroken == BEAM_IS_BROKEN) { 
            io.loadDisabled();
            currentLoaderMode = WAIT_FOR_NOTE_TO_SETTLE;
            iterationCounter = 0;
          }
        break;

        case WAIT_FOR_NOTE_TO_SETTLE:
          iterationCounter++;
          if(iterationCounter > 6) {
              if(inputs.isShooterFrontBeamBreakBroken == BEAM_IS_BROKEN) { //set off flag that determines adjust direction
                desiredBeamBreakState = BEAM_IS_NOT_BROKEN;
                io.fineAdjustBck();
              } else {
                desiredBeamBreakState = BEAM_IS_BROKEN;
                io.fineAdjustFwd();
              }
                currentLoaderMode = FINE_TUNE;
          }
            
        break;

        case LOAD_OUT: 
          io.loadBackward();
        break;

        case FINE_TUNE:

          if(inputs.isShooterFrontBeamBreakBroken == desiredBeamBreakState) { //if front is still conncected adjust foward until it breaks
            io.loadDisabled();
            currentLoaderMode = LOAD_OFF;
          }

        break;

        case LOAD_OFF:
          io.loadDisabled();

        break;
    }
    Logger.recordOutput("current load state", currentLoaderMode);
  }

  //-------------------------------------------Flywheel Commands------------------------------------------

  public Command cmdShooterEnabled() {
    return run(()->io.setShooterEnabled());
  }

  public Command cmdShooterDisabled() {
    return run(()->io.setShooterDisabled());
  }

  //-------------------------------------------Load Commands------------------------------------------

  public Command loadFowardCmd() {
    return runOnce(()->currentLoaderMode = 1);
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
