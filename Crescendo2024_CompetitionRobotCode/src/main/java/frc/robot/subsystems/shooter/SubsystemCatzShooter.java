// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.sql.Driver;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;


public class SubsystemCatzShooter extends SubsystemBase {
  
  //shooter io delcaration block
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  //shooter instatntation
  private static SubsystemCatzShooter instance = new SubsystemCatzShooter();

  //shooter constants and variables
  private static final boolean BEAM_IS_BROKEN  = true;
  private static final boolean BEAM_IS_NOT_BROKEN = false;

  private boolean desiredFrontBeamBreakState;
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

  private static ServoState currentServoState;
  private static enum ServoState {
    AUTO,
    FULL_EXTEND,
    FULL_RETRACT
  }

  private static FlywheelEnable currentFlywheelEnableMode;
  private static enum FlywheelEnable {
    ON,
    OFF
  }

  private static LoaderState currentLoaderState = LoaderState.LOAD_OFF;
  private static enum LoaderState {
    LOAD_OFF,
    LOAD_IN,
    LOAD_OUT,
    FINE_TUNE_CHECK,
    LOAD_IN_DONE,
    WAIT_FOR_NOTE_TO_SETTLE
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
      io.setFlywheelDisabled();
      io.loadDisabled();
      currentLoaderState = LoaderState.LOAD_OFF;
    } else {

        //shooter flywheel logic
        if(currentFlywheelEnableMode == FlywheelEnable.ON) {
          io.setFlywheelEnabled();
        } else {
          io.setFlywheelDisabled();
        }

        //servo logic states
        switch(currentServoState) {
          case AUTO:
          break;

          case FULL_EXTEND:
            io.setServoPosition(1.0);
          break;

          case FULL_RETRACT:
            io.setServoPosition(0.0);
          break;
        }
        
        
        //load logic
        switch(currentLoaderState) { 
          case LOAD_IN: //fire of the method call to make the load mtr run
            io.loadNote();
            currentLoaderState = LoaderState.LOAD_IN_DONE;
          break;

          case LOAD_IN_DONE: //disable and set off state to determine which direction to run load mtr
            if(inputs.shooterBackBeamBreakBroken == BEAM_IS_BROKEN) { 
              io.loadDisabled();
              currentLoaderState = LoaderState.WAIT_FOR_NOTE_TO_SETTLE;
              iterationCounter = 0;
            }
          break;

          case WAIT_FOR_NOTE_TO_SETTLE: //run timer and after determien which direction to run load mtr
            iterationCounter++;
            if(iterationCounter > 6) {
                if(inputs.isShooterFrontBeamBreakBroken == BEAM_IS_BROKEN) { //set off flag that determines adjust direction
                  desiredFrontBeamBreakState = BEAM_IS_NOT_BROKEN;
                  io.fineAdjustBck();
                } else {
                  desiredFrontBeamBreakState = BEAM_IS_BROKEN;
                  io.fineAdjustFwd();
                }
                  currentLoaderState = LoaderState.FINE_TUNE_CHECK;
              }
          break;

          case LOAD_OUT: //run the note out
            io.loadBackward();
          break;

          case FINE_TUNE_CHECK: //shooter checks to see if the beambreak is in desired state to turn off
            if(inputs.isShooterFrontBeamBreakBroken == desiredFrontBeamBreakState) { 
              io.loadDisabled();
              currentLoaderState = LoaderState.LOAD_OFF;
            }
          break;

          case LOAD_OFF: //run load mtr off
            io.loadDisabled();
          break;
      }

    }
    Logger.recordOutput("current load state", currentLoaderState.toString());
  }

  //-------------------------------------------Flywheel Commands------------------------------------------

  public Command cmdShooterEnabled() {
    return run(()->currentFlywheelEnableMode = FlywheelEnable.ON);
  }

  public Command cmdShooterDisabled() {
    return run(()->currentFlywheelEnableMode = FlywheelEnable.OFF);
  }

  //-------------------------------------------Load Commands------------------------------------------

  public Command loadFowardCmd() {
    return run(()->currentLoaderState = LoaderState.LOAD_IN);
  }

  public Command loadDisabled() {
    return run(()->currentLoaderState = LoaderState.LOAD_OFF);
  }
  public Command loadBackward() {
    return run(()->currentLoaderState = LoaderState.LOAD_OUT);
  }

  //-------------------------------------------Servo Commands------------------------------------------
  public Command setServoPowerExtend() {
    return run(()-> currentServoState = ServoState.FULL_EXTEND);
  }

  public Command setServoPowerRetract() {
    return run(()->currentServoState = ServoState.FULL_RETRACT);
  }


}
