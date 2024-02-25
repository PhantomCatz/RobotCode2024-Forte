// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.Utils.LoggedTunableNumber;
import frc.robot.RobotContainer;


public class SubsystemCatzShooter extends SubsystemBase {
  
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static SubsystemCatzShooter instance = new SubsystemCatzShooter();
  
  //shooter constants and variables
  private static int currentLoaderMode;
      
  /*-----------------------------------------------------------------------------------------
   * Linear Servo Values
   *-----------------------------------------------------------------------------------------*/
  LoggedTunableNumber servoPos = new LoggedTunableNumber("ServoPos", 0);
    
  /*-----------------------------------------------------------------------------------------
   *
   * States and Variables for Periodic
   *  
   *-----------------------------------------------------------------------------------------*/
  private static final int LOAD_IN = 1;
  private static final int FINE_TUNE = 2;
  private static final int LOAD_IN_DONE = 3;
  private static final int WAIT_FOR_NOTE_TO_SETTLE = 4;
  private static final int WAIT_FOR_MOTORS_TO_REV_UP = 5;
  private static final int START_SHOOTER_FLYWHEEL = 6;
  private static final int SHOOTING = 7;
  private static final int LOAD_OFF = 8;
  private static final int LOAD_OUT = 9;

  private static final boolean BEAM_IS_BROKEN  = true;
  private static final boolean BEAM_IS_NOT_BROKEN = false;

  private double SHOOTER_DELAY_IN_SECONDS = 1;

  private boolean desiredBeamBreakState;

  private int iterationCounter;


  public SubsystemCatzShooter() {
    //Shooter delay calculation
    SHOOTER_DELAY_IN_SECONDS = Math.round( (SHOOTER_DELAY_IN_SECONDS / 0.02) + 1);

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

    double servoPosition = servoPos.get();
    System.out.println(servoPosition);
    io.setServoPosition(servoPosition);

      switch(currentLoaderMode) { 
        case LOAD_IN:
          io.loadNote();
          currentLoaderMode = LOAD_IN_DONE;
        break;

        case LOAD_IN_DONE:
          if(inputs.shooterLoadBeamBreakState == BEAM_IS_BROKEN) { 
            iterationCounter = 0;
            currentLoaderMode = WAIT_FOR_NOTE_TO_SETTLE;
            io.loadDisabled();
          }
        break;

        case WAIT_FOR_NOTE_TO_SETTLE:
          iterationCounter++;
          if(iterationCounter > 0) {
              if(inputs.shooterAdjustBeamBreakState == BEAM_IS_BROKEN) { //set off flag that determines adjust direction
                desiredBeamBreakState = BEAM_IS_NOT_BROKEN;
                io.fineAdjustBck();
              } else {
                desiredBeamBreakState = BEAM_IS_BROKEN;
                io.fineAdjustFwd();
              }
              currentLoaderMode = FINE_TUNE;
          }
        break;

        case FINE_TUNE:
          if(inputs.shooterAdjustBeamBreakState == desiredBeamBreakState) { //if front is still conncected adjust foward until it breaks
            io.loadDisabled();
            currentLoaderMode = LOAD_OFF;
          }
        break;
        
        case START_SHOOTER_FLYWHEEL:
          io.setShooterEnabled();
          currentLoaderMode = WAIT_FOR_MOTORS_TO_REV_UP;
        break;
        
        case WAIT_FOR_MOTORS_TO_REV_UP:
        //System.out.println(-inputs.shooterVelocityLT + " Lt sHOOTER " + inputs.velocityThresholdLT);
          if(-inputs.shooterVelocityLT >= inputs.velocityThresholdLT &&
              inputs.shooterVelocityRT >= inputs.velocityThresholdRT) {
              iterationCounter = 0;

              //RobotContainer.xboxDrv.setRumble()
          }
        break;

        case SHOOTING:
          io.feedShooter();
          iterationCounter++;
          //System.out.println("Iteration Counter " + iterationCounter);
          if(iterationCounter >= SHOOTER_DELAY_IN_SECONDS) {
              io.setShooterDisabled();
              currentLoaderMode = LOAD_OFF;
          }
        break;

        case LOAD_OFF:
          io.loadDisabled();
        break;

        case LOAD_OUT:
          io.loadBackward();
        break;
    }
    Logger.recordOutput("current load state", currentLoaderMode);
  }

  //-------------------------------------------Flywheel Commands------------------------------------------

  public Command cmdShooterEnabled() {
    return runOnce(()->currentLoaderMode = START_SHOOTER_FLYWHEEL);
  }

  public Command cmdShooterDisabled() {
    return runOnce(()->io.setShooterDisabled());
  }

  //-------------------------------------------Load Commands------------------------------------------

  public Command cmdShoot() {
      return runOnce(()->currentLoaderMode = SHOOTING);
  }

  public Command cmdLoad(){
    return runOnce(()->currentLoaderMode = LOAD_IN);
  }
  
  public Command loadBackward() {
    return runOnce(()->currentLoaderMode = 9);
  }
  
  public Command loadDisabled() {
    return runOnce(()->currentLoaderMode = 8);
  }

  //-------------------------------------------Servo Commands------------------------------------------

  public Command setPosition(double position) {
    System.out.println("aa");
    return run(()->io.setServoPosition(position));
  }
}
