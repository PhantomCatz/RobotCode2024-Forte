// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.OIConstants;


public class SubsystemCatzShooter extends SubsystemBase {
  
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static SubsystemCatzShooter instance = new SubsystemCatzShooter();
  //shooter constants and variables
  private static int currentLoaderMode;
  //private CatzMechanismPosition m_targetPosition;
      
  private Servo shooterLeftServo;
  private Servo shooterRightServo;

  private final int SERVO_LEFT_PWM_ID  = 19; //TBD change
  private final int SERVO_RIGHT_PWM_ID = 18;
    
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

  //Xbox controller to get what buttons were pressed
  public CommandXboxController xboxDrv;

  private int iterationCounter;

  public SubsystemCatzShooter() {

    xboxDrv = new CommandXboxController(OIConstants.XBOX_DRV_PORT);
  
    SHOOTER_DELAY_IN_SECONDS = Math.round( (SHOOTER_DELAY_IN_SECONDS / 0.02) + 1);

    shooterLeftServo = new Servo(SERVO_LEFT_PWM_ID);
    shooterRightServo = new Servo(SERVO_RIGHT_PWM_ID);

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
          if(inputs.shooterLoadBeamBreakState == BEAM_IS_BROKEN) { 
            iterationCounter = 0;
            currentLoaderMode = WAIT_FOR_NOTE_TO_SETTLE;
            io.loadDisabled();
          }
        break;

        case WAIT_FOR_NOTE_TO_SETTLE:
          iterationCounter++;
          if(iterationCounter > 5) {
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
          if(inputs.shooterVelocityLT >= inputs.velocityThresholdLT &&
             inputs.shooterVelocityRT >= inputs.velocityThresholdRT) {
              iterationCounter = 0;
              currentLoaderMode = SHOOTING;
              desiredBeamBreakState = BEAM_IS_NOT_BROKEN;
          }
        break;

        case SHOOTING:
          io.feedShooter();
          iterationCounter++;
          if(iterationCounter >= SHOOTER_DELAY_IN_SECONDS) {
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
    return run(()->io.setShooterEnabled());
  }

  public Command cmdShooterDisabled() {
    return run(()->io.setShooterDisabled());
  }

  //-------------------------------------------Load Commands------------------------------------------

  public Command loadFowardCmd() {
    return runOnce(()->currentLoaderMode = 1);
  }
  
  public Command loadBackward() {
    return run(()->currentLoaderMode = 9);
  }
  
  public Command loadDisabled() {
    return run(()->currentLoaderMode = 8);
  }

  public void updateLoadState(int newLoadState) {
    currentLoaderMode = newLoadState;
  }

  //-------------------------------------------Servo Commands------------------------------------------
  public Command setServoPowerExtend() {
    return run(()->io.setServoPosition(1));
  }

  public Command setServoPowerRetract() {
    return run(()->io.setServoPosition(0.0));
  }

}
