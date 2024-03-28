// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.sql.Driver;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.Utils.LoggedTunableNumber;
import frc.robot.commands.mechanismCmds.MoveToPreset;
import frc.robot.subsystems.turret.SubsystemCatzTurret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;


public class SubsystemCatzShooter extends SubsystemBase {
  
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static SubsystemCatzShooter instance = new SubsystemCatzShooter();

      
  /*-----------------------------------------------------------------------------------------
   * Linear Servo Values
   *-----------------------------------------------------------------------------------------*/
  LoggedTunableNumber servoPosTunning = new LoggedTunableNumber("ServoPos", 0);
  
  //Servo SetPositions
  public static final double SERVO_STOW_POS = 0.0;
  public static final double SERVO_OPTIMAL_HANDOFF_HIGH_POS = 0.4;
  public static final double SERVO_NULL_POSITION  = -999.0;


  private double m_newServoPosition;
  private double m_servoPosError;

  /*-----------------------------------------------------------------------------------------
   * Time Constants
   *-----------------------------------------------------------------------------------------*/
  private final double LOOP_CYCLE_SECONDS = 0.02;

  private final int WAIT_FOR_MOTORS_TO_REV_UP_TIMEOUT = (int) (Math.round(1.0/LOOP_CYCLE_SECONDS) + 1.0); 
  private final int SHOOTING_TIMEOUT                  = (int) (Math.round(0.5/LOOP_CYCLE_SECONDS) + 1.0);
  private final int LOAD_OUT_TIMEOUT                  = (int) (Math.round(0.5/LOOP_CYCLE_SECONDS) + 1.0);

  /*-----------------------------------------------------------------------------------------
   * States
   *-----------------------------------------------------------------------------------------*/

  private static ShooterState currentShooterState = ShooterState.NONE;
  public static enum ShooterState {
    LOAD_IN,
    FINE_TUNE,
    LOAD_IN_DONE,
    WAIT_FOR_NOTE_TO_SETTLE,
    WAIT_FOR_MOTORS_TO_REV_UP,
    START_SHOOTER_FLYWHEEL,
    SHOOTING,
    LOAD_OFF,
    LOAD_OUT,
    NONE;
  }

  private static ShooterServoState currentServoState;
  public static enum ShooterServoState {
    SET_POSITION,
    AUTO_AIM
  }

 
  //shooter note state for determining when other mechanism should turn off
  private ShooterNoteState currentNoteState;
  public enum ShooterNoteState {
    NOTE_IN_POSTION,
    NOTE_IN_ADJUST,
    NOTE_HAS_BEEN_SHOT,
    NULL
  }

  /*-----------------------------------------------------------------------------------------
  * Constants for Shooter
  *-----------------------------------------------------------------------------------------*/
  private static final boolean BEAM_IS_BROKEN     = true;
  private static final boolean BEAM_IS_NOT_BROKEN = false;

  private boolean m_desiredBeamBreakState;
  private int     m_iterationCounter;

  private boolean m_shooterServoInPos = false;
  private boolean autonKeepFlywheelOn = false;
  
  //XboxController for rumbling
  private XboxController xboxAuxRumble;

  private SubsystemCatzShooter() {
    
    //XboxController
    xboxAuxRumble = new XboxController(OIConstants.XBOX_AUX_PORT);

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

    if(DriverStation.isDisabled()) { //TBD this thing delayed the start of auton by more than a second 
      disableShooter();
    } else {
      switch(currentShooterState) {
          //-------------------------------------------------------------------------------------------
          //
          // feeder roller periodic logic
          //
          //-------------------------------------------------------------------------------------------
          case NONE:
          break;
          case LOAD_IN:
            io.loadNote();
            currentShooterState = ShooterState.LOAD_IN_DONE;
            currentNoteState = ShooterNoteState.NOTE_IN_ADJUST;
          break;

          case LOAD_IN_DONE:
            if(inputs.shooterLoadBeamBreakState == BEAM_IS_BROKEN) { 
              io.loadDisabled();
              m_iterationCounter = 0;
              currentShooterState = ShooterState.WAIT_FOR_NOTE_TO_SETTLE;
            }
          break;

          case WAIT_FOR_NOTE_TO_SETTLE:
            if(inputs.shooterAdjustBeamBreakState == BEAM_IS_BROKEN) { //set off flag that determines adjust direction
              m_desiredBeamBreakState = BEAM_IS_NOT_BROKEN;
              io.fineAdjustBck();
            } else {
              m_desiredBeamBreakState = BEAM_IS_BROKEN;
              io.fineAdjustFwd();
            }
                currentShooterState = ShooterState.FINE_TUNE;
          break;

          case FINE_TUNE:
            if(inputs.shooterAdjustBeamBreakState == m_desiredBeamBreakState) { //if front is still conncected adjust foward until it breaks
              io.loadDisabled();
              currentShooterState = ShooterState.LOAD_OFF;
              currentNoteState = ShooterNoteState.NOTE_IN_POSTION;
            }
          break;

          case LOAD_OFF:
            io.loadDisabled();
          break;

          case LOAD_OUT:
            io.loadBackward();
            
            if(m_iterationCounter >= LOAD_OUT_TIMEOUT) {
              io.loadDisabled();
              currentShooterState = ShooterState.LOAD_OFF;
              m_iterationCounter = 0;
            }
            m_iterationCounter++;
          break;
          
          case START_SHOOTER_FLYWHEEL:
            io.setShooterEnabled();
            currentShooterState = ShooterState.WAIT_FOR_MOTORS_TO_REV_UP;
          break;
          
          
          case WAIT_FOR_MOTORS_TO_REV_UP:
            if(inputs.shooterVelocityRT >= inputs.velocityThresholdRT &&
               inputs.shooterVelocityLT >= inputs.velocityThresholdLT) {

                  if(DriverStation.isAutonomous()) {
                      
                  } else {
                    xboxAuxRumble.setRumble(RumbleType.kBothRumble, 0.7);
                  }

            } else {

              if(DriverStation.isAutonomous()) {
                if(m_iterationCounter > WAIT_FOR_MOTORS_TO_REV_UP_TIMEOUT) {
                  if(!autonKeepFlywheelOn){
                    m_iterationCounter = 0;
                    currentShooterState = ShooterState.SHOOTING;
                  }
                }
                m_iterationCounter++;
              }
            }
            break;

          case SHOOTING:
            io.feedShooter();
            
            xboxAuxRumble.setRumble(RumbleType.kBothRumble, 0);

            if(m_iterationCounter >= SHOOTING_TIMEOUT) {
              if(DriverStation.isAutonomous()) {

                if(autonKeepFlywheelOn == true) {
                  currentShooterState = ShooterState.WAIT_FOR_MOTORS_TO_REV_UP;
;
                } else {
                  io.setShooterDisabled();
                  currentShooterState = ShooterState.LOAD_OFF;
                }

              }
              currentNoteState = ShooterNoteState.NOTE_HAS_BEEN_SHOT;
              SubsystemCatzTurret.getInstance().setTurretTargetDegree(0.0);

              // TBD Shouldn't need this anymore    updateShooterServo(0.0);
              io.setServoPosition(0.0);
            }
            
            m_iterationCounter++;
          break;

          default:
          break;
      }
  
      //-------------------------------------------------------------------------------------------
      //
      // servo periodic logic
      //
      //-------------------------------------------------------------------------------------------
      io.setServoPosition(m_newServoPosition);
      
      m_servoPosError = inputs.servoLeftPosition - m_newServoPosition;
      if(Math.abs(m_servoPosError) < 0.05) {
        m_shooterServoInPos = true;
      }
    } // End of Enabled
    
    Logger.recordOutput("shooter/servopos", m_newServoPosition);

  } //end of shooter periodic

  // Testing
  public Command setServoPos(double position) {
    return run(()->io.setServoPosition(position));
  }

  //-------------------------------------------------------------------------------------
  // Intake Calculation Methods
  //-------------------------------------------------------------------------------------
  public void updateTargetPositionShooter(CatzMechanismPosition newPosition) {
    // double previousServoPosition = m_newServoPosition;
    // m_shooterServoInPos = false;
    // m_newServoPosition = newPosition.getShooterVerticalTargetAngle();
    // if(newPosition.getShooterVerticalTargetAngle() == SERVO_NULL_POSITION) {
    //   m_newServoPosition = previousServoPosition;
    // }
  }

  public Command cmdSetKeepShooterOn(boolean state){
    return runOnce(() -> {
      autonKeepFlywheelOn = state;
    });
  }

  public double getScuffedShootingSpeed(){
    return ((inputs.shooterVelocityRT + inputs.shooterVelocityLT)/2+2) * CatzConstants.ShooterConstants.WHEEL_CIRCUMFERENCE; //math is definitely correct (winkwink) TBD
  }

  public Command cmdServoPosition(Supplier<Double> value) {
    return run(()->io.setServoPosition(value.get()));
  }

  public void updateShooterServo(double position) {
    m_shooterServoInPos = false;
    m_newServoPosition = position;
  }

  public Command cmdManualHoldOn(Supplier<Double> pwr) {
    return run(()-> setServoManualHold(pwr.get()));
  }

  public void setServoManualHold(double power) {
    if(Math.abs(power) > 0.1) {
      m_newServoPosition = m_newServoPosition + (power * 0.01);
    }

  } 

  //-------------------------------------------------------------------------------------
  // Getter Methods 
  //------------------------------------------------------------------------------------- 

  private boolean getShooterServoInPos() {
    return m_shooterServoInPos;
  }
  public ShooterNoteState getShooterNoteState() {
    return currentNoteState;
  }

  public boolean shooterLoadBeamBrkBroken() {
    return inputs.shooterLoadBeamBreakState;
  }
  
  //-------------------------------------------------------------------------------------
  // Flywheel Commands
  //-------------------------------------------------------------------------------------

  public void startShooterFlywheel() {
    currentShooterState = ShooterState.START_SHOOTER_FLYWHEEL;
  }

  public Command cmdShooterRamp(){
    return runOnce(()-> startShooterFlywheel());
  }

  public Command cmdShooterDisabled() {
    return runOnce(()->disableShooter());
  }

  public void disableShooter() {
    currentShooterState = ShooterState.LOAD_OFF;
    io.setShooterDisabled(); 
    io.loadDisabled();
  }

  public void setShooterState(ShooterState state) {
    currentShooterState = state;
  }

  public Command cmdShoot() {
    return runOnce(()->setShooterState(ShooterState.SHOOTING));
  }

  //-------------------------------------------------------------------------------------
  // Loading Methods 
  //-------------------------------------------------------------------------------------
  public Command cmdLoad(){
    return runOnce(()->setShooterState(ShooterState.LOAD_IN));
  }
  
  public Command loadBackward() {
    m_iterationCounter = 0;
    return runOnce(()->setShooterState(ShooterState.LOAD_OUT));
  }
  
  public Command loadDisabled() {
    return runOnce(()->setShooterState(ShooterState.LOAD_OFF));

  }

  public void hoardShootingLogic(){
    new MoveToPreset(CatzConstants.CatzMechanismConstants.SHOOTER_HOARD_PRESET);
    startShooterFlywheel();
  }
  public Command hoardShooterShot(){
    return runOnce(()-> hoardShootingLogic());
  }
}
