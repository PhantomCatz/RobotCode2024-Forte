// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.geometry.Translation2d;
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


  private double m_newServoPosition;
  private double m_servoPosError;

  /*-----------------------------------------------------------------------------------------
   * States
   *-----------------------------------------------------------------------------------------*/

  private static ShooterLoadState currentShooterLoadState;
  public static enum ShooterLoadState {
    LOAD_IN,
    FINE_TUNE,
    LOAD_IN_DONE,
    WAIT_FOR_NOTE_TO_SETTLE,
    WAIT_FOR_MOTORS_TO_REV_UP,
    START_SHOOTER_FLYWHEEL,
    SHOOTING,
    LOAD_OFF,
    LOAD_OUT
  }

  //for determining state machine for shooter
  private ShooterServoState currentShooterServoState;
  public enum ShooterServoState {
    FULL_MANUAL,
    AUTO,
    TUNNING
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

  private final double LOOP_CYCLE_MS = 0.02;

  private static final boolean BEAM_IS_BROKEN     = true;
  private static final boolean BEAM_IS_NOT_BROKEN = false;

  private boolean m_desiredBeamBreakState;
  private int     m_iterationCounter;
  private int     m_iterationCounter2;

  private boolean m_shooterServoInPos = false;
  
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
    if(DriverStation.isDisabled()) {
      currentShooterLoadState = ShooterLoadState.LOAD_OFF;
      io.setShooterDisabled();
    } else {
      switch(currentShooterLoadState) {
          //-------------------------------------------------------------------------------------------
          //
          // feeder roller periodic logic
          //
          //-------------------------------------------------------------------------------------------
          case LOAD_IN:
            io.loadNote();
            currentShooterLoadState = ShooterLoadState.LOAD_IN_DONE;
            currentNoteState = ShooterNoteState.NOTE_IN_ADJUST;
          break;

          case LOAD_IN_DONE:
            if(inputs.shooterLoadBeamBreakState == BEAM_IS_BROKEN) { 
              io.loadDisabled();
              m_iterationCounter = 0;
              currentShooterLoadState = ShooterLoadState.WAIT_FOR_NOTE_TO_SETTLE;


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
                currentShooterLoadState = ShooterLoadState.FINE_TUNE;
          break;

          case FINE_TUNE:
            if(inputs.shooterAdjustBeamBreakState == m_desiredBeamBreakState) { //if front is still conncected adjust foward until it breaks
              io.loadDisabled();
              currentShooterLoadState = ShooterLoadState.LOAD_OFF;
              currentNoteState = ShooterNoteState.NOTE_IN_POSTION;
            }
          break;
          
          case START_SHOOTER_FLYWHEEL:
            io.setShooterEnabled();
            m_iterationCounter2 =0; 
            currentShooterLoadState = ShooterLoadState.WAIT_FOR_MOTORS_TO_REV_UP;
          break;
          
          
          case WAIT_FOR_MOTORS_TO_REV_UP:
          //System.out.println(-inputs.shooterVelocityLT + " Lt sHOOTER " + inputs.velocityThresholdLT);
          if(DriverStation.isAutonomous()) {
            m_iterationCounter2++;
            if(m_iterationCounter2 > timer(1)) {
                m_iterationCounter = 0;
                currentShooterLoadState = ShooterLoadState.SHOOTING;
   
            }
          } else {
            if(inputs.shooterVelocityRT >= inputs.shooterVelocityRT-20 &&
               inputs.shooterVelocityLT >= inputs.shooterVelocityLT-20) {
                xboxAuxRumble.setRumble(RumbleType.kBothRumble, 0.7);
                
                m_iterationCounter = 0;  
                
            }
          }
          break;

          case SHOOTING:
            io.feedShooter();
            if(DriverStation.isAutonomous() == false) {
              xboxAuxRumble.setRumble(RumbleType.kBothRumble, 0);
            }

            m_iterationCounter++;
            // System.out.println(m_iterationCounter);

            if(m_iterationCounter >= timer(1)) {
              io.setShooterDisabled();
              currentShooterLoadState = ShooterLoadState.LOAD_OFF;
              currentNoteState = ShooterNoteState.NOTE_HAS_BEEN_SHOT;
              SubsystemCatzTurret.getInstance().setTurretTargetDegree(0.0);
              
            }
          break;

          case LOAD_OFF:
            io.loadDisabled();
          break;

          case LOAD_OUT:
            io.loadBackward();
            m_iterationCounter++;
            if(m_iterationCounter >= timer(0.5)) {
              io.loadDisabled();
              currentShooterLoadState = ShooterLoadState.LOAD_OFF;
              m_iterationCounter = 0;
            }
          break;
      }
    
      Logger.recordOutput("shooter/current load state", currentShooterLoadState.toString());
      Logger.recordOutput("shooter/servopos", m_newServoPosition);

      //-------------------------------------------------------------------------------------------
      //
      // servo periodic logic
      //
      //-------------------------------------------------------------------------------------------
      m_servoPosError = inputs.servoLeftPosition - m_newServoPosition;

      io.setServoPosition(m_newServoPosition);
      if(Math.abs(m_servoPosError) < 0.1) {
        m_shooterServoInPos = true;
      }
    }
  }

  //-------------------------------------------------------------------------------------
  // Intake Calculation Methods
  //-------------------------------------------------------------------------------------
  public void updateTargetPositionShooter(CatzMechanismPosition newPosition) {
    m_shooterServoInPos = false;
    m_newServoPosition = newPosition.getShooterVerticalTargetAngle();
  }

  public double getScuffedShootingSpeed(){
    return ((inputs.shooterVelocityRT + inputs.shooterVelocityLT)/2+2) * CatzConstants.ShooterConstants.WHEEL_CIRCUMFERENCE; //math is definitely correct (winkwink) TBD
  }

  public Command cmdServoPosition(double value) {
    return runOnce(()-> updateShooterServo(value));
  }

  public void updateShooterServo(double position) {
    m_shooterServoInPos = false;
    m_newServoPosition = position;
  }

  public Command setPositionCmd(Supplier<Double> position) {
    return run(()->m_newServoPosition = position.get());
  }

  //-------------------------------------------------------------------------------------
  // Calculation Methods 
  //------------------------------------------------------------------------------------- 
  private double timer(double seconds){ // in seconds; converts time to iteration counter units
    //System.out.println(Math.round(seconds/LOOP_CYCLE_MS) + 1);
    return Math.round(seconds/LOOP_CYCLE_MS) + 1;
  }

  //-------------------------------------------------------------------------------------
  // Getter Methods 
  //------------------------------------------------------------------------------------- 

  public boolean getShooterServoInPos() {
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
    currentShooterLoadState = ShooterLoadState.START_SHOOTER_FLYWHEEL;
  }

  public Command cmdShooterDisabled() {
    return runOnce(()->disableShooterFlywheel());
  }

  public void disableShooterFlywheel() {
    io.setShooterDisabled();
  }

  //-------------------------------------------------------------------------------------
  // Shooter Loading Methods 
  //-------------------------------------------------------------------------------------
  public Command cmdShoot() {
      return runOnce(()->setShooterLoadState(ShooterLoadState.SHOOTING));
  }

  public Command cmdLoad(){
    return runOnce(()->setShooterLoadState(ShooterLoadState.LOAD_IN));
  }
  
  public Command loadBackward() {
    m_iterationCounter = 0;
    return runOnce(()->setShooterLoadState(ShooterLoadState.LOAD_OUT));
  }
  
  public Command loadDisabled() {
    return runOnce(()->setShooterLoadState(ShooterLoadState.LOAD_OFF));
  }

  public Command rampUpFlyWheels(){
    return runOnce(()->startShooterFlywheel());
  }

  public void setShooterLoadState(ShooterLoadState state) {
    currentShooterLoadState = state;
  }

  public void hoardShootingLogic(){
    new MoveToPreset(CatzConstants.CatzMechanismConstants.SHOOTER_HOARD_PRESET);
    startShooterFlywheel();
  }
  public Command hoardShooterShot(){
    return runOnce(()-> hoardShootingLogic());
  }
}
