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
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.subsystems.intake.SubsystemCatzIntake;
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
  LoggedTunableNumber servoPosTuning = new LoggedTunableNumber("ServoPos", 0);
  
  //Servo SetPositions
  public static final double SERVO_MIN_POS = 0.0;
  public static final double SERVO_OPTIMAL_HANDOFF_HIGH_POS = 0.4;
  public static final double SERVO_MAX_POS = 1.0;
  public static final double SERVO_NULL_POSITION  = -999.0;

  private static final double SERVO_DEADBAND = 0.05;


  private double m_targetServoPosition;
  private double m_previousServoPosition;
  private double m_servoPosError;

  /*-----------------------------------------------------------------------------------------
   * Time Constants
   *-----------------------------------------------------------------------------------------*/
  private final double LOOP_CYCLE_SECONDS = 0.02;

  private final int WAIT_FOR_MOTORS_TO_REV_UP_TIMEOUT = (int) (Math.round(5.0/LOOP_CYCLE_SECONDS) + 1.0); 
  private final int SHOOTING_TIMEOUT                  = (int) (Math.round(0.5/LOOP_CYCLE_SECONDS) + 1.0);
  private final int LOAD_OUT_TIMEOUT                  = (int) (Math.round(0.5/LOOP_CYCLE_SECONDS) + 1.0);

  private Timer servoTimer = new Timer();

  private final double SERVO_MAX_SET_POSITION_TIME = 2.0;

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
  private boolean autonIsShooterRamped = false;
  
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

  private boolean shooterTimeout = false;

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
          // System.out.println(inputs.shooterVelocityRT + " " + inputs.velocityThresholdRT);
          // System.out.println(inputs.shooterVelocityLT + " " + inputs.velocityThresholdLT);
            if(inputs.shooterVelocityRT >= inputs.velocityThresholdRT &&
               Math.abs(inputs.shooterVelocityLT) >= Math.abs(inputs.velocityThresholdLT)) {

                  if(DriverStation.isAutonomous()) {
                      autonIsShooterRamped = true;
                  } else {
                    xboxAuxRumble.setRumble(RumbleType.kBothRumble, 0.7);
                  }

            } else {

              if(DriverStation.isAutonomous()) {
                if(m_iterationCounter > WAIT_FOR_MOTORS_TO_REV_UP_TIMEOUT) {
                    shooterTimeout = true;
                    m_iterationCounter = 0;
                    currentShooterState = ShooterState.SHOOTING;
                    autonIsShooterRamped = true;
                }
                m_iterationCounter++;
              }
            }
            break;

          case SHOOTING:
            io.feedShooter();
            
            xboxAuxRumble.setRumble(RumbleType.kBothRumble, 0);

            if(m_iterationCounter >= SHOOTING_TIMEOUT) 
            {
              if(DriverStation.isAutonomous()) 
              {
                if (autonKeepFlywheelOn == true) 
                  {
                    currentShooterState = ShooterState.WAIT_FOR_MOTORS_TO_REV_UP;
                  } 
                else if (autonKeepFlywheelOn == false)
                  {
                    io.setShooterDisabled();
                    autonIsShooterRamped = false;
                    currentShooterState = ShooterState.LOAD_OFF;
                  }
                }
              else {
                  io.setShooterDisabled();
                  currentShooterState = ShooterState.LOAD_OFF;
                }
                
              currentNoteState = ShooterNoteState.NOTE_HAS_BEEN_SHOT; //ends autoaim sequence
              SubsystemCatzTurret.getInstance().setTurretTargetDegree(-999);
              m_iterationCounter = 0;
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
      
      //min max clamping
      if(m_targetServoPosition > SERVO_MAX_POS) {
        m_targetServoPosition = SERVO_MAX_POS;
      } else if(m_targetServoPosition < SERVO_MIN_POS) {
        m_targetServoPosition = SERVO_MIN_POS;
      }

      //turret clamping
      if(Math.abs(SubsystemCatzTurret.getInstance().getTurretAngle()) > 80) {
        if(m_targetServoPosition > SubsystemCatzTurret.SERVO_TURRET_CONSTRAINT) {
          m_targetServoPosition = SubsystemCatzTurret.SERVO_TURRET_CONSTRAINT;
        } 
      } 
    
      //cmd final output
      io.setServoPosition(m_targetServoPosition);

      
      
      //servos in position
      m_servoPosError = Math.abs(m_previousServoPosition - m_targetServoPosition);
      if(m_servoPosError > SERVO_DEADBAND) {
        m_shooterServoInPos = false;
        servoTimer.restart();
      }

      if(servoTimer.hasElapsed(SERVO_MAX_SET_POSITION_TIME)) {
        m_shooterServoInPos = true;
      }

      m_previousServoPosition = m_targetServoPosition;
    } // End of Enabled loop
    
    Logger.recordOutput("shooter/servopos", m_targetServoPosition);
    Logger.recordOutput("shooter/isAutonRamped", isAutonShooterRamped());
    Logger.recordOutput("shooter/isShooting", currentShooterState == ShooterState.SHOOTING);
    Logger.recordOutput("shooter/shooterTimeuot",shooterTimeout);
    Logger.recordOutput("shooter/servoTimer", servoTimer.get());

  } //end of shooter periodic

  //-------------------------------------------------------------------------------------
  // Shooter Calculation Methods
  //-------------------------------------------------------------------------------------
  public void updateTargetPositionShooter(CatzMechanismPosition newPosition) {
    double previousServoPosition = m_targetServoPosition;
    m_shooterServoInPos = false;
    m_targetServoPosition = newPosition.getShooterVerticalTargetAngle();
    if(newPosition.getShooterVerticalTargetAngle() == SERVO_NULL_POSITION) {
      m_targetServoPosition = previousServoPosition;
    }
  }

  public Command cmdSetKeepShooterOn(boolean state){
    return runOnce(() -> {
      autonKeepFlywheelOn = state;
    });
  }

  public boolean isAutonShooterRamped(){
    return autonIsShooterRamped;
  }

  public double getScuffedShootingSpeed(){
    return ((inputs.shooterVelocityRT + inputs.shooterVelocityLT)/2+2) * CatzConstants.ShooterConstants.WHEEL_CIRCUMFERENCE; //math is definitely correct (winkwink) TBD
  }

  public Command cmdServoPosition(Supplier<Double> value) {
    return run(()->io.setServoPosition(value.get()));
  }

  public void updateShooterServo(double position) {
    m_shooterServoInPos = false;
    m_targetServoPosition = position;
  }

  public Command cmdManualHoldOn(Supplier<Double> pwr) {
    return run(()-> setServoManualHold(pwr.get()));
  }

  public void setServoManualHold(double position) {
    //reverse direction so Up on the joystick is upward direction on servo
    position = -position;
    
    m_targetServoPosition = m_targetServoPosition + (position * 0.01);
  
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

  public double getServoCommandedPosition() {
    return m_targetServoPosition;
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
