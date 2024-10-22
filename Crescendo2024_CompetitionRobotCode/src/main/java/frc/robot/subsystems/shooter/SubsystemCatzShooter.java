// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.CatzConstants.RobotMode;
import frc.robot.Utils.LoggedTunableNumber;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.turret.SubsystemCatzTurret;
import frc.robot.subsystems.turret.SubsystemCatzTurret.TurretState;
import frc.robot.subsystems.vision.SubsystemCatzVision;


public class SubsystemCatzShooter extends SubsystemBase {
  
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static SubsystemCatzShooter instance = new SubsystemCatzShooter();

  /*-----------------------------------------------------------------------------------------
   * Linear Servo Values
   *-----------------------------------------------------------------------------------------*/
  LoggedTunableNumber servoPosTuning = new LoggedTunableNumber("ServoPosition", 0);
  
  //Servo SetPositions
  public static final double SERVO_MIN_POS = 0.0;
  public static final double SERVO_MAX_POS = 1.0;
  public static final double SERVO_IGNORE_POSITION  = -999.0;
  
  private static final double SERVO_MAX_EXTENSTION_MM = 100.0;

  private static final double SERVO_STARTING_CONFIG_POS = 0.85;



  private final double SERVO_VELOCITY_MM_PER_SEC = 100.0/3.3;

  private double m_targetServoPosition   = SERVO_STARTING_CONFIG_POS;
  private double m_previousServoPosition = SERVO_MAX_POS;
  private double m_servoPosError;

  private double servoDistToMoveMm;
  private double servoPositionTimeout;

  /*-----------------------------------------------------------------------------------------
   * Time Constants
   *-----------------------------------------------------------------------------------------*/
  private final double LOOP_CYCLE_SECONDS = 0.02;

  private final int WAIT_FOR_MOTORS_TO_REV_UP_TIMEOUT = (int) (Math.round(1.0/LOOP_CYCLE_SECONDS) + 1.0); 
  private final int SHOOTING_TIMEOUT                  = (int) (Math.round(0.5/LOOP_CYCLE_SECONDS) + 1.0);
  private final int LOAD_OUT_TIMEOUT                  = (int) (Math.round(0.5/LOOP_CYCLE_SECONDS) + 1.0);

  private Timer servoTimer = new Timer();
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
    START_SHOOTER_FLYWHEEL_HOARD_MODE,
    PREP_FOR_HANDOFF_SHIFT,
    HANDOFF_SHIFT,
    HANDOFF_FINE_ADJUST,
    SHOOTING,
    LOAD_OFF,
    LOAD_OUT,
    NONE;
  }

  private static ServoState currentServoState = ServoState.IDLE;
  public static enum ServoState {
    IDLE,
    MOVE_SERVO_INIT,
    WAIT_FOR_SERVO_IN_POSITION
  }
 
  //shooter note state for determining when other mechanism should turn off
  private ShooterNoteState currentNoteState = ShooterNoteState.NULL;
  public enum ShooterNoteState {
    NOTE_IN_POSTION,
    NOTE_IN_ADJUST,
    NOTE_HAS_BEEN_SHOT,
    NOTE_POSITION_ADJUST,
    NULL
  }

  /*-----------------------------------------------------------------------------------------
  * Constants for Shooter
  *-----------------------------------------------------------------------------------------*/
  private static final boolean BEAM_IS_BROKEN     = true;
  private static final boolean BEAM_IS_NOT_BROKEN = false;

  private static final double HANDOFF_TRANSFER_CNT_SHIFT = 3.0;

  private boolean m_desiredBeamBreakState;
  private int     m_iterationCounterRampingTimeout = 0;
  private int     m_iterationCounterShooting = 0;

  private double m_startingLoadEncoderHandoff;

  private boolean m_shooterServoInPos = false;
  private boolean autonKeepFlywheelOn = false;
  private boolean autonIsShooterRamped = false;

  /*-------------------------------------------------------------------------------------------
   * Shooter Velocities
   *--------------------------------------------------------------------------------------------*/
  public static final double SHOOTER_VELOCITY_LT = 57.0; //For Shooting Speaker
  public static final double SHOOTER_VELOCITY_RT = 80.0;
  public static final double FLYWHEEL_THRESHOLD_OFFSET = 5;


  //Will be changed to a final double when confirmed speed, right now those speeds are made up
  public static LoggedTunableNumber hoardShooterVelLT = new LoggedTunableNumber("HoardLTVelShooter", 50); // For Hoarding 
  public static LoggedTunableNumber hoardShooterVelRT = new LoggedTunableNumber("HoardRTVelShooter", 70); 

  private double m_velocityThresholdRT;
  private double m_velocityThresholdLT;
  
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
    updateShooterServo(SERVO_STARTING_CONFIG_POS);
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
     
          case NONE:
          break;
      /*--------------------------------------------------------------------------------------------------------------------
      * LOADING NOTE
      *--------------------------------------------------------------------------------------------------------------------*/
          case LOAD_IN:
            io.loadNote();
            currentShooterState = ShooterState.LOAD_IN_DONE;
            currentNoteState = ShooterNoteState.NOTE_IN_ADJUST;
          break;

          case LOAD_IN_DONE:
            if(inputs.shooterLoadBeamBreakState == BEAM_IS_BROKEN) { 
              io.loadDisabled();
              m_iterationCounterRampingTimeout = 0;
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
      /*--------------------------------------------------------------------------------------------------------------------
      * ADJUSTING NOTE
      *--------------------------------------------------------------------------------------------------------------------*/
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
          
      /*--------------------------------------------------------------------------------------------------------------------
      * HANDING OFF
      *--------------------------------------------------------------------------------------------------------------------*/
          case PREP_FOR_HANDOFF_SHIFT:

            currentNoteState = ShooterNoteState.NOTE_IN_ADJUST;
            if(inputs.shooterAdjustBeamBreakState == BEAM_IS_BROKEN) {
              //note is at the desired loading position for shooting
              m_desiredBeamBreakState = BEAM_IS_BROKEN;
              currentShooterState = ShooterState.HANDOFF_SHIFT;
            

            } else {
              //note has not reached the stowed position
              io.fineTransferAdjust();
              m_desiredBeamBreakState = BEAM_IS_BROKEN;
              currentShooterState = ShooterState.HANDOFF_SHIFT;            
            }

          break;

          case HANDOFF_SHIFT:

            if(inputs.shooterAdjustBeamBreakState == m_desiredBeamBreakState) { //if front is still conncected adjust foward until it breaks
              //note is at the desired loading position for shooting
              io.loadDisabled();
              m_startingLoadEncoderHandoff = inputs.loadMotorEncCnts;
              currentShooterState = ShooterState.HANDOFF_FINE_ADJUST;
              currentNoteState = ShooterNoteState.NOTE_POSITION_ADJUST;   
              io.loadNote();
            }            

          break;

          case HANDOFF_FINE_ADJUST:
            if(Math.abs(inputs.loadMotorEncCnts - m_startingLoadEncoderHandoff) > HANDOFF_TRANSFER_CNT_SHIFT) { //absoulte value because encoder cnts are in negative
              //note is at desired loading position for handoff transfer
              io.loadDisabled();
              currentShooterState = ShooterState.LOAD_OUT;
              currentNoteState = ShooterNoteState.NOTE_IN_POSTION;
            }
          break;
      /*--------------------------------------------------------------------------------------------------------------------
      * LOAD OUT
      *--------------------------------------------------------------------------------------------------------------------*/
          case LOAD_OUT:
            io.loadBackward();
            
            if(m_iterationCounterRampingTimeout >= LOAD_OUT_TIMEOUT) {
              io.loadDisabled();
              currentShooterState = ShooterState.LOAD_OFF;
              m_iterationCounterRampingTimeout = 0;
            }
            m_iterationCounterRampingTimeout++;
          break;

      /*--------------------------------------------------------------------------------------------------------------------
      * SHOOTING LOGIC 
      *--------------------------------------------------------------------------------------------------------------------*/
          case START_SHOOTER_FLYWHEEL:
            setFlyWheelVelocities();
            currentShooterState = ShooterState.WAIT_FOR_MOTORS_TO_REV_UP;
            io.loadDisabled();

          break;

          case WAIT_FOR_MOTORS_TO_REV_UP:
            if(inputs.shooterVelocityRT >= m_velocityThresholdRT &&
               Math.abs(inputs.shooterVelocityLT) >= Math.abs(m_velocityThresholdLT)) { //abs due to negative inversion
                //flywheels have reached speeds
              if(DriverStation.isAutonomous()) {

                autonIsShooterRamped = true;
              } else {

                xboxAuxRumble.setRumble(RumbleType.kBothRumble, 0.7);
              }
            } else { 
              //flywheels not at velocity
              if(DriverStation.isAutonomous()) {

                //timeout if desired velocity not reached in time
                if(m_iterationCounterRampingTimeout > WAIT_FOR_MOTORS_TO_REV_UP_TIMEOUT) {

                  //we passed the timeout shoot anyway
                  m_iterationCounterRampingTimeout = 0;
                  autonIsShooterRamped = true;
                }
                m_iterationCounterRampingTimeout++;
              }
            }
            
            break;

          case SHOOTING:
            io.feedShooter();
            SubsystemCatzIntake.getInstance().setRollersOutakeHandoff();
            
            xboxAuxRumble.setRumble(RumbleType.kBothRumble, 0);

            if(m_iterationCounterShooting >= SHOOTING_TIMEOUT) 
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
              SubsystemCatzTurret.getInstance().setTurretState(TurretState.AUTO);
              SubsystemCatzTurret.getInstance().setTurretTargetDegree(0);
              m_iterationCounterShooting = 0;
              SubsystemCatzIntake.getInstance().setRollersOff();
            }
            m_iterationCounterShooting++;
          break;

          default:
          break;
      }
  
      //-------------------------------------------------------------------------------------------
      //
      // servo periodic logic
      //
      //-------------------------------------------------------------------------------------------
      
      //min max servo value clamping
      if(m_targetServoPosition > SERVO_MAX_POS) {

        m_targetServoPosition = SERVO_MAX_POS;
      } else if(m_targetServoPosition < SERVO_MIN_POS) {

        m_targetServoPosition = SERVO_MIN_POS;
      }

      //turret clamping
      if(Math.abs(SubsystemCatzTurret.getInstance().getTurretAngle()) > SubsystemCatzTurret.TURRET_MAX_SERVO_LIMIT_DEG) {

        if(m_targetServoPosition > SubsystemCatzTurret.SERVO_TURRET_CONSTRAINT) {

           m_targetServoPosition = SubsystemCatzTurret.SERVO_TURRET_CONSTRAINT;
        } 
      } 
    
      //cmd final output
      io.setServoPosition(m_targetServoPosition); //TBD change back to 10/

      //-------------------------------------------------------------------------------------------
      //  Servos are commanded from 0.0 to 1.0 where 0.0 represents 0% of max extension and 1.0
      //  represents 100% of max extension or 100% of 100 mm.  m_xxxServoPosition represents % of
      //  max extension.  Convert % of max extension to a distance in mm and use that to calculate
      //  timeout value based on servo velocity in mm/sec
      //-------------------------------------------------------------------------------------------    
      switch(currentServoState) {
        case IDLE:

        break;

        case MOVE_SERVO_INIT:

          m_servoPosError = Math.abs(m_previousServoPosition - m_targetServoPosition);

          if(Math.abs(m_servoPosError) > 0.0) {
            //init
            //-------------------------------------------------------------------------------------------
            //  If a new servo position is being commanded, then clear servo in position flag and restart
            //  timer.  Note that we can't readback position of the servos so we are going to assume 
            //  servo is in position based on time (e.g. velocity * time = disatnce)
            //-------------------------------------------------------------------------------------------
            servoTimer.restart();
            m_shooterServoInPos = false;

            servoDistToMoveMm    = m_servoPosError * SERVO_MAX_EXTENSTION_MM; 
            servoPositionTimeout = servoDistToMoveMm / SERVO_VELOCITY_MM_PER_SEC ;

            m_previousServoPosition = m_targetServoPosition;
            currentServoState = ServoState.WAIT_FOR_SERVO_IN_POSITION;
          } else {
            //not commanding new position
            m_shooterServoInPos = true;
            currentServoState = ServoState.IDLE;
          }
        break;
          
        case WAIT_FOR_SERVO_IN_POSITION:

          if(servoTimer.hasElapsed(servoPositionTimeout)) {
            //shooter servos are in position due to timeout
            m_shooterServoInPos = true;
            currentServoState = ServoState.IDLE;
          }
        break;
      }
    } // End of Enabled loop

    importantShooterLogs();

  } //end of shooter periodic

  //-------------------------------------------------------------------------------------
  // Debug Logger For Shooter 
  //-------------------------------------------------------------------------------------
  public void importantShooterLogs(){
    Logger.recordOutput("shooter/servopos", m_targetServoPosition);
  }

  public void debugLogsShooter(){
    //DEBUG
    // Logger.recordOutput("shooter/seroDistToMoveMm", servoDistToMoveMm);
    // Logger.recordOutput("shooter/seroPosTimeOut", servoPositionTimeout);
    // Logger.recordOutput("shooter/servopos", m_targetServoPosition);
    // Logger.recordOutput("shooter/isAutonRamped", isAutonShooterRamped());
    // Logger.recordOutput("shooter/isServoInPos", m_shooterServoInPos);
    // Logger.recordOutput("shooter/currentServoState", currentServoState.toString());
    // Logger.recordOutput("shooter/servoTimer", servoTimer.get());
    // Logger.recordOutput("shooter/startingenchandoff", m_startingLoadEncoderHandoff);
    // Logger.recordOutput("shooter/currentNoteState", currentNoteState.toString());
    // Logger.recordOutput("shooter/iterationCounterShooting", m_iterationCounterShooting);


  }

  //-------------------------------------------------------------------------------------
  // Auto Aim Calculations
  //-------------------------------------------------------------------------------------

  public void updateShooterServo(double position) {
    
    if(position == SERVO_IGNORE_POSITION) {
      //change position back to the previous position
      position = m_previousServoPosition;
    }

    if(m_previousServoPosition == position) {
        //target position hasn't changed 
        //periodic loop will handle if servo is in position or not

    } else {
      //there is a new target poition
      m_previousServoPosition = m_targetServoPosition;
      m_targetServoPosition = position;
      currentServoState = ServoState.MOVE_SERVO_INIT; 
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

  public double getNoteShootingSpeed(){
    //72 rps *  0.2 m wheel circum  = 14.4 mps note speed
    return ((inputs.shooterVelocityRT + inputs.shooterVelocityLT)/2+2) * CatzConstants.ShooterConstants.WHEEL_CIRCUMFERENCE; //math is definitely correct (winkwink) TBD
  }

  public Command cmdManualHoldOn(Supplier<Double> pwr) {
    return run(()-> setServoManualHold(pwr.get()));
  }

  public void setServoManualHold(double position) {
    //reverse direction so Up on the joystick is upward direction on servo
    position = -position;
    
    double manualPosition = m_targetServoPosition + (position * 0.01);

    updateShooterServo(manualPosition);
  } 

  public void aprilTagVerticalTargeting() {

    if(SubsystemCatzVision.getInstance().getOffsetY(2) > 1) {

      setServoManualHold(1.0);
      m_shooterServoInPos = false;      
    } else if (SubsystemCatzVision.getInstance().getOffsetY(2) < 1) {

      setServoManualHold(-1.0);
      m_shooterServoInPos = false;
    } else {

      m_shooterServoInPos = true;
    }
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
  
  public double getShooterServoTargetPosition() {
    return m_targetServoPosition;
  }

  public boolean shooterLoadBeamBrkBroken() {
    return inputs.shooterLoadBeamBreakState;
  }

  public double getServoCommandedPosition() {
    return m_targetServoPosition;
  }
  
  //-------------------------------------------------------------------------------------
  // Flywheel Methods and Commands
  //-------------------------------------------------------------------------------------

  public Command cmdShooterRamp(){
    return runOnce(()-> startShooterFlywheel());
  }

  public Command cmdShooterDisabled() {
    return runOnce(()->disableShooter());
  }

  public Command cmdShoot() {
    return runOnce(()->setShooterState(ShooterState.SHOOTING));
  }

  public void startShooterFlywheel() {
    currentShooterState = ShooterState.START_SHOOTER_FLYWHEEL;
  }

  public void disableShooter() {
    currentShooterState = ShooterState.LOAD_OFF;
    io.setShooterDisabled(); 
    io.loadDisabled();
    autonIsShooterRamped = false;
    xboxAuxRumble.setRumble(RumbleType.kBothRumble, 0);
  }

  public void setShooterState(ShooterState state) {
    currentShooterState = state;
  }

  public void setFlyWheelVelocities() {
    double velocityLT;
    double velocityRT;

    if(CatzConstants.currentRobotMode == RobotMode.HOARD) {

      m_velocityThresholdLT = -hoardShooterVelLT.get() + FLYWHEEL_THRESHOLD_OFFSET;
      m_velocityThresholdRT =  hoardShooterVelRT.get() - FLYWHEEL_THRESHOLD_OFFSET;
      velocityLT = hoardShooterVelLT.get();
      velocityRT = hoardShooterVelRT.get();
    } else {

      m_velocityThresholdLT = -SHOOTER_VELOCITY_LT + FLYWHEEL_THRESHOLD_OFFSET;
      m_velocityThresholdRT =  SHOOTER_VELOCITY_RT - FLYWHEEL_THRESHOLD_OFFSET;
      velocityLT = SHOOTER_VELOCITY_LT;
      velocityRT = SHOOTER_VELOCITY_RT;
    }

      io.setShooterEnabled(velocityLT, velocityRT);    
  }

  //-------------------------------------------------------------------------------------
  // Loading Methods 
  //-------------------------------------------------------------------------------------
  public Command cmdLoad(){
    return runOnce(()->setShooterState(ShooterState.LOAD_IN));
  }
  
  public Command loadBackward() {
    m_iterationCounterRampingTimeout = 0;
    return runOnce(()->setShooterState(ShooterState.LOAD_OUT));
  }
  
  public Command loadDisabled() {
    return runOnce(()->setShooterState(ShooterState.LOAD_OFF));

  }

  public void setShooterNoteState(ShooterNoteState noteState) {
    currentNoteState = noteState;
  }

}
