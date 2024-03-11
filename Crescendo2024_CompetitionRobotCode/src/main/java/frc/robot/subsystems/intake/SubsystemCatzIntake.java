// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.Utils.LoggedTunableNumber;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.subsystems.turret.SubsystemCatzTurret;


public class SubsystemCatzIntake extends SubsystemBase {
  //intake io block
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  //intake instance
  private static SubsystemCatzIntake instance = new SubsystemCatzIntake();

 /************************************************************************************************************************
  * 
  * rollers
  *
  ************************************************************************************************************************/
  private final double ROLLERS_MTR_PWR_IN_GROUND      =  0.25;//0.6;
  private final double ROLLERS_MTR_PWR_IN_SOURCE      =  0.25;
  private final double ROLLERS_MTR_PWR_OUT_EJECT      = -0.6; 
  private final double ROLLERS_MTR_PWR_OUT_HANDOFF    = -0.4; 


  private IntakeRollerState m_currentRollerState;
  public static enum IntakeRollerState {
    ROLLERS_IN_SOURCE,
    ROLLERS_IN_GROUND,
    ROLLERS_OUT_EJECT,
    ROLLERS_OUT_SHOOTER_HANDOFF,
    ROLLERS_OFF
  }

  /************************************************************************************************************************
  * 
  * pivot
  *
  ************************************************************************************************************************/
  private static final double INTAKE_PIVOT_DRIVEN_GEAR      = 52.0;
  private static final double INTAKE_PIVOT_DRIVING_GEAR     = 30.0;

  private static final double INTAKE_PIVOT_DRIVEN_SPROCKET  = 32.0;
  private static final double INTAKE_PIVOT_DRIVING_SPROCKET = 16.0;

  private static final double MAX_PLANETARY_RATIO           = 5.0;

  private static final double INTAKE_PIVOT_GEAR_RATIO = (INTAKE_PIVOT_DRIVEN_GEAR     / INTAKE_PIVOT_DRIVING_GEAR) * 
                                                        (INTAKE_PIVOT_DRIVEN_SPROCKET / INTAKE_PIVOT_DRIVING_SPROCKET) * 
                                                        (MAX_PLANETARY_RATIO); 

  private static final double INTAKE_PIVOT_MTR_REV_PER_DEG = INTAKE_PIVOT_GEAR_RATIO / 360.0;

  private static final double INTAKE_PIVOT_MTR_POS_OFFSET_IN_DEG = 164.09;

  public static final double INTAKE_PIVOT_MTR_POS_OFFSET_IN_REV = INTAKE_PIVOT_MTR_POS_OFFSET_IN_DEG * INTAKE_PIVOT_MTR_REV_PER_DEG;


  //pid constants
  public final double PIVOT_FF_kS = 0.00;
  public final double PIVOT_FF_kG = 0.437;
  public final double PIVOT_FF_kV = 0.00;
  public final double PIVOT_FF_kA = 0.0;

  public static final double PIVOT_PID_kP = 9.00; //0.044
  public static final double PIVOT_PID_kI = 0.00; //0.005 
  public static final double PIVOT_PID_kD = 0.27; 

  //Intake positions
  public static final double INTAKE_GROUND_PICKUP             = -22.0;
  public static final double INTAKE_AMP_SCORE                 = -22.0;
  public static final double INTAKE_POS_UPRIGHT               = 125;//92.6; //90.43; //97 with drivetrain inner rail to the bottom inner rail 7 1/4 inches
  public static final double INTAKE_STOW                      = 163.0;
  public static final double INTAKE_STOW_UPRIGHT_AMP          = 92.6;
  public static final double INTAKE_OFFSET_FROM_ZERO          = 160.0;
  public static final double INTAKE_POSE_DOWNRIGHT            = -60;

  private static final double NULL_INTAKE_POSITION = -999.0;

  private final double MANUAL_HOLD_STEP_COEFFICIENT = 2.0;

  //crash check constants
  private final static double ELEVATOR_THRESHOLD_FOR_INTAKE_REV              = 10.0;
  private final static double TURRET_THRESHOLD_FOR_INTAKE_DEG                = 17.0;
  private final static double ELEVATOR_THRESHOLD_FOR_AMP_TRANSITION_REV      = 32.0;



  //pivot variables
  private double m_pivotManualPwr = 0.0;

  private double m_pidVolts = 0.0;
  private double m_ffVolts = 0.0;
  private double m_finalVolts = 0.0;

  private double m_targetPositionDeg = 0.0;
  private double m_currentPositionDeg = 0.0;
  private double m_previousCurrentDeg = 0.0;

  private int m_iterationCounter;

  private double positionError = 0.0;
  private double pivotVelRadPerSec = 0.0;

  private double elevatorThresholdRev = 0.0;

  private CatzMechanismPosition m_targetPosition = null;


  private static IntakeControlState m_currentIntakeControlState = IntakeControlState.AUTO;
  public static enum IntakeControlState {
    AUTO,
    SEMI_MANUAL,
    FULL_MANUAL
  }

  public static enum IntakeMechanismWaitStates {
    IN_POSITION,
    NOT_IN_POSITION
  }
  private static IntakeMechanismWaitStates m_intakeInPosition         = IntakeMechanismWaitStates.NOT_IN_POSITION;

  private boolean hasElevatorPassedClearanceCheck = false;
  private boolean isTurretInIntakeStowRange = false;
  private boolean hasElevatorPassedDrivetrainClearanceCheck = false;

  private Timer rollerTimer = new Timer();

  //tunning numbers
  LoggedTunableNumber speednumber = new LoggedTunableNumber("roller speed out", 0.0);
  LoggedTunableNumber kgtunning = new LoggedTunableNumber("kgtunningVolts",0.0);
  LoggedTunableNumber kftunning = new LoggedTunableNumber("kFtunningVolts",0.0);


  //-------------------------------------------------------------------------------------
  //
  //  SubsystemCatzIntake()
  //
  //-------------------------------------------------------------------------------------
  private SubsystemCatzIntake() {

    switch (CatzConstants.currentMode) {
      case REAL: io = new IntakeIOReal();
                 System.out.println("Intake Configured for Real");
      break;

      case REPLAY: io = new IntakeIOReal() {};
                   System.out.println("Intake Configured for Replayed simulation");
      break;

      case SIM:
      default: io = null;
               System.out.println("Intake Unconfigured");
      break;
    }
  }

  // Get the singleton instance of the intake Subsystem
  public static SubsystemCatzIntake getInstance() {
      return instance;
  }

  //-------------------------------------------------------------------------------------
  //
  //  periodic()
  //
  //-------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("intake/inputs", inputs);   

      //collect ff variables and pid variables
    m_currentPositionDeg = calcWristAngleDeg();
    positionError        = m_currentPositionDeg - m_targetPositionDeg;
    pivotVelRadPerSec    = Math.toRadians(m_currentPositionDeg - m_previousCurrentDeg)/0.02;

    //voltage control calculation
    m_ffVolts    = calculatePivotFeedFoward(Math.toRadians(m_currentPositionDeg), pivotVelRadPerSec, 0);

    if(DriverStation.isDisabled()) {
      setRollersOff();
      setPivotDisabled();
    } else { 
      //robot enabled

      //---------------------------------------Intake Roller logic -------------------------------------------
      switch(m_currentRollerState) {
        case ROLLERS_IN_SOURCE:
            if(inputs.isIntakeBeamBrkBroken || rollerTimer.hasElapsed(4.0)) {
              setRollersOff();
            }       
            break;
        case ROLLERS_IN_GROUND:
            if(inputs.isIntakeBeamBrkBroken || rollerTimer.hasElapsed(4.0)) {
              setRollersOff();
            }       
            break;
        case ROLLERS_OUT_EJECT:

            if(rollerTimer.hasElapsed(2.0)) {
              setRollersOff();
            }
            break; 
        case ROLLERS_OUT_SHOOTER_HANDOFF:

            if(rollerTimer.hasElapsed(2.0)) {
              setRollersOff();
            }            
            break;
      }

      //---------------------------------------Intake Pivot logic -------------------------------------------
      if(isTurretInIntakeStowRange == false) { //when intake is waiting for elevator change state to auto to wait in holding position

        io.setIntakePivotPostionRev(SubsystemCatzIntake.INTAKE_STOW_UPRIGHT_AMP * INTAKE_PIVOT_MTR_REV_PER_DEG, m_ffVolts);
        if(SubsystemCatzTurret.getInstance().getTurretAngle() < TURRET_THRESHOLD_FOR_INTAKE_DEG) {
          isTurretInIntakeStowRange = true;
        }
      } 

      if(hasElevatorPassedClearanceCheck == false) {

        io.setIntakePivotPostionRev(SubsystemCatzIntake.INTAKE_STOW_UPRIGHT_AMP * INTAKE_PIVOT_MTR_REV_PER_DEG, m_ffVolts);
        if(SubsystemCatzElevator.getInstance().getElevatorRevPos() < ELEVATOR_THRESHOLD_FOR_INTAKE_REV) {
          hasElevatorPassedClearanceCheck = true;
        }

      } 

      if(hasElevatorPassedDrivetrainClearanceCheck == false) {
        
        io.setIntakePivotPostionRev(SubsystemCatzIntake.INTAKE_STOW_UPRIGHT_AMP * INTAKE_PIVOT_MTR_REV_PER_DEG, m_ffVolts);
        if(SubsystemCatzElevator.getInstance().getElevatorRevPos() > ELEVATOR_THRESHOLD_FOR_AMP_TRANSITION_REV) {
          hasElevatorPassedClearanceCheck = true;
        }
      }
                
      if(isTurretInIntakeStowRange && 
         hasElevatorPassedClearanceCheck &&
         hasElevatorPassedDrivetrainClearanceCheck) { 
            //IntakTurret and elevator in position

          //If in manual mode set pwr through percent output, otherwise set pwr through pid
          if(m_currentIntakeControlState == IntakeControlState.FULL_MANUAL) { 

            io.setIntakePivotPercentOutput(m_pivotManualPwr);
          } else { 

            //in position checker
            if(Math.abs(positionError) < 5.0) { //for normal positions
              //stow cut off
              if(m_targetPositionDeg == INTAKE_STOW) {
                io.setIntakePivotVoltage(0.0);
              }
              
              //bounce checker
              m_iterationCounter++;
              if (m_iterationCounter >= 5) {
                m_intakeInPosition = IntakeMechanismWaitStates.IN_POSITION;  
              }

            } else {
              io.setIntakePivotPostionRev(m_targetPositionDeg * INTAKE_PIVOT_MTR_REV_PER_DEG, m_ffVolts);
              m_iterationCounter = 0; //resetcounter if intake hasn't leveled off
            }          
          }
      }
    } 
    m_previousCurrentDeg = m_currentPositionDeg;
    
    Logger.recordOutput("intake/finalPivotVoltage", m_finalVolts);
    Logger.recordOutput("intake/ff volts", m_ffVolts);
    Logger.recordOutput("intake/pid volts", m_pidVolts);
    Logger.recordOutput("intake/pivotvel", pivotVelRadPerSec);
    Logger.recordOutput("intake/position error", positionError);
    Logger.recordOutput("intake/targetAngle", m_targetPositionDeg);
    Logger.recordOutput("intake/currentAngle", m_currentPositionDeg);
    Logger.recordOutput("intake/roller mode", m_currentRollerState.toString());
    Logger.recordOutput("intake/intake mode", m_currentIntakeControlState.toString());


} 
  //-------------------------------------------------------------------------------------
  //  Pivot Methods
  //-------------------------------------------------------------------------------------

  //auto                                          //target source elve pickup
  public void updateTargetPositionIntake(CatzMechanismPosition targetPosition) {

    m_iterationCounter = 0; //reset counter for intake in position
    this.m_targetPosition = targetPosition;

    //if intake target position is between upright and the ground
    if(targetPosition.getIntakePivotTargetAngle() < 100 && 
       targetPosition.getIntakePivotTargetAngle() > -30) {

        hasElevatorPassedClearanceCheck = true;
        hasElevatorPassedDrivetrainClearanceCheck = true;
        isTurretInIntakeStowRange = true;
    
      //if the intake is attempting to reach Amp transition
    } else if(targetPosition.getIntakePivotTargetAngle() < -30) {
      if(m_currentPositionDeg > 100) { //if the current position is towards stow
        hasElevatorPassedClearanceCheck = false;
        hasElevatorPassedDrivetrainClearanceCheck = false;
        isTurretInIntakeStowRange = false;
      } else { //intake is fully out
        hasElevatorPassedClearanceCheck = true;
        hasElevatorPassedDrivetrainClearanceCheck = false;
        isTurretInIntakeStowRange = true;
      }

    } else if(targetPosition.getIntakePivotTargetAngle() > 100) {  //stow type angles

      if(m_currentPositionDeg < 100 && //if intake is deployed
         m_currentPositionDeg > -30) {
        hasElevatorPassedClearanceCheck = false;
        hasElevatorPassedDrivetrainClearanceCheck = true;
        isTurretInIntakeStowRange = false;
      } else if(m_currentPositionDeg >= 100) { //if intake in stow
        hasElevatorPassedClearanceCheck = true;
        hasElevatorPassedDrivetrainClearanceCheck = true;
        isTurretInIntakeStowRange = true;
      } else { //if intake is in amp transition
        hasElevatorPassedClearanceCheck = false;
        hasElevatorPassedDrivetrainClearanceCheck = false;
        isTurretInIntakeStowRange = false;
      }
    } 

  }

  //semi manual
  public void pivotSemiManual(double semiManualPwr) {
    if (semiManualPwr > 0) {
      m_targetPositionDeg = Math.min((m_targetPositionDeg + semiManualPwr * MANUAL_HOLD_STEP_COEFFICIENT),
              150); //stow position bound
    } else {
      m_targetPositionDeg = Math.max((m_targetPositionDeg + semiManualPwr * MANUAL_HOLD_STEP_COEFFICIENT),
              -60); //full deploy to ground bound
    }

    m_currentIntakeControlState = IntakeControlState.SEMI_MANUAL;
  }

  //full manual
  public void pivotFullManual(double fullManualPwr) {
    m_pivotManualPwr = 0.4 * fullManualPwr;
    m_currentIntakeControlState = IntakeControlState.FULL_MANUAL;
  }

  public void setPivotDisabled() {
    io.setIntakePivotPercentOutput(0.0);
    m_targetPositionDeg = NULL_INTAKE_POSITION;
    m_targetPosition = null;
  }

  //set modified current lmit for amp scoring
  public void setSquishyMode(boolean set) {
    io.setSquishyMode(set);
  }

  //-------------------------------------------------------------------------------------
  // Intake Calculation Methods
  //-------------------------------------------------------------------------------------
  private double calculatePivotFeedFoward(double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {
      double finalFF = PIVOT_FF_kS * Math.signum(velocityRadPerSec)
                     + PIVOT_FF_kG * Math.cos(positionRadians)
                     + PIVOT_FF_kV * velocityRadPerSec
                     + PIVOT_FF_kA * accelRadPerSecSquared;
    return finalFF;
  };

  private double calcWristAngleDeg() {
    double wristAngle = inputs.pivotMtrRev/INTAKE_PIVOT_MTR_REV_PER_DEG;
    return wristAngle;
  }

  //-------------------------------------------------------------------------------------
  // Intake getters
  //-------------------------------------------------------------------------------------
  public double getWristAngle() {
    return m_currentPositionDeg;
  }

  public IntakeMechanismWaitStates getIstIntakeInPosition() {
    return m_intakeInPosition;
  }

  public IntakeRollerState getIntakeRollerState() {
    return m_currentRollerState;
  }

  public boolean getIntakeBeamBreakBroken() {
    return inputs.isIntakeBeamBrkBroken;
  }

  //-------------------------------------------------------------------------------------
  // Roller Methods
  //-------------------------------------------------------------------------------------
  public Command cmdRollerIn() {
    return runOnce(()-> setRollersGround());
  }

  public Command cmdRollerOut() {
    return runOnce(()-> setRollersOutakeShoot());
  }

  public Command cmdRollerOff() {
    return runOnce(()->  setRollersOff());
  }

  public void setRollerState(IntakeRollerState rollerRunningMode) {
    m_currentRollerState = rollerRunningMode;
  }

  public void setRollersGround() {
    rollerTimer.restart();
    io.setRollerPercentOutput(ROLLERS_MTR_PWR_IN_GROUND);
    m_currentRollerState = IntakeRollerState.ROLLERS_IN_GROUND;
  }

  public void setRollersIntakeSource() {
    rollerTimer.restart();
    io.setRollerPercentOutput(ROLLERS_MTR_PWR_IN_GROUND);
    m_currentRollerState = IntakeRollerState.ROLLERS_IN_SOURCE;
  }

  public void setRollersOutakeHandoff() {
    rollerTimer.restart();
    io.setRollerPercentOutput(ROLLERS_MTR_PWR_OUT_HANDOFF);
    m_currentRollerState = IntakeRollerState.ROLLERS_OUT_SHOOTER_HANDOFF;
  }

  public void setRollersOutakeShoot() {
    rollerTimer.restart();
    io.setRollerPercentOutput(ROLLERS_MTR_PWR_OUT_EJECT);
    m_currentRollerState = IntakeRollerState.ROLLERS_OUT_EJECT;
  }

  public void setRollersOff() {
    io.setRollerPercentOutput(0.0);
    m_currentRollerState = IntakeRollerState.ROLLERS_OFF;
  }

}
