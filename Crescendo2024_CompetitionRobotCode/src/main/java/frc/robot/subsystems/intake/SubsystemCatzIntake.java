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
import frc.robot.CatzConstants.RobotMode;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.Utils.LoggedTunableNumber;
import frc.robot.CatzConstants.NoteDestination;
import frc.robot.CatzConstants.NoteSource;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.subsystems.intake.SubsystemCatzIntake.IntakeControlState;
import frc.robot.subsystems.turret.SubsystemCatzTurret;

public class SubsystemCatzIntake extends SubsystemBase {
  // intake io block
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // intake instance
  private static SubsystemCatzIntake instance = new SubsystemCatzIntake();

  /************************************************************************************************************************
   * 
   * rollers
   *
   ************************************************************************************************************************/
  private final double ROLLERS_MTR_PWR_IN_GROUND = 0.45;//0.6//TBD - need to handle carpet and non-carpet value or code
                                                       // issue
  private final double ROLLERS_MTR_PWR_IN_SOURCE = 0.25;
  private final double ROLLERS_MTR_PWR_OUT_EJECT = -1.0; // TBD fix top rooler before testing
  private final double ROLLERS_MTR_PWR_OUT_AMP_SCORE = 0.6;
  private final double ROLLERS_MTR_PWR_OUT_HANDOFF = -0.4;

  public static enum IntakeRollerState {
    ROLLERS_IN_SOURCE,
    ROLLERS_IN_GROUND,
    ROLLERS_IN_SCORING_AMP,
    ROLLERS_OUT_EJECT,
    ROLLERS_OUT_SHOOTER_HANDOFF,
    ROLLERS_OFF
  }

  private IntakeRollerState m_currentRollerState;

  // -----------------------------------------------------------------------------------------------
  //
  // Pivot Definitions & Variables
  //
  // -----------------------------------------------------------------------------------------------
  // -----------------------------------------------------------------------------------------------
  // Pivot Gearbox Def
  // -----------------------------------------------------------------------------------------------
  private static final double INTAKE_PIVOT_DRIVEN_GEAR = 52.0;
  private static final double INTAKE_PIVOT_DRIVING_GEAR = 30.0;

  private static final double INTAKE_PIVOT_DRIVEN_SPROCKET = 32.0;
  private static final double INTAKE_PIVOT_DRIVING_SPROCKET = 16.0;

  private static final double MAX_PLANETARY_RATIO = 5.0;

  private static final double INTAKE_PIVOT_GEAR_RATIO = (INTAKE_PIVOT_DRIVEN_GEAR / INTAKE_PIVOT_DRIVING_GEAR) *
      (INTAKE_PIVOT_DRIVEN_SPROCKET / INTAKE_PIVOT_DRIVING_SPROCKET) *
      (MAX_PLANETARY_RATIO);

  private static final double INTAKE_PIVOT_MTR_REV_PER_DEG = INTAKE_PIVOT_GEAR_RATIO / 360.0;

  private static final double INTAKE_PIVOT_MTR_POS_OFFSET_IN_DEG = 164.09;

  public static final double INTAKE_PIVOT_MTR_POS_OFFSET_IN_REV = INTAKE_PIVOT_MTR_POS_OFFSET_IN_DEG
      * INTAKE_PIVOT_MTR_REV_PER_DEG;

  // -----------------------------------------------------------------------------------------------
  // Pivot Closed Loop Processing (PIDF, etc)
  // -----------------------------------------------------------------------------------------------
  public static final double PIVOT_PID_kP = 9.00; // 0.044
  public static final double PIVOT_PID_kI = 0.00; // 0.005
  public static final double PIVOT_PID_kD = 0.27;

  public final double PIVOT_FF_kS = 0.00;
  public final double PIVOT_FF_kG = 0.437;
  public final double PIVOT_FF_kV = 0.00;
  public final double PIVOT_FF_kA = 0.00;

  private double m_ffVolts = 0.0;

  private double m_targetPositionDeg = 0.0;
  private double m_nextTargetPositionDeg = INTAKE_NULL_DEG;
  private double m_currentPositionDeg = 0.0;
  private double m_previousTargetPositionDeg = 0.0;

  private boolean isIntakeInScoreAmp;

  private int m_iterationCounter;

  private double positionErrorDeg = 0.0;
  private double pivotVelRadPerSec = 0.0;

  // -----------------------------------------------------------------------------------------------
  // Control Mode Defs
  // -----------------------------------------------------------------------------------------------
  public static enum IntakeControlState {
    AUTO,
    SEMI_MANUAL, // TBD or Manual Hold?
    FULL_MANUAL,
    VOLTAGE_CONTROL
  }

  private static IntakeControlState m_currentIntakeControlState = IntakeControlState.AUTO;

  private final double SEMI_MANUAL_STEP_COEFFICIENT = 2.0;

  private double m_pivotManualPwr = 0.0;

  // -----------------------------------------------------------------------------------------------
  // Intake position defs & variables
  // -----------------------------------------------------------------------------------------------
  public static final double INTAKE_STOW_DEG           = 163.0;
  public static final double INTAKE_SOURCE_LOAD_DN_DEG = 30.0;
  public static final double INTAKE_SOURCE_LOAD_UP_DEG =  97.0; //with drivetrain inner rail to the
                                                             // bottom inner rail 7 1/4 inches
  public static final double INTAKE_AMP_SCORE_DN_DEG   =  92.6; //90.43; 
  public static final double INTAKE_HOARD_DEG          = 30.0;
  public static final double INTAKE_GROUND_PICKUP_DEG  = -25.0; //-22.0;
  public static final double INTAKE_AMP_SCORE_DEG      = 80.0;
  public static final double INTAKE_AMP_TRANSITION_DEG = -77.0; //TBD Change to -80 on sn2

  public static final double INTAKE_MIN_ELEV_CLEARANCE_DEG = 100.0;
  public static final double INTAKE_TRANSITION_CHECK_DEG = -47.0;

  private static final double INTAKE_NULL_DEG = -999.0;

  private final static double INTAKE_TURRET_MAX_ANGLE_FOR_HANDOFF_DEG = 17.0;
  public final static double INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV = 40.0;
  private final static double INTAKE_ELEV_MIN_HEIGHT_FOR_AMP_TRANS_REV = 32.0;

  public final static double INTAKE_STOW_ELEV_CLEARED_DEG = 120.0;
  public static final double INTAKE_TURRET_CLEARANCE = 125.0;


  private double elevatorThresholdRev = 0.0;
  private double nextElevatorThresholdRev = 0.0;

  private static boolean m_intakeTurretInSafetyZone = false;
  private static boolean m_intakeElevatorInSafetyZone = false;

  private static boolean m_intakeInPosition = false;

  private boolean m_intermediatePositionReached = false;

  

  private Timer rollerTimer = new Timer();

  // tunning numbers
  LoggedTunableNumber speednumber = new LoggedTunableNumber("roller speed out", 0.0);
  LoggedTunableNumber kgtunning = new LoggedTunableNumber("kgtunningVolts", 0.0);
  LoggedTunableNumber kftunning = new LoggedTunableNumber("kFtunningVolts", 0.0);

  // -------------------------------------------------------------------------------------
  //
  // SubsystemCatzIntake()
  //
  // -------------------------------------------------------------------------------------
  private SubsystemCatzIntake() {

    switch (CatzConstants.currentMode) {
      case REAL:
        io = new IntakeIOReal();
        System.out.println("Intake Configured for Real");
        break;

      case REPLAY:
        io = new IntakeIOReal() {
        };
        System.out.println("Intake Configured for Replayed simulation");
        break;

      case SIM:
      default:
        io = null;
        System.out.println("Intake Unconfigured");
        break;
    }
  }

  // Get the singleton instance of the intake Subsystem
  public static SubsystemCatzIntake getInstance() {
    return instance;
  }

  boolean isTransferingToHighPosition = false;

  // -------------------------------------------------------------------------------------
  //
  // periodic()
  //
  // -------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("intake/inputs", inputs);

    // collect ff variables and pid variables
    m_currentPositionDeg = calcWristAngleDeg();
    positionErrorDeg = m_currentPositionDeg - m_targetPositionDeg;

    // voltage control calculation
    m_ffVolts = calculatePivotFeedFoward(Math.toRadians(m_currentPositionDeg), pivotVelRadPerSec, 0);

    if (DriverStation.isDisabled()) {
      setRollersOff();
      setPivotDisabled();
    } else {
      // robot enabled

      // ---------------------------------------Intake Roller logic -------------------------------------------
      switch (m_currentRollerState) {
        case ROLLERS_IN_SOURCE:
          if (inputs.isIntakeBeamBrkBroken) {
            setRollersOff();
          }
          break;
        case ROLLERS_IN_GROUND:
          if (inputs.isIntakeBeamBrkBroken) {
            setRollersOff();
          }
          break;
        case ROLLERS_IN_SCORING_AMP:
          break;
        case ROLLERS_OUT_EJECT:

          if (rollerTimer.hasElapsed(0.5)) {
            setRollersOff();
          }
          break;
        case ROLLERS_OUT_SHOOTER_HANDOFF:

          if (rollerTimer.hasElapsed(0.5)) {
            setRollersOff();
          }
          break;
        case ROLLERS_OFF:
          break;
      }

      // ---------------------------------------Intake Pivot logic -------------------------------------------
      if ((m_currentIntakeControlState == IntakeControlState.AUTO ||
          m_currentIntakeControlState == IntakeControlState.SEMI_MANUAL)) {
        // -------------------------------------------------------------------------------------
        // AUTO or Semi Manual Control Mode - Use Closed Loop control to mote to target
        // position
        // -------------------------------------------------------------------------------------

        if ((m_targetPositionDeg != INTAKE_NULL_DEG)) {
          // -------------------------------------------------------------------------------------------
          // Check if Turret & Elevator are in position such that the intake will not
          // collide with
          // either mechanism when it is commanded to target position. For example, Turret
          // must be
          // in Home position before we stow the intake to avoid damage to intake rollers
          // -------------------------------------------------------------------------------------------
          if (m_intakeTurretInSafetyZone == false) {

            if (Math.abs(SubsystemCatzTurret.getInstance().getTurretAngle()) < INTAKE_TURRET_MAX_ANGLE_FOR_HANDOFF_DEG) {
              m_intakeTurretInSafetyZone = true;
            }
          }

          if (m_intakeElevatorInSafetyZone == false) {
            if (m_targetPositionDeg == INTAKE_AMP_TRANSITION_DEG){  //amp transition for going up
              // -----------------------------------------------------------------------------------
              // intake going to amp transition
              // -----------------------------------------------------------------------------------
              if (SubsystemCatzElevator.getInstance().getElevatorRevPos() > elevatorThresholdRev) {
                m_intakeElevatorInSafetyZone = true;
               // System.out.println("reserve for intake amp transition");
              }
            } else {
              // -----------------------------------------------------------------------------------
              // intake coming from stow type angles
              // -----------------------------------------------------------------------------------
              if (SubsystemCatzElevator.getInstance().getElevatorRevPos() < elevatorThresholdRev) {
                m_intakeElevatorInSafetyZone = true;

                //System.out.println("Coming from stow");
              }
            }

            if(m_targetPositionDeg == INTAKE_AMP_SCORE_DN_DEG &&  //amp intermediate for going down
               m_nextTargetPositionDeg == INTAKE_STOW_DEG) {
                m_intakeElevatorInSafetyZone = true;
            }

          }

 

          if (m_intakeTurretInSafetyZone && m_intakeElevatorInSafetyZone) {
            // ---------------------------------------------------------------------------------------
            // Turret & Elevator are in a safe position for Intake to proceed to target
            // position
            // ---------------------------------------------------------------------------------------

            if (Math.abs(positionErrorDeg) < 5.0) { // for normal positions
              // -----------------------------------------------------------------------------------
              // We are within acceptable range of target position.
              //
              // If we are heading to STOW position, we will turn off control loop and let
              // gravity
              // finish positioning intake on hardstop.
              //
              // To make sure intake has settled at target position (e.g. it wasn't passing
              // through or bounced outside desired threshold), we will wait until n
              // consecutive
              // samples within target threshold have been seen
              // -----------------------------------------------------------------------------------
              if (m_targetPositionDeg == INTAKE_STOW_DEG) {
                io.setIntakePivotVoltage(0.0);
              }
              m_iterationCounter++;
              if (m_iterationCounter >= 5) {
                m_intakeInPosition = true;
                
                // -----------------------------------------------------------------------------------
                // Chk if we were commanded to AMP_TRANS position. If that was the case, then
                // update target position to original commanded position
                // -----------------------------------------------------------------------------------
                if (m_nextTargetPositionDeg == INTAKE_AMP_TRANSITION_DEG ||
                    m_nextTargetPositionDeg == INTAKE_STOW_DEG) {
                     // System.out.println("updating to transition" + m_nextTargetPositionDeg);

                  m_targetPositionDeg = m_nextTargetPositionDeg;
                  m_intermediatePositionReached = true;

                  updateAutoTargetPositionIntake(m_targetPositionDeg);

                  if(m_intermediatePositionReached == true) {
                    m_intermediatePositionReached = false;
                  }

                  m_nextTargetPositionDeg = INTAKE_NULL_DEG;
                }
              }
            } else {
              // -----------------------------------------------------------------------------------
              // Not at target position yet...
              // -----------------------------------------------------------------------------------
              io.setIntakePivotPostionRev(m_targetPositionDeg * INTAKE_PIVOT_MTR_REV_PER_DEG, m_ffVolts);

              m_iterationCounter = 0; // reset counter - we've overshot target position or bounced
            }
          }
        }
      } else if(m_currentIntakeControlState == IntakeControlState.VOLTAGE_CONTROL) {
        io.setIntakePivotVoltage(PIVOT_FF_kG);
    
      } else {
        // -------------------------------------------------------------------------------------
        // Manual Control Mode - Use operator input to change intake angle
        // -------------------------------------------------------------------------------------
        io.setIntakePivotPercentOutput(m_pivotManualPwr); // TBD Call method?

      }
    }

    // Logger.recordOutput("intake/ff volts", m_ffVolts);
    // Logger.recordOutput("intake/pivotvel", pivotVelRadPerSec);
    // Logger.recordOutput("intake/position error", positionErrorDeg);
    Logger.recordOutput("intake/targetAngle", m_targetPositionDeg);
    Logger.recordOutput("intake/currentAngle", m_currentPositionDeg);
    // Logger.recordOutput("intake/roller mode", m_currentRollerState.toString());
    // Logger.recordOutput("intake/intake mode", m_currentIntakeControlState.toString());

  }

  // -----------------------------------------------------------------------------------------------
  //
  // Pivot Control Mode Methods
  //
  // -----------------------------------------------------------------------------------------------
  public void updateAutoTargetPositionIntake(double targetPosition) {
    // System.out.println("IUP" + targetPosition);
    // -------------------------------------------------------------------------------------
    // Initialize Variables
    // -------------------------------------------------------------------------------------
    m_nextTargetPositionDeg = INTAKE_NULL_DEG;
    isTransferingToHighPosition = false;
    m_currentIntakeControlState = IntakeControlState.AUTO;

    m_iterationCounter = 0; // reset counter for intake in position

    m_intakeTurretInSafetyZone = false;
    m_intakeElevatorInSafetyZone = false;

    m_intakeInPosition = false;

    m_targetPositionDeg = targetPosition;

    // -------------------------------------------------------------------------------------
    // Based on target destination, we need to determine safe elevator position. For
    // Turret, the safe position is always the same and will aways be checked in
    // periodic()
    // -------------------------------------------------------------------------------------
    if (m_targetPositionDeg == INTAKE_STOW_DEG) {
      //System.out.println("I-A");
      // -------------------------------------------------------------------------------------
      // If intake is already behind the elevator then elevator is already in a safe
      // position. If the intake is NOT behind the elevator then we need to make sure
      // the
      // elevator is low enough to safely pivot the intake under the cross bar to get
      // to
      // the stow position. To simply things we will use the same angle for going
      // to/from
      // STOW position
      // -------------------------------------------------------------------------------------
      if(m_currentPositionDeg < INTAKE_TRANSITION_CHECK_DEG ||
          getIsIntakeInAmpScoring() ||
          SubsystemCatzElevator.getInstance().getElevatorRevPos() > INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV) {
            
        if(m_intermediatePositionReached == false) {
          m_nextTargetPositionDeg = INTAKE_STOW_DEG; 
              m_targetPositionDeg = INTAKE_AMP_SCORE_DN_DEG; // set intermediate destination
          
              elevatorThresholdRev = INTAKE_ELEV_MIN_HEIGHT_FOR_AMP_TRANS_REV;
          nextElevatorThresholdRev = INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV;
        } 
      } else {
          //System.out.println("I-BA");
        elevatorThresholdRev = INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV;
        if (SubsystemCatzElevator.getInstance().getElevatorRevPos() < INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV) {
          m_intakeElevatorInSafetyZone = true;
          //System.out.println("I-B");
        }
      }
    } else if (m_targetPositionDeg == INTAKE_GROUND_PICKUP_DEG ||
              m_targetPositionDeg == INTAKE_AMP_SCORE_DN_DEG ||
              m_targetPositionDeg == INTAKE_SOURCE_LOAD_DN_DEG ||
              m_targetPositionDeg == INTAKE_HOARD_DEG         ||
              m_targetPositionDeg == INTAKE_AMP_SCORE_DEG) {
     // System.out.println("I-E");

      // -------------------------------------------------------------------------------------
      // If intake is already in front of the elevator then elevator is already in a
      // safe
      // position. If the intake is NOT in front of the elevator then we need to make
      // sure
      // the elevator is low enough to safely pivot the intake under the cross bar to
      // get to
      // the target position. To simply things we will use the same angle for going
      // to/from
      // STOW position
      // ------------------------------------------------------------------------------------

      elevatorThresholdRev = INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV;
      if(m_currentPositionDeg < INTAKE_TRANSITION_CHECK_DEG) {
       // System.out.println("I-G");
        m_intakeElevatorInSafetyZone = true;
      } else if (SubsystemCatzElevator.getInstance().getElevatorRevPos() < INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV) {
          // System.out.println("I-F");
          m_intakeElevatorInSafetyZone = true;
      }
    }

    m_previousTargetPositionDeg = m_targetPositionDeg;

  } // End of updateTargetPositionIntake()

  // semi manual
  public void pivotSemiManual(double semiManualPwr) {
    if (semiManualPwr > 0) {
      m_targetPositionDeg = Math.min((m_targetPositionDeg + semiManualPwr * SEMI_MANUAL_STEP_COEFFICIENT),
          150); // stow position bound
    } else {
      m_targetPositionDeg = Math.max((m_targetPositionDeg + semiManualPwr * SEMI_MANUAL_STEP_COEFFICIENT),
          -60); // full deploy to ground bound
    }

    m_currentIntakeControlState = IntakeControlState.SEMI_MANUAL;
    m_intakeInPosition = false;
  }

  // full manual
  public void pivotFullManual(double fullManualPwr) {
    m_pivotManualPwr = 0.4 * fullManualPwr;
    m_currentIntakeControlState = IntakeControlState.FULL_MANUAL;
    m_intakeInPosition = false;
  }


  public void setPivotDisabled() {
    io.setIntakePivotPercentOutput(0.0);
    m_targetPositionDeg = INTAKE_NULL_DEG;
  }

  // set modified current lmit for amp scoring
  public void setSquishyMode(boolean set) {
    io.setSquishyMode(set);
  }

  // -------------------------------------------------------------------------------------
  // Intake Calculation Methods
  // -------------------------------------------------------------------------------------
  private double calculatePivotFeedFoward(double positionRadians, double velocityRadPerSec,
      double accelRadPerSecSquared) {
    double finalFF = PIVOT_FF_kS * Math.signum(velocityRadPerSec)
                  + PIVOT_FF_kG * Math.cos(positionRadians)
                  + PIVOT_FF_kV * velocityRadPerSec
                  + PIVOT_FF_kA * accelRadPerSecSquared;
    return finalFF;
  };

  private double calcWristAngleDeg() {
    double wristAngle = inputs.pivotMtrRev / INTAKE_PIVOT_MTR_REV_PER_DEG;
    return wristAngle;
  }

  // -------------------------------------------------------------------------------------
  // Intake getters
  // -------------------------------------------------------------------------------------
  public double getWristAngle() {
    return m_currentPositionDeg;
  }

  public double getWristVelocity() {
    return inputs.pivotMtrVelocityRPS;
  }

  public double getWristCurrent() {
    return inputs.pivotMtrCurrent;
  }

  public boolean getIntakeInPos() {
    return m_intakeInPosition;
  }

  public IntakeRollerState getIntakeRollerState() {
    return m_currentRollerState;
  }

  public boolean getIntakeBeamBreakBroken() {
    return inputs.isIntakeBeamBrkBroken;
  }

  public void setWasIntakeInAmpScoring(boolean set) {
    isIntakeInScoreAmp = set;
    // System.out.println("is intake in amp score" + set);
  }

  public boolean getIsIntakeInAmpScoring() {
    // System.out.println(isIntakeInScoreAmp);
    return isIntakeInScoreAmp;
  }

  // -------------------------------------------------------------------------------------
  // Roller Methods
  // -------------------------------------------------------------------------------------
  public Command cmdRollerIn() {
    return runOnce(() -> setRollersIn());
  }

  public Command cmdRollerOut() {
    return runOnce(() -> setRollersOutakeShoot());
  }

  public Command cmdRollerOff() {
    return runOnce(() -> setRollersOff());
  }

  public void setRollersIn() {
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
