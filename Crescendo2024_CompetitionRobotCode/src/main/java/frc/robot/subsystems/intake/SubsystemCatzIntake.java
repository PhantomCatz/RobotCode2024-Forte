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
import frc.robot.subsystems.intake.SubsystemCatzIntake.IntakeControlState;
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
  private final double ROLLERS_MTR_PWR_IN_GROUND      =  0.4;//0.6;//TBD - need to handle carpet and non-carpet value or code issue
  private final double ROLLERS_MTR_PWR_IN_SOURCE      =  0.25;
  private final double ROLLERS_MTR_PWR_OUT_EJECT      = -1.0;  //TBD fix top rooler before testing
  private final double ROLLERS_MTR_PWR_OUT_AMP_SCORE  = -0.6;  
  private final double ROLLERS_MTR_PWR_OUT_HANDOFF    = -0.4; 


  public static enum IntakeRollerState {
    ROLLERS_IN_SOURCE,
    ROLLERS_IN_GROUND,
    ROLLERS_OUT_EJECT,
    ROLLERS_OUT_SHOOTER_HANDOFF,
    ROLLERS_OFF
  }
  private IntakeRollerState m_currentRollerState;

  //-----------------------------------------------------------------------------------------------
  // 
  //  Pivot Definitions & Variables
  //
  //-----------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------
  //  Pivot Gearbox Def
  //-----------------------------------------------------------------------------------------------
  private static final double INTAKE_PIVOT_DRIVEN_GEAR      = 52.0;
  private static final double INTAKE_PIVOT_DRIVING_GEAR     = 30.0;

  private static final double INTAKE_PIVOT_DRIVEN_SPROCKET  = 32.0;
  private static final double INTAKE_PIVOT_DRIVING_SPROCKET = 16.0;

  private static final double MAX_PLANETARY_RATIO           = 5.0;

  private static final double INTAKE_PIVOT_GEAR_RATIO = (INTAKE_PIVOT_DRIVEN_GEAR     / INTAKE_PIVOT_DRIVING_GEAR) * 
                                                        (INTAKE_PIVOT_DRIVEN_SPROCKET / INTAKE_PIVOT_DRIVING_SPROCKET) * 
                                                        (MAX_PLANETARY_RATIO); 

  private static final double INTAKE_PIVOT_MTR_REV_PER_DEG       = INTAKE_PIVOT_GEAR_RATIO / 360.0;

  private static final double INTAKE_PIVOT_MTR_POS_OFFSET_IN_DEG = 164.09;

  public static final double INTAKE_PIVOT_MTR_POS_OFFSET_IN_REV  = INTAKE_PIVOT_MTR_POS_OFFSET_IN_DEG * INTAKE_PIVOT_MTR_REV_PER_DEG;

  //-----------------------------------------------------------------------------------------------
  //  Pivot Closed Loop Processing (PIDF, etc)
  //-----------------------------------------------------------------------------------------------
  public static final double PIVOT_PID_kP = 9.00; //0.044
  public static final double PIVOT_PID_kI = 0.00; //0.005 
  public static final double PIVOT_PID_kD = 0.27; 

  public        final double PIVOT_FF_kS = 0.00;
  public        final double PIVOT_FF_kG = 0.437;
  public        final double PIVOT_FF_kV = 0.00;
  public        final double PIVOT_FF_kA = 0.00;

  private double m_pidVolts   = 0.0;
  private double m_ffVolts    = 0.0;
  private double m_finalVolts = 0.0;

  private double m_targetPositionDeg  = 0.0;
  private double m_nextTargetPositionDeg = INTAKE_NULL_DEG;
  private double m_currentPositionDeg = 0.0;
  private double m_previousCurrentDeg = 0.0;

  private int    m_iterationCounter;

  private double positionErrorDeg     = 0.0;      
  private double pivotVelRadPerSec = 0.0;

  //-----------------------------------------------------------------------------------------------
  //  Control Mode Defs
  //-----------------------------------------------------------------------------------------------
  public static enum IntakeControlState {
    AUTO,
    SEMI_MANUAL,  //TBD or Manual Hold?
    FULL_MANUAL
  }
  private static IntakeControlState m_currentIntakeControlState = IntakeControlState.AUTO;
  
  private final double SEMI_MANUAL_STEP_COEFFICIENT = 2.0;    
  
  private       double m_pivotManualPwr = 0.0;



  //-----------------------------------------------------------------------------------------------
  //  Intake position defs & variables
  //-----------------------------------------------------------------------------------------------
  public static final double INTAKE_STOW_DEG                  = 163.0;    
  public static final double INTAKE_AMP_SCORE_DN_DEG          = 92.6;    //92.6; //90.43; //97 with drivetrain inner rail to the bottom inner rail 7 1/4 inches
  public static final double INTAKE_SOURCE_LOAD_UP_DEG        = 125.0; 
  public static final double INTAKE_GROUND_PICKUP_DEG         = -22.0;
  public static final double INTAKE_AMP_SCORE_DEG             = -22.0;    
  public static final double INTAKE_AMP_TRANSITION_DEG        = -60.0;    

  private static final double INTAKE_NULL_DEG                 = -999.0;   

  private final static double INTAKE_TURRET_MAX_ANGLE_FOR_HANDOFF_DEG    = 17.0;  
  public final static double INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV = 10.0;
  private final static double INTAKE_ELEV_MIN_HEIGHT_FOR_AMP_TRANS_REV   = 32.0;


  public final static double INTAKE_STOW_ELEV_CLEARED_DEG = 120.0;

  private double elevatorThresholdRev = 0.0;
  private double nextElevatorThresholdRev = 0.0;


  private static boolean m_intakeTurretInSafetyZone   = false;
  private static boolean m_intakeElevatorInSafetyZone = false;

  private static boolean m_intakeInPosition         = false;


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
 boolean isTransferingToHighPosition = false;
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
    positionErrorDeg        = m_currentPositionDeg - m_targetPositionDeg;
    //pivotVelRadPerSec    = Math.toRadians(m_currentPositionDeg - m_previousCurrentDeg)/0.02;

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
            if(inputs.isIntakeBeamBrkBroken) {
              setRollersOff();
            }       
            break;
        case ROLLERS_IN_GROUND:
            if(inputs.isIntakeBeamBrkBroken) {
              setRollersOff();
            }       
            break;
        case ROLLERS_OUT_EJECT:

            if(rollerTimer.hasElapsed(0.5)) {
              setRollersOff();
            }
            break; 
        case ROLLERS_OUT_SHOOTER_HANDOFF:

            if(rollerTimer.hasElapsed(0.5)) {
              setRollersOff();
            }  
            break;
        case ROLLERS_OFF:        
            break;
      }

      //---------------------------------------Intake Pivot logic -------------------------------------------
      //-------------------------------------------------------------------------------------------
      //  Check if Turret & Elevator are in position such that the intake will not collide with
      //  either mechanism when it is commanded to target position.  For example, Turret must be 
      //  in Home position before we stow the intake to avoid damage to intake rollers
      //-------------------------------------------------------------------------------------------
      if(m_intakeTurretInSafetyZone == false) {
        
        if(Math.abs(SubsystemCatzTurret.getInstance().getTurretAngle()) < INTAKE_TURRET_MAX_ANGLE_FOR_HANDOFF_DEG) {  
          m_intakeTurretInSafetyZone = true;
        }
      } 

      if(m_intakeElevatorInSafetyZone == false) {

        if(m_targetPositionDeg ==  INTAKE_AMP_TRANSITION_DEG) {
          //-----------------------------------------------------------------------------------
          //  intake going to amp transition
          //----------------------------------------------------------------------------------- 
          if(SubsystemCatzElevator.getInstance().getElevatorRevPos() > elevatorThresholdRev) {
            m_intakeElevatorInSafetyZone = true;
          }
        } else {
          //-----------------------------------------------------------------------------------
          //  intake coming from stow type angles
          //----------------------------------------------------------------------------------- 
          if(SubsystemCatzElevator.getInstance().getElevatorRevPos() < elevatorThresholdRev) {
            m_intakeElevatorInSafetyZone = true;
          if(m_targetPositionDeg == INTAKE_SOURCE_LOAD_UP_DEG && 
             m_nextTargetPositionDeg == INTAKE_STOW_DEG) {
              isTransferingToHighPosition = true;
            }
          }
        }
      }

      if(m_targetPositionDeg == INTAKE_SOURCE_LOAD_UP_DEG && 
         m_nextTargetPositionDeg == INTAKE_STOW_DEG &&
         isTransferingToHighPosition == false) {
          io.setIntakePivotPostionRev(INTAKE_SOURCE_LOAD_UP_DEG * INTAKE_PIVOT_MTR_REV_PER_DEG, m_ffVolts);

        }

      
      if(m_intakeTurretInSafetyZone && m_intakeElevatorInSafetyZone) { 
          //---------------------------------------------------------------------------------------
          //  Turret & Elevator are in a safe position for Intake to proceed to target position
          //---------------------------------------------------------------------------------------
          if(m_currentIntakeControlState == IntakeControlState.FULL_MANUAL) {   //TBD - Where does FULL MANUAL get set?
            //-------------------------------------------------------------------------------------
            //  Manual Control Mode - Use operator input to change intake angle
            //-------------------------------------------------------------------------------------
            io.setIntakePivotPercentOutput(m_pivotManualPwr);   //TBD Call method?

          } else if(m_targetPositionDeg != INTAKE_NULL_DEG){ 
            //-------------------------------------------------------------------------------------
            //  AUTO or Semi Manual Control Mode - Use Closed Loop control to mote to target 
            //  position
            //-------------------------------------------------------------------------------------

            if(Math.abs(positionErrorDeg) < 5.0) { //for normal positions
              //-----------------------------------------------------------------------------------
              //  We are within acceptable range of target position.  
              //
              //  If we are heading to STOW position, we will turn off control loop and let gravity
              //  finish positioning intake on hardstop.  
              //
              //  To make sure intake has settled at target position (e.g. it wasn't passing 
              //  through or bounced outside desired threshold), we will wait until n consecutive 
              //  samples within target threshold have been seen
              //-----------------------------------------------------------------------------------
              if(m_targetPositionDeg == INTAKE_STOW_DEG) {
                io.setIntakePivotVoltage(0.0);
              }
              m_iterationCounter++;
              if (m_iterationCounter >= 5) {
                m_intakeInPosition = true;  
              }

              //-----------------------------------------------------------------------------------
              //  Chk if we were commanded to AMP_TRANS position.  If that was the case, then
              //  update target position to original commanded position
              //-----------------------------------------------------------------------------------
              if(m_nextTargetPositionDeg == INTAKE_AMP_TRANSITION_DEG ||
                  m_nextTargetPositionDeg == INTAKE_STOW_DEG) {
                updateAutoTargetPositionIntake(m_nextTargetPositionDeg);
              }
            } else {
              //-----------------------------------------------------------------------------------
              //  Not at target position yet...
              //-----------------------------------------------------------------------------------
              io.setIntakePivotPostionRev(m_targetPositionDeg * INTAKE_PIVOT_MTR_REV_PER_DEG, m_ffVolts);

              m_iterationCounter = 0;  //reset counter - we've overshot target position or bounced
            }          
          }
      }
    } 
    m_previousCurrentDeg = m_currentPositionDeg;
    
    Logger.recordOutput("intake/finalPivotVoltage", m_finalVolts);
    Logger.recordOutput("intake/ff volts", m_ffVolts);
    Logger.recordOutput("intake/pid volts", m_pidVolts);
    Logger.recordOutput("intake/pivotvel", pivotVelRadPerSec);
    Logger.recordOutput("intake/position error", positionErrorDeg);
    Logger.recordOutput("intake/targetAngle", m_targetPositionDeg);
    Logger.recordOutput("intake/currentAngle", m_currentPositionDeg);
    Logger.recordOutput("intake/roller mode", m_currentRollerState.toString());
    Logger.recordOutput("intake/intake mode", m_currentIntakeControlState.toString());


  } 

  //-----------------------------------------------------------------------------------------------
  //
  //  Pivot Control Mode Methods
  //
  //-----------------------------------------------------------------------------------------------
  public void updateAutoTargetPositionIntake(double targetPosition) {
    isTransferingToHighPosition = false;
    m_currentIntakeControlState = IntakeControlState.AUTO;
    //-------------------------------------------------------------------------------------
    //  Initialize Variables 
    //-------------------------------------------------------------------------------------
    this.m_targetPositionDeg   = targetPosition;       //TBD - Why not just save / pass in targetPositionDeg?

    m_nextTargetPositionDeg    = INTAKE_NULL_DEG;

    m_iterationCounter         = 0; //reset counter for intake in position

    m_intakeTurretInSafetyZone   = false;
    m_intakeElevatorInSafetyZone = false;

    m_intakeInPosition           = false;
  
    //-------------------------------------------------------------------------------------
    //  Based on target destination, we need to determine safe elevator position.  For
    //  Turret, the safe position is always the same and will aways be checked in periodic()
    //-------------------------------------------------------------------------------------
    if(m_targetPositionDeg > 90) { 
        //-------------------------------------------------------------------------------------
        //  If intake is already behind the elevator then elevator is already in a safe
        //  position.  If the intake is NOT behind the elevator then we need to make sure the
        //  elevator is low enough to safely pivot the intake under the cross bar to get to
        //  the stow position.  To simply things we will use the same angle for going to/from 
        //  STOW position
        //-------------------------------------------------------------------------------------
        if (m_currentPositionDeg > INTAKE_STOW_ELEV_CLEARED_DEG) {
          m_intakeElevatorInSafetyZone = true;     
          System.out.println("Int clear to stow");   
        } else {

            //bound for when the intake is in amp transitition or amp scoring
            if(m_currentPositionDeg < 80 &&
               m_targetPositionDeg != INTAKE_GROUND_PICKUP_DEG) {
              m_nextTargetPositionDeg = INTAKE_STOW_DEG;
              m_targetPositionDeg     = INTAKE_SOURCE_LOAD_UP_DEG;
            } else {

            }
              elevatorThresholdRev = INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV;
              System.out.println("Int not clear to stow");   
          
        }
    } else if(m_targetPositionDeg == INTAKE_AMP_TRANSITION_DEG) {
        //-------------------------------------------------------------------------------------
        //  There are two cases to consider based on where Note is coming from:
        //    1. SRC/Gnd Pickup
        //    2. Shooter
        //
        //  Case 1: intake is already in front of the elevator so we don't need to worry
        //  about clearing elevator cross bar.  We only need to make sure that the elevator is
        //  high enough such that we don't hit robot chassis/bumpers
        //
        //  Case 2: We are doing a Handoff so Intake, Turret AND Elevator should all be in 
        //  STOW position.  So this means the intake can pivot past the elevator cross bar 
        //  but MUST then wait for Elevator to raise up high enough to allow the intake to go
        //  to AMP_TRANS angle without hitting the robot chassis.  We will save a 'next' target
        //  position
        //
        //  save desired target pos (maybe nextTargetAngleDeg) and then set target pos to INTAKE_SRC_DN_PICKUP_ANGLE_DEG
        //  when we get to this pos, we can then check if nextTargetAngleDeg is valid and if so set a new target angle
        //  and elev threshold. Will also need a flag to determine if check is '>' or '<' than for elev
        //  (elev periodic will be waiting for intake to clear before starting to move up)
        //-------------------------------------------------------------------------------------
        if (m_currentPositionDeg < INTAKE_STOW_ELEV_CLEARED_DEG) {
           System.out.println("Int to Amp Trnas");   
            //-----------------------------------------------------------------------------------
            //  Source/ground
            //----------------------------------------------------------------------------------- 
              elevatorThresholdRev = INTAKE_ELEV_MIN_HEIGHT_FOR_AMP_TRANS_REV;
          nextElevatorThresholdRev = INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV;  

        } else {
          System.out.println("Int stow to transition");   

          //-----------------------------------------------------------------------------------
          //  Coming from handoff position (stow)
          //-----------------------------------------------------------------------------------
              elevatorThresholdRev = INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV; 
          nextElevatorThresholdRev = INTAKE_ELEV_MIN_HEIGHT_FOR_AMP_TRANS_REV;

          m_nextTargetPositionDeg = INTAKE_AMP_TRANSITION_DEG;
              m_targetPositionDeg = INTAKE_SOURCE_LOAD_UP_DEG;
        }
       } else if(m_targetPositionDeg >= 90){//m_targetPositionDeg == INTAKE_GROUND_PICKUP_DEG ||
      //           m_targetPositionDeg == INTAKE_AMP_SCORE_DN_DEG||
      //           m_targetPositionDeg == INTAKE_SOURCE_LOAD_UP_DEG||
      //           m_targetPositionDeg == INTAKE_AMP_SCORE_DEG) {
         //-------------------------------------------------------------------------------------
        //  If intake is already in front of the elevator then elevator is already in a safe
        //  position.  If the intake is NOT in front of the elevator then we need to make sure
        //  the elevator is low enough to safely pivot the intake under the cross bar to get to
        //  the target position.  To simply things we will use the same angle for going to/from 
        //  STOW position
        //-------------------------------------------------------------------------------------
        if (m_currentPositionDeg < INTAKE_STOW_ELEV_CLEARED_DEG) {
            m_intakeElevatorInSafetyZone = true;
                    
            System.out.println("Int clear to deploy");   

          } else {
            elevatorThresholdRev = INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV;
           System.out.println("Int wait for deploy");   

          }
      }

  }   //End of updateTargetPositionIntake()

  //semi manual
  public void pivotSemiManual(double semiManualPwr) {
    if (semiManualPwr > 0) {
      m_targetPositionDeg = Math.min((m_targetPositionDeg + semiManualPwr * SEMI_MANUAL_STEP_COEFFICIENT),
              150); //stow position bound
    } else {
      m_targetPositionDeg = Math.max((m_targetPositionDeg + semiManualPwr * SEMI_MANUAL_STEP_COEFFICIENT),
              -60); //full deploy to ground bound
    }

    m_currentIntakeControlState = IntakeControlState.SEMI_MANUAL;
    m_intakeInPosition = false;
  }

  //full manual
  public void pivotFullManual(double fullManualPwr) {
    m_pivotManualPwr = 0.4 * fullManualPwr;
    m_currentIntakeControlState = IntakeControlState.FULL_MANUAL;
    m_intakeInPosition = false;
  }

  public void setPivotDisabled() {
    io.setIntakePivotPercentOutput(0.0);
    m_targetPositionDeg = INTAKE_NULL_DEG;
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

  public boolean getIntakeInPos() {
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
