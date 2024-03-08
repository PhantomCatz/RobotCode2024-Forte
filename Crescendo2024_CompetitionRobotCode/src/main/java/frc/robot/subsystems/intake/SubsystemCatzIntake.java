// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.NoteDestination;
import frc.robot.Robot.manipulatorMode;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.Utils.LoggedTunableNumber;
import frc.robot.commands.mechanismCmds.MoveToNewPositionCmd;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterNoteState;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterServoState;
import frc.robot.subsystems.turret.SubsystemCatzTurret;
import frc.robot.subsystems.turret.SubsystemCatzTurret.TurretState;


public class SubsystemCatzIntake extends SubsystemBase {
  //intake io block
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  //intake instance
  private static SubsystemCatzIntake instance = new SubsystemCatzIntake();

  LoggedTunableNumber speednumber = new LoggedTunableNumber("roller speed out", 0.0);

  private CatzMechanismPosition m_targetPosition = null;
 /************************************************************************************************************************
  * 
  * rollers
  *
  ************************************************************************************************************************/
  private final double ROLLERS_MTR_PWR_IN  = 0.25;//0.6;// 0.25;//0.4;
  private final double ROLLERS_MTR_PWR_OUT_FULL_EJECT = -0.7; //Make different output powers for //-0.4 for handoff //-0.7 for amp vertical scoring
    private final double ROLLERS_MTR_PWR_OUT_HANDOFF = -0.4; //Make different output powers for //-0.4 for handoff 


  private IntakeRollerState currentRollerState;
  public static enum IntakeRollerState {
    ROLLERS_IN,
    ROLLERS_OUT_FULL_EJECT,
    ROLLERS_OUT_HANDOFF_EJECT,
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

  public static final double INTAKE_VOLT_LIMIT = 3.5;

  public final double PIVOT_FF_kS = 0.00;
  public final double PIVOT_FF_kG = 0.437;
  public final double PIVOT_FF_kV = 0.00;
  public final double PIVOT_FF_kA = 0.0;


  private PIDController pivotPID;
  private ArmFeedforward pivotFeedFoward;

  private static final double PIVOT_PID_kP = 0.05; //0.044
  private static final double PIVOT_PID_kI = 0.000; //0.005 
  private static final double PIVOT_PID_kD = 0.000; 

  private final double PID_FINE_GROSS_THRESHOLD_DEG = 20;
  private final double ERROR_INTAKE_THRESHOLD_DEG = 5.0;

  //Intake positions
  public static final double INTAKE_GROUND_PICKUP             = -22.0;
  public static final double INTAKE_SCORE_AMP                 = 125;//92.6; //90.43; //97 with drivetrain inner rail to the bottom inner rail 7 1/4 inches
  public static final double INTAKE_STOW                      = 160.0;
  public static final double INTAKE_OFFSET_FROM_ZERO          = 164.0;

  private final double STOW_CUTOFF = INTAKE_OFFSET_FROM_ZERO - 4; //TBD need to dial in
  private final double GROUND_CUTTOFF = 200;

  private final double MANUAL_HOLD_STEP_COEFFICIENT = 2.0;

  private final double STOW_ENC_POS = 0.0;
  private final double ANGLE_AMP_SCORING = 0.0;
  private final double ANGLE_GROUND_INTAKE = 0.0; //TBD need to dial in on wednesday
  private final double NULL_INTAKE_POSITION = -999.0;

  private final double GRAVITY_KG_OFFSET = 0.0;//9.0;

  private final double ELEVATOR_THRESHOLD_FOR_INTAKE = 10.0;

  //pivot variables
  private double m_pivotManualPwr = 0.0;

  private double m_pidVolts = 0.0;
  private double m_ffVolts = 0.0;
  private double m_finalVolts = 0.0;

  private double m_targetPositionDeg = 0.0;
  private double m_currentPositionDeg = 0.0;
  private double m_previousCurrentDeg = 0.0;

  private double m_iterationCounter;
  private boolean m_intakeInPosition;

  private double positionError = 0.0;
  private double pivotVelRadPerSec = 0.0;

  private static IntakeState currentIntakeState = IntakeState.IN_POSITION;
  public static enum IntakeState {
    AUTO,
    SEMI_MANUAL,
    FULL_MANUAL,
    WAITING_FOR_ELEVATOR,
    IN_POSITION
  }


  LoggedTunableNumber kgtunning = new LoggedTunableNumber("kgtunningVolts",0.0);
  LoggedTunableNumber kftunning = new LoggedTunableNumber("kFtunningVolts",0.0);



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

    pivotPID = new PIDController(PIVOT_PID_kP, 
                                 PIVOT_PID_kI, 
                                 PIVOT_PID_kD);

    pivotFeedFoward = new ArmFeedforward(PIVOT_FF_kS,
                                         PIVOT_FF_kG,
                                         PIVOT_FF_kV);
  }

  // Get the singleton instance of the intake Subsystem
  public static SubsystemCatzIntake getInstance() {
      return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("intake/inputs", inputs);   

      //collect ff variables and pid variables
    m_currentPositionDeg = calcWristAngleDeg();
    positionError = m_currentPositionDeg - m_targetPositionDeg;
    pivotVelRadPerSec = Math.toRadians(m_currentPositionDeg - m_previousCurrentDeg)/0.02;

    //voltage control calculation
    m_ffVolts    = calculatePivotFeedFoward(Math.toRadians(m_currentPositionDeg + GRAVITY_KG_OFFSET), pivotVelRadPerSec, 0);

    if(DriverStation.isDisabled()) {
      io.setRollerPercentOutput(0.0);
      currentRollerState = IntakeRollerState.ROLLERS_OFF;
      io.setIntakePivotPercentOutput(0.0);
      m_targetPositionDeg = NULL_INTAKE_POSITION;
      m_targetPosition = null;
    } else { 
      //robot enabled

      //---------------------------------------Intake Roller logic -------------------------------------------

      if(m_targetPosition == CatzConstants.CatzMechanismConstants.NOTE_POS_SHOOTER_HANDOFF) { //speaker handoff check
        //if turret/shooterservos/Intake is in position 
        if(currentIntakeState == IntakeState.IN_POSITION && 
          SubsystemCatzTurret.getInstance().getTurretState() == TurretState.IN_POSITION &&
          SubsystemCatzShooter.getInstance().getShooterServoState() == ShooterServoState.IN_POSITION) {
            currentRollerState = IntakeRollerState.ROLLERS_OUT_FULL_EJECT;
        } 

        //if note is in position shut off rollers
        if(SubsystemCatzShooter.getInstance().getShooterNoteState() == ShooterNoteState.NOTE_IN_POSTION) {
            currentRollerState = IntakeRollerState.ROLLERS_OFF;
        }
        
      } else if(m_targetPosition == CatzConstants.CatzMechanismConstants.NOTE_POS_HANDOFF_AMP_PREP) { //amp handoff check
          //if turret/shooterservos/Intake is in position 
        if(currentIntakeState == IntakeState.IN_POSITION && 
          SubsystemCatzTurret.getInstance().getTurretState() == TurretState.IN_POSITION &&
          SubsystemCatzShooter.getInstance().getShooterServoState() == ShooterServoState.IN_POSITION) {
            currentRollerState = IntakeRollerState.ROLLERS_IN;
        } 
      }

      switch(currentRollerState) {
        case ROLLERS_IN:
            if(inputs.isIntakeBeamBrkBroken) {
              currentRollerState = IntakeRollerState.ROLLERS_OFF; 

              //if the current pos is the ground
              if(m_targetPosition == CatzMechanismConstants.NOTE_POS_INTAKE_GROUND) {
                if(CatzConstants.targetNoteDestination == NoteDestination.SPEAKER) {
                  m_targetPosition = CatzMechanismConstants.NOTE_POS_SHOOTER_HANDOFF;
                  SubsystemCatzElevator.getInstance().updateElevatorTargetPosition(m_targetPosition);
                  SubsystemCatzTurret.getInstance().updateTurretTargetPosition(m_targetPosition);
                  SubsystemCatzShooter.getInstance().updateShooterTargetPosition(m_targetPosition);
                  updateIntakeTargetPosition(m_targetPosition);

                } else if(CatzConstants.targetNoteDestination == NoteDestination.AMP){
                  m_targetPosition = CatzMechanismConstants.NOTE_POS_HANDOFF_AMP_PREP;
                  SubsystemCatzElevator.getInstance().updateElevatorTargetPosition(m_targetPosition);
                  SubsystemCatzTurret.getInstance().updateTurretTargetPosition(m_targetPosition);
                  SubsystemCatzShooter.getInstance().updateShooterTargetPosition(m_targetPosition);
                  updateIntakeTargetPosition(m_targetPosition);

                }
              } 
            } else {
              io.setRollerPercentOutput(ROLLERS_MTR_PWR_IN);
            }        
            break;
        case ROLLERS_OUT_FULL_EJECT:
              io.setRollerPercentOutput(ROLLERS_MTR_PWR_OUT_FULL_EJECT);
            break; 
        case ROLLERS_OUT_HANDOFF_EJECT:
              io.setRollerPercentOutput(ROLLERS_MTR_PWR_OUT_HANDOFF);
            break; 
        case ROLLERS_OFF:
            io.setRollerPercentOutput(0.0);
            break;
      }

      //---------------------------------------Intake Pivot logic -------------------------------------------
      if(currentIntakeState == IntakeState.WAITING_FOR_ELEVATOR) { //when intake is waiting for elevator change state to auto to wait in holding position
          io.setIntakePivotEncOutput(m_targetPositionDeg * INTAKE_PIVOT_MTR_REV_PER_DEG, m_ffVolts); //go to holding position

        if(SubsystemCatzElevator.getInstance().getElevatorRevPos() < ELEVATOR_THRESHOLD_FOR_INTAKE) {
          this.m_targetPositionDeg = m_targetPosition.getIntakePivotTargetAngle();
          currentIntakeState = IntakeState.AUTO;
        }

      } else if ((currentIntakeState == IntakeState.AUTO || 
                  currentIntakeState == IntakeState.SEMI_MANUAL ||
                  currentIntakeState == IntakeState.IN_POSITION) && 
                  m_targetPositionDeg != NULL_INTAKE_POSITION) { 
        //in position checker
        if(Math.abs(positionError) < 5.0) { //for normal positions
          //stow cut off
          if(m_targetPositionDeg == INTAKE_STOW) {
            io.setIntakePivotVoltage(0.0);
          }
          
          //bounce checker
          m_iterationCounter++;
          if (m_iterationCounter >= 5) {
            currentIntakeState = IntakeState.IN_POSITION;  
          }

        } else {
          io.setIntakePivotEncOutput(m_targetPositionDeg * INTAKE_PIVOT_MTR_REV_PER_DEG, m_ffVolts);
          m_iterationCounter = 0; //resetcounter if intake hasn't leveled off
        }
        
      } else { //currently setting pwr through manual
        io.setIntakePivotVoltage(kgtunning.get());
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
    Logger.recordOutput("intake/roller mode", currentRollerState.toString());
    Logger.recordOutput("intake/intake mode", currentIntakeState.toString());


} 
  //-------------------------------------------------------------------------------------
  // Intake Access Methods
  //-------------------------------------------------------------------------------------

  //auto
  public void updateIntakeTargetPosition(CatzMechanismPosition targetPosition) {
    m_iterationCounter = 0; //reset counter for intake in position
    this.m_targetPosition = targetPosition;

    //elevator intake crash zone checks
    if(SubsystemCatzElevator.getInstance().getElevatorRevPos() > ELEVATOR_THRESHOLD_FOR_INTAKE &&
       targetPosition.getIntakePivotTargetAngle() > 100) {
      currentIntakeState = IntakeState.WAITING_FOR_ELEVATOR;
      this.m_targetPositionDeg = SubsystemCatzIntake.INTAKE_SCORE_AMP; //set the target to a holding position
    } else { //intake is free to move
      this.m_targetPositionDeg = targetPosition.getIntakePivotTargetAngle();
      currentIntakeState = IntakeState.AUTO;
      //if we need to go to amp prep but note is in shooter
      if(m_targetPosition == CatzMechanismConstants.NOTE_POS_HANDOFF_AMP_PREP) {
        if(SubsystemCatzShooter.getInstance().getShooterNoteState() == ShooterNoteState.NOTE_IN_POSTION) {
          m_targetPositionDeg = INTAKE_STOW;
        }
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

    currentIntakeState = IntakeState.SEMI_MANUAL;
  }

  //full manual
  public void pivotFullManual(double fullManualPwr) {
    m_pivotManualPwr = 0.4 * fullManualPwr;
    currentIntakeState = IntakeState.FULL_MANUAL;

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

  public IntakeState getIntakeState() {
    return currentIntakeState;
  }

  //-------------------------------------------------------------------------------------
  // Roller Methods
  //-------------------------------------------------------------------------------------
  public Command cmdRollerIn() {
    return runOnce(()-> setRollerState(IntakeRollerState.ROLLERS_IN));
  }

  public Command cmdRollerOut() {
    return runOnce(()-> setRollerState(IntakeRollerState.ROLLERS_OUT_FULL_EJECT));
  }

  public Command cmdRollerOff() {
    return runOnce(()->  setRollerState(IntakeRollerState.ROLLERS_OFF));
  }

  public void setRollerState(IntakeRollerState rollerRunningMode) {
    currentRollerState = rollerRunningMode;
  }

}
