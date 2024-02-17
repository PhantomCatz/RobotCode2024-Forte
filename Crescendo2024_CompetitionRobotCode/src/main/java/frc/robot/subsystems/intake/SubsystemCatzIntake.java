// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class SubsystemCatzIntake extends SubsystemBase {
  //intake io block
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  //intake instance
  private static SubsystemCatzIntake instance = new SubsystemCatzIntake();


  //intake rollers Constants
  private final double ROLLERS_MTR_PWR_IN  =  0.6;
  private final double ROLLERS_MTR_PWR_OUT = -1.0;

  //intake roller variables


  //intake pivot variables
  //constants
  private final double ENC_TO_INTAKE_GEAR_RATIO = 18;//(60 / 20)* (32 / 16);
  private final double WRIST_CNTS_PER_DEGREE = (2048.0 * ENC_TO_INTAKE_GEAR_RATIO) / 360.0;

  private final double INTAKE_ANGLE_TO_MTR_ROTATIONS = 6/360;

  private static final double GROSS_kP = 0.003; //0.003 //0.008
  private static final double GROSS_kI = 0.000; //0.00 //0.0
  private static final double GROSS_kD = 0.00025; //0.00025 0.0

  private static final double FINE_kP = 0.04; //0.003
  private static final double FINE_kI = 0.000; 
  private static final double FINE_kD = 0.000;

  private static final double HANDOFF_TO_STOW_KP = 0.008;

  private final double PID_FINE_GROSS_THRESHOLD_DEG = 20;
  private final double ERROR_INTAKE_THRESHOLD_DEG = 5.0;

  private final double STOW_CUTOFF = 0.0; //TBD need to dial in
  private final double CENTER_OF_MASS_OFFSET_DEG = 177.0;//177.0;
  private final double GRAVITY_FF_SCALING_COEFFICIENT = 0.04;

  private final double MANUAL_HOLD_STEP_COEFFICIENT = 2.0;

  private final double STOW_ENC_POS = 0.0;
  private final double ANGLE_AMP_SCORING = 0.0;
  private final double ANGLE_GROUND_INTAKE = 0.0; //TBD need to dial in on wednesday
  private final double NULL_INTAKE_POSITION = -999.0;

  //intake variables
  private double m_pivotManualPwr;
  private double m_targetPower;
  private double m_pidPower;
  private double m_ffPower;
  private double m_prevTargetPwr;
  private double m_prevCurrentPosition;
  private double m_targetPositionDeg;
  private double m_numConsectSamples;
  private boolean m_intakeInPosition;
  private int    m_rollerRunningMode;
  private double m_previousTargetAngle;
  private double m_finalEncOutput;



  private PIDController pid;


  public SubsystemCatzIntake() {

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

    pid = new PIDController(GROSS_kP, 
                            GROSS_kI, 
                            GROSS_kD);

  }

  // Get the singleton instance of the intake Subsystem
  public static SubsystemCatzIntake getInstance() {
      return instance;
  }

  private static IntakeState currentIntakeState;

  public static enum IntakeState {
    AUTO,
    SEMI_MANUAL,
    FULL_MANUAL
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("intake/inputs", inputs);   

              //collect current targetPosition in degrees
    double currentPositionDeg = calcWristAngle();
    double positionError = currentPositionDeg - m_targetPositionDeg;

    if(DriverStation.isDisabled()) {
      io.setRollerPercentOutput(0.0);
      m_rollerRunningMode = 0;
      io.setIntakePivotPercentOutput(0.0);
      m_targetPositionDeg = NULL_INTAKE_POSITION;
    } else { 
      //robot enabled

        if(m_rollerRunningMode == 2) {
            io.setRollerPercentOutput(ROLLERS_MTR_PWR_OUT); 
        } else if(m_rollerRunningMode == 1) {
              if(inputs.BeamBrkBackBroken) {
                io.setRollerPercentOutput(0.0);
              } else {
                io.setRollerPercentOutput(ROLLERS_MTR_PWR_IN);
              }
        } else {
          io.setRollerPercentOutput(0.0);
        }

      //Intake Pivot Logic
      if ((currentIntakeState == IntakeState.AUTO || 
           currentIntakeState == IntakeState.SEMI_MANUAL) && 
           m_targetPositionDeg == NULL_INTAKE_POSITION) { 

            //check if at final position using counter
        if ((Math.abs(positionError) <= ERROR_INTAKE_THRESHOLD_DEG)) {
            m_numConsectSamples++;
            if (m_numConsectSamples >= 1) {
                m_intakeInPosition = true;
            }
        } else {
            m_numConsectSamples = 0; //resetcounter if intake hasn't leveled off
        }
        
        //calculate ff pwr and and sends to mtr through motion magic
        m_ffPower = calculateGravityFF();
        m_finalEncOutput = m_targetPositionDeg * INTAKE_ANGLE_TO_MTR_ROTATIONS;

        // ----------------------------------------------------------------------------------
        // If we are going to Stow Position & have passed the power cutoff angle, set
        // power to 0, otherwise calculate new motor power based on position error and
        // current angle
        // ----------------------------------------------------------------------------------
        if (m_targetPositionDeg == STOW_ENC_POS && currentPositionDeg > STOW_CUTOFF) {
            io.setIntakePivotPercentOutput(0.0);

        } else {
        //set final mtr pwr
        io.setIntakePivotEncOutput(m_finalEncOutput, m_ffPower);
        }
        
        m_prevCurrentPosition = currentPositionDeg;
        m_prevTargetPwr = m_targetPower;

      } else { //we are current setting pwr through manual
        io.setIntakePivotPercentOutput(m_pivotManualPwr);
      }
    } 
    Logger.recordOutput("intake/manual pwr", m_pivotManualPwr);
    Logger.recordOutput("intake/final pwr", m_targetPower);
    Logger.recordOutput("intake/position error", positionError);
    Logger.recordOutput("intake/pidPower", m_pidPower);
    Logger.recordOutput("intake/ffPower", m_ffPower);
    Logger.recordOutput("intake/targetAngle", m_targetPositionDeg);
    Logger.recordOutput("intake/currentAngle", currentPositionDeg);
    Logger.recordOutput("intake/roller target",m_rollerRunningMode);
    Logger.recordOutput("intake/intake angle", calcWristAngle());

  }

  //-------------------------------------Pivot methods--------------------------------
  //auto update intake angle
  public void updateIntakeTargetPosition(double intakeTargetAngle) {
    this.m_targetPositionDeg = intakeTargetAngle;

    currentIntakeState = IntakeState.AUTO;
    System.out.println("in auto");
  }

  public Command cmdSemiManual(double semiManualPwr) {
    return run(()->pivotSemiManual(semiManualPwr));
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
    System.out.println("in semi manual");

  }

  public Command cmdFullManual(double fullManualPwr) {
    return run(()-> pivotFullManual(fullManualPwr));
  }


  //full manual
  public void pivotFullManual(double fullManualPwr) {
    m_pivotManualPwr = 0.4*fullManualPwr;
    currentIntakeState = IntakeState.FULL_MANUAL;
    System.out.println("in pivot manual");

  }

  private double calculateGravityFF() {
    double pivotAngleRadians = Math.toRadians(calcWristAngle());//+CENTER_OF_MASS_OFFSET_DEG);
    double appliedCosineValue = Math.cos(pivotAngleRadians);
    
    //is the intake is dipped below horizontal in which case we need more pwr...
    //take the difference of the cosine value and 1 and add it to the applied cosine value
    if(pivotAngleRadians < 0) {
      double addedDiffernce = 1 - appliedCosineValue;
      appliedCosineValue = 1 + addedDiffernce;
    }

    return GRAVITY_FF_SCALING_COEFFICIENT * appliedCosineValue;
  }

  private double calcWristAngle() {
    double wristAngle = (((inputs.pivotMtrEncRev *360)/6));
    return wristAngle;
  }

  //-------------------------------------Roller methods--------------------------------
  public Command cmdRollerIn() {
    return runOnce(()-> m_rollerRunningMode = 1);
  }

  public Command cmdRollerOut() {
    return runOnce(()-> m_rollerRunningMode = 2);
  }

  public Command cmdRollerOff() {
    return runOnce(()->  m_rollerRunningMode = 0);
  }

}
