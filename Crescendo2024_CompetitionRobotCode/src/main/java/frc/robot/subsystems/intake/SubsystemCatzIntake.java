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
  private final double MAX_GRAVITY_FF = 0.04;

  private final double MANUAL_HOLD_STEP_SIZE = 2.0;

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

        // //change pid values between aggressive and gentle depending on how close to target angle pivot is
        // if (Math.abs(positionError) >= PID_FINE_GROSS_THRESHOLD_DEG) {
        //     pid.setP(GROSS_kP);
        //     pid.setI(GROSS_kI);
        //     pid.setD(GROSS_kD);

        // } else if (Math.abs(positionError) < PID_FINE_GROSS_THRESHOLD_DEG) {
        //     pid.setP(FINE_kP);
        //     pid.setI(FINE_kI);
        //     pid.setD(FINE_kD);
        // }

        //calculate pwr based off given pid values
        m_pidPower = pid.calculate(currentPositionDeg, m_targetPositionDeg);
        m_ffPower = calculateGravityFF();
        m_targetPower = m_pidPower + m_ffPower;

        // // -------------------------------------------------------------
        // // checking if we did not get updated position value(Sampling Issue).
        // // If no change in position, this give invalid target power(kD issue).
        // // Therefore, go with prev targetPower Value.
        // // -------------------------------------------------------------------
        // if (m_prevCurrentPosition == currentPositionDeg) {
        //     m_targetPower = prevTargetPwr;
        // }

        // ----------------------------------------------------------------------------------
        // If we are going to Stow Position & have passed the power cutoff angle, set
        // power to 0, otherwise calculate new motor power based on position error and
        // current angle
        // ----------------------------------------------------------------------------------
        if (m_targetPositionDeg == STOW_ENC_POS && currentPositionDeg > STOW_CUTOFF) {
            m_targetPower = 0.0;

        }

        //set pivot pwr
        io.setIntakePivotPercentOutput(-m_targetPower);

        m_prevCurrentPosition = currentPositionDeg;
        m_prevTargetPwr = m_targetPower;
      } else {
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

    // //if logicals for determining pid gains
    // //logicals for amp
    // if(intakeTargetAngle == -140){
    //     if(previousTargetAngle == -190) {
    //       pid.setP(FINE_kP);
    //       pid.setI(FINE_kI);
    //       pid.setD(FINE_kD);
    //     } else {
    //       pid.setP(GROSS_kP);
    //       pid.setI(GROSS_kI);
    //       pid.setD(GROSS_kD);
    //     }
    // //logcials for ground
    // }else if(intakeTargetAngle == -190) {
    //     if(previousTargetAngle == -140) {
    //       pid.setP(FINE_kP);
    //       pid.setI(FINE_kI);
    //       pid.setD(FINE_kD);
    //     } else {
    //       pid.setP(GROSS_kP);
    //       pid.setI(GROSS_kI);
    //       pid.setD(GROSS_kD);
    //     }
    // //logicals for stow
    // } else if(intakeTargetAngle == 0) {
    //   if(previousTargetAngle == -20) {
    //       pid.setP(HANDOFF_TO_STOW_KP);
    //       pid.setI(FINE_kI);
    //       pid.setD(FINE_kD);
    //   } else if(previousTargetAngle == -140 ||
    //              previousTargetAngle == -190) {
    //       pid.setP(GROSS_kP);
    //       pid.setI(GROSS_kI);
    //       pid.setD(GROSS_kD);    
    //   }
    // //logicals for the handoff position
    // } else if(intakeTargetAngle == -20) {
    //       pid.setP(FINE_kP);
    //       pid.setI(FINE_kI);
    //       pid.setD(FINE_kD);
    // }

    //m_previousTargetAngle = m_targetPositionDeg;
    io.setIntakePivotEncOutput((intakeTargetAngle*6)/360);
    currentIntakeState = IntakeState.AUTO;
    System.out.println("in auto");
  }

  public Command cmdSemiManual(double semiManualPwr) {
    return run(()->pivotSemiManual(semiManualPwr));
  }

  //semi manual
  public void pivotSemiManual(double semiManualPwr) {
    if (semiManualPwr > 0) {
      m_targetPositionDeg = Math.min((m_targetPositionDeg + semiManualPwr * MANUAL_HOLD_STEP_SIZE),
              -30);
    } else {
      m_targetPositionDeg = Math.max((m_targetPositionDeg + semiManualPwr * MANUAL_HOLD_STEP_SIZE),
              -180);
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
    double radians = Math.toRadians(calcWristAngle()+CENTER_OF_MASS_OFFSET_DEG);
    double cosineScalar = Math.cos(radians);

    return MAX_GRAVITY_FF * cosineScalar;
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
