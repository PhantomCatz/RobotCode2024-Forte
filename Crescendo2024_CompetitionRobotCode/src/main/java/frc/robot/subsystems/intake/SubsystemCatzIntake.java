// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final double ROLLERS_MTR_PWR_OUT = -0.6;

  //intake roller variables


  //intake pivot variables
  //constants
  private final double ENC_TO_INTAKE_GEAR_RATIO = (46.0 / 18.0)* (32.0 / 10.0);
  private final double WRIST_CNTS_PER_DEGREE = (2096.0 * ENC_TO_INTAKE_GEAR_RATIO) / 360.0;

  private static final double GROSS_kP = 0.0;
  private static final double GROSS_kI = 0.0;
  private static final double GROSS_kD = 0.0;

  private static final double FINE_kP = 0.0;
  private static final double FINE_kI = 0.0;
  private static final double FINE_kD = 0.0;

  private final double PID_FINE_GROSS_THRESHOLD_DEG = 17.0;
  private final double ERROR_INTAKE_THRESHOLD_DEG = 5.0;

  private final double STOW_CUTOFF = 0.0; //TBD need to dial in
  private final double CENTER_OF_MASS_OFFSET_DEG = 177.0;
  private final double MAX_GRAVITY_FF = 0.07;

  private final double MANUAL_HOLD_STEP_SIZE = 2.0;

  private final double STOW_ENC_POS = 0.0;
  private final double ANGLE_AMP_SCORING = 0.0;
  private final double ANGLE_GROUND_INTAKE = 0.0; //TBD need to dial in on wednesday

  //intake variables
  private double m_pivotManualPwr;
  private double m_targetPower;
  private double pidPower;
  private double ffPower;
  private double prevTargetPwr;
  private double m_prevCurrentPosition;
  private double m_targetPositionDeg;
  private double numConsectSamples;
  private boolean intakeInPosition;


  private PIDController pid;


  public SubsystemCatzIntake() {

    switch (CatzConstants.currentMode) {
      case REAL: io = new IntakeIOReal();
      break;

      case SIM : io = null;
      break;

      case REPLAY: io = new IntakeIOReal() {};
      break;

      default: io = null;
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
    double currentPositionDeg = inputs.pivotMtrEncPos / WRIST_CNTS_PER_DEGREE;

    if(DriverStation.isDisabled()) {
      io.setIntakePivotPercentOutput(0.0);
    } else { 
      //robot enabled

      //if we are outtaking by looking at the hardware mtr pwrs with the flag enabled...ignored logic TBD
      if(inputs.rollerPercentOutput < 0.0) {

      } else {
        //Intake Roller Logic
        if(!inputs.BeamBrkFrontBroken) {
          cmdRollerOff();
        }
      }

      //Intake Pivot Logic
      if (currentIntakeState == IntakeState.AUTO || 
          currentIntakeState == IntakeState.SEMI_MANUAL) { 

            //check if at final position using counter
        double positionError = currentPositionDeg - m_targetPositionDeg;
        if ((Math.abs(positionError) <= ERROR_INTAKE_THRESHOLD_DEG)) {
            numConsectSamples++;
            if (numConsectSamples >= 1) {
                intakeInPosition = true;
            }
        } else {
            numConsectSamples = 0; //resetcounter if intake hasn't leveled off
        }

        //change pid values between aggressive and gentle depending on how close to target angle pivot is
        if (Math.abs(positionError) >= PID_FINE_GROSS_THRESHOLD_DEG) {
            pid.setP(GROSS_kP);
            pid.setI(GROSS_kI);
            pid.setD(GROSS_kD);

        } else if (Math.abs(positionError) < PID_FINE_GROSS_THRESHOLD_DEG) {
            pid.setP(FINE_kP);
            pid.setI(FINE_kI);
            pid.setD(FINE_kD);
        }

        //calculate pwr based off given pid values
        pidPower = pid.calculate(currentPositionDeg, m_targetPositionDeg);
        ffPower = calculateGravityFF();
        m_targetPower = pidPower + ffPower;

        // -------------------------------------------------------------
        // checking if we did not get updated position value(Sampling Issue).
        // If no change in position, this give invalid target power(kD issue).
        // Therefore, go with prev targetPower Value.
        // -------------------------------------------------------------------
        if (m_prevCurrentPosition == currentPositionDeg) {
            m_targetPower = prevTargetPwr;
        }

        // ----------------------------------------------------------------------------------
        // If we are going to Stow Position & have passed the power cutoff angle, set
        // power to 0, otherwise calculate new motor power based on position error and
        // current angle
        // ----------------------------------------------------------------------------------
        if (m_targetPositionDeg == STOW_ENC_POS && currentPositionDeg > STOW_CUTOFF) {
            m_targetPower = 0.0;
        }

        //set pivot pwr
        io.setIntakePivotPercentOutput(m_targetPower);

        m_prevCurrentPosition = currentPositionDeg;
        prevTargetPwr = m_targetPower;
      } else {
        io.setIntakePivotPercentOutput(m_pivotManualPwr);
      }
    } 

    Logger.recordOutput("intake/final pwr", m_targetPower);
    Logger.recordOutput("intake/pidPower", pidPower);
    Logger.recordOutput("intake/ffPower", ffPower);
    Logger.recordOutput("intake/targetAngle", m_targetPositionDeg);
    Logger.recordOutput("intake/currentAngle", currentPositionDeg);

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
    return run(()->pivotFullManual(fullManualPwr));
  }

  //full manual
  public void pivotFullManual(double fullManualPwr) {
    m_pivotManualPwr = fullManualPwr;
    currentIntakeState = IntakeState.FULL_MANUAL;
    System.out.println("in pivot manual");

  }

  private double calculateGravityFF() {
    double radians = Math.toRadians(calcWristAngle() - CENTER_OF_MASS_OFFSET_DEG);
    double cosineScalar = Math.cos(radians);

    return MAX_GRAVITY_FF * cosineScalar;
  }

  private double calcWristAngle() {
    double wristAngle = ((inputs.pivotMtrEncPos / WRIST_CNTS_PER_DEGREE));
    return wristAngle;
  }

  //-------------------------------------Roller methods--------------------------------
  public Command cmdRollerIn() {
    return run(()-> io.setRollerPercentOutputIO(ROLLERS_MTR_PWR_IN));
  }

  public Command cmdRollerOut() {
    return run(()->  io.setRollerPercentOutputIO(ROLLERS_MTR_PWR_OUT));
  }

  public Command cmdRollerOff() {
    return run(()->  io.setRollerPercentOutputIO(0.0));
  }

}
