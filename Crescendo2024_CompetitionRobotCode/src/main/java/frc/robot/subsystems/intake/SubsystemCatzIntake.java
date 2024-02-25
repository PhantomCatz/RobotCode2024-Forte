// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.Utils.LoggedTunableNumber;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

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
  private final double ROLLERS_MTR_PWR_IN  =  0.6;
  private final double ROLLERS_MTR_PWR_OUT = -0.4;

  private static final int ROLLERS_STATE_OFF = 0;
  private static final int ROLLERS_STATE_IN  = 1;
  private static final int ROLLERS_STATE_OUT = 2;

  //intake roller variables

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

  public final double PIVOT_FF_kS = 0.2;
  public final double PIVOT_FF_kG = 0.437;
  public final double PIVOT_FF_kV = 0.03;
  public final double PIVOT_FF_kA = 0.0;


  private PIDController pivotPID;
  private ArmFeedforward pivotFeedFoward;

  private static final double PIVOT_PID_kP = 0.034;//0.034;
  private static final double PIVOT_PID_kI = 0.000; 
  private static final double PIVOT_PID_kD = 0.000; 

  private final double PID_FINE_GROSS_THRESHOLD_DEG = 20;
  private final double ERROR_INTAKE_THRESHOLD_DEG = 5.0;

  private final double STOW_CUTOFF = 150; //TBD need to dial in
  private final double GROUND_CUTTOFF = 200;

  private final double MANUAL_HOLD_STEP_COEFFICIENT = 2.0;

  private final double STOW_ENC_POS = 0.0;
  private final double ANGLE_AMP_SCORING = 0.0;
  private final double ANGLE_GROUND_INTAKE = 0.0; //TBD need to dial in on wednesday
  private final double NULL_INTAKE_POSITION = -999.0;

  //intake variables
  private double m_pivotManualPwr;
  private double m_targetPower;
  private double m_pidVolts;
  private double m_ffVolts;
  private double m_prevTargetPwr;
  private double m_prevCurrentPosition;
  private double m_targetPositionDeg;
  private double m_numConsectSamples;
  private boolean m_intakeInPosition;
  private int    m_rollerRunningMode;
  private double m_previousCurrentAngle;
  private double m_finalEncOutput;
  private double currentPositionDeg;
  private double positionError;
  private double pivotVelRadPerSec;

  LoggedTunableNumber kgtunning = new LoggedTunableNumber("kgtunningVolts",0.0);
  LoggedTunableNumber kftunning = new LoggedTunableNumber("kFtunningVolts",0.0);



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

      //collect ff variables and pid variables
    currentPositionDeg = calcWristAngleDeg();
    positionError = currentPositionDeg - m_targetPositionDeg;
    pivotVelRadPerSec = Math.toRadians(currentPositionDeg - m_previousCurrentAngle)/0.02;
    
    if(DriverStation.isDisabled()) {
      io.setRollerPercentOutput(0.0);
      m_rollerRunningMode = 0;
      io.setIntakePivotPercentOutput(0.0);
      m_targetPositionDeg = NULL_INTAKE_POSITION;
    } else { 
      //robot enabled

        if(m_rollerRunningMode == ROLLERS_STATE_OUT) {
            io.setRollerPercentOutput(ROLLERS_MTR_PWR_OUT); 
        } else if(m_rollerRunningMode == ROLLERS_STATE_IN) {
              if(inputs.IntakeBeamBrkBroken) {
                io.setRollerPercentOutput(0.0);
                m_rollerRunningMode = ROLLERS_STATE_OFF;
              } else {
                io.setRollerPercentOutput(ROLLERS_MTR_PWR_IN);
              }
        } else {
          io.setRollerPercentOutput(0.0);
        }

      // ----------------------------------------------------------------------------------
      // IntakePivot
      // ----------------------------------------------------------------------------------
      if ((currentIntakeState == IntakeState.AUTO || 
           currentIntakeState == IntakeState.SEMI_MANUAL) && 
           m_targetPositionDeg != NULL_INTAKE_POSITION && 
           SubsystemCatzElevator.getInstance().getElevatorRevPos() > -3) { 

        //check if at final position using counter
        if ((Math.abs(positionError) <= ERROR_INTAKE_THRESHOLD_DEG)) {
          m_numConsectSamples++;
          if (m_numConsectSamples >= 1) {
              m_intakeInPosition = true;
          }
        } else {
          m_numConsectSamples = 0; //resetcounter if intake hasn't leveled off
        }
        
        
        m_ffVolts = calculatePivotFeedFoward(Math.toRadians(currentPositionDeg), pivotVelRadPerSec, 0);
        m_pidVolts = -pivotPID.calculate(m_targetPositionDeg, currentPositionDeg);
        double finalVolts = m_pidVolts + m_ffVolts;

        // // ----------------------------------------------------------------------------------
        // // If we are going to Stow Position & have passed the power cutoff angle, set
        // // power to 0, otherwise calculate new motor power based on position error and
        // // current angle
        // // ----------------------------------------------------------------------------------
        // if (m_targetPositionDeg == STOW_ENC_POS && currentPositionDeg > STOW_CUTOFF) {
        //   io.setIntakePivotPercentOutput(0.0);
        // } else {
        // //set final mtr pwr
        // }
        io.setIntakePivotVoltage(finalVolts);
        
        m_prevCurrentPosition = currentPositionDeg;
        m_prevTargetPwr = m_targetPower;

      } else { //we are current setting pwr through manual
        io.setIntakePivotVoltage(kgtunning.get());
      }
    } 
    m_previousCurrentAngle = currentPositionDeg;
    Logger.recordOutput("intake/ff volts", m_ffVolts);
    Logger.recordOutput("intake/pid volts", m_pidVolts);
    Logger.recordOutput("intake/pivotvel", pivotVelRadPerSec);
    Logger.recordOutput("intake/final pwr", m_targetPower);
    Logger.recordOutput("intake/position error", positionError);
    Logger.recordOutput("intake/pidPower", m_pidVolts);
    Logger.recordOutput("intake/targetAngle", m_targetPositionDeg);
    Logger.recordOutput("intake/currentAngle", currentPositionDeg);
    Logger.recordOutput("intake/roller target",m_rollerRunningMode);
    Logger.recordOutput("intake/intake angle", calcWristAngleDeg());

  }

  //-------------------------------------Pivot methods--------------------------------
  //auto update intake angle
  public void updateIntakeTargetPosition(double intakeTargetAngle) {
    this.m_targetPositionDeg = intakeTargetAngle;

    currentIntakeState = IntakeState.AUTO;
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

  //full manual
  public void pivotFullManual(double fullManualPwr) {
    m_pivotManualPwr = 0.4*fullManualPwr;
    currentIntakeState = IntakeState.FULL_MANUAL;
    System.out.println("in pivot manual");

  }

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

  public double getWristAngle() {
    return currentPositionDeg;
  }

  //-------------------------------------Roller methods--------------------------------
  public Command cmdRollerIn() {
    return runOnce(()-> setRollerState(1));
  }

  public Command cmdRollerOut() {
    return runOnce(()-> setRollerState(2));
  }

  public Command cmdRollerOff() {
    return runOnce(()->  setRollerState(0));
  }

  public void setRollerState(int rollerRunningMode) {
    m_rollerRunningMode = rollerRunningMode;
  }

}
