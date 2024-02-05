// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class SubsystemCatzIntake extends SubsystemBase {

  private CatzMechanismPosition m_targetPosition;
  private final IntakeIO io;
  private static SubsystemCatzIntake instance = new SubsystemCatzIntake();
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private double m_stickAngle;
  private CatzMechanismPosition m_newPosition;
  private boolean rollerEnable;
  private boolean isBBAllBroken = false;
  public SubsystemCatzIntake() {

            switch (CatzConstants.currentMode) {
            case REAL: io = 
                    new IntakeIOReal();
                break;
            case SIM : io = null;
                break;
            default : io = 
                    new IntakeIOReal() {};
                break;
        }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("intake/inputs", inputs);
    io.setIntakePosition(7);

    /*
     * Intake Roller Stuff
     */
      if(inputs.BBFrontConnected == false) {
        io.rollerDisable();
      } else {
      }
    
  

    /*
     * Intake Pivot Stuff
     */
    if (m_targetPosition != null) {
      io.setIntakePosition(m_targetPosition.getIntakePivotTargetEncPos());
    } else {
      io.setIntakePivotPercentOutput(m_stickAngle);
    }
  }

  public void updateIntakeTargetPosition(CatzMechanismPosition intakeTargetPosition) {
    this.m_targetPosition = intakeTargetPosition;
  }

  public void pivotFullManual(double fullManualPwr) {
    m_stickAngle = fullManualPwr;
    m_targetPosition = null;
  }

  //-------------------------------------Roller Sequencing--------------------------------
  public Command rollerIntakeCommand() {
    return run(()->setRollers(0.6));
  }

  public Command rollerOutakeCommand() {
    return run(()-> setRollers(-0.6));
  }

  public Command rollersOff() {
    return run(()-> setRollers(0));
  }

  //void method for setting output to roller motors and checks if note has already been intaked
  public void setRollers(double output) {
    if(inputs.BBFrontConnected == false) {
      io.setRollerPercentOutput(0.0);
    } else {
      io.setRollerPercentOutput(output);
    }
  }

  // 
  public Command intakePivotOverrideCommand(double stickPwr) {
    m_stickAngle = stickPwr;
    System.out.println("e");
    return run(()->intakePivotOverrideSet(stickPwr));
  }
  public void intakePivotOverrideSet(double stickAngle) {
    m_stickAngle = stickAngle;
  }
  // Get the singleton instance of the ClimbSubsystem
  public static SubsystemCatzIntake getInstance() {
      return instance;
  }

}
