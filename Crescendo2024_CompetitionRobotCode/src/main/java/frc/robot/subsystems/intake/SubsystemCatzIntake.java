// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;

public class SubsystemCatzIntake extends SubsystemBase {
  
  private final IntakeIO io;
  private static SubsystemCatzIntake instance;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

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
    Logger.processInputs("climb/inputs", inputs);

    if(!DriverStation.isEnabled()) {
    
    } 
    else if(true) {

    }
    
    // This method will be called once per scheduler run
    io.exampleAccessMethod(0);
  }

  // Get the singleton instance of the ClimbSubsystem
  public static SubsystemCatzIntake getInstance() {
      return instance;
  }

}
