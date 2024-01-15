// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SubsystemCatzElevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;


public class SubsystemCatzElevator extends SubsystemBase {
  
  private final ElevatorIO io;
  private static SubsystemCatzElevator instance;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public SubsystemCatzElevator() {

            switch (CatzConstants.currentMode) {
            case REAL: io = 
                    new ElevatorIOReal();
                break;

            case SIM : io = null;
                break;
            default : io = 
                    new ElevatorIOReal() {};
                break;
        }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/inputs", inputs);


    io.exampleAccessMethod(0);
  }

  // Get the singleton instance of the ClimbSubsystem
  public static SubsystemCatzElevator getInstance() {
      return instance;
  }

}
