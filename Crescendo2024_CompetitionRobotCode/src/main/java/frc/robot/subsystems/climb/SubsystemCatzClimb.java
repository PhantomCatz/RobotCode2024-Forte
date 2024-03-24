// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;


public class SubsystemCatzClimb extends SubsystemBase {
  //instance instantiation
  private static SubsystemCatzClimb instance = new SubsystemCatzClimb();

  //io block
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  //CLIMB Constants
  private static double CLIMB_EXTEND_SET_POS = 10.0;
  private static double CLIMB_RETRACT_SET_POS = 0.0;


  //climb variables
  private double climbPercentOutputLT = 0.0;
  private double climbPercentOutputRT = 0.0;

  private boolean isClimbingEnabled = false;



  private SubsystemCatzClimb() {
        switch (CatzConstants.currentMode) {
      case REAL: io = new ClimbIOReal();
                 System.out.println("Climb Configured for Real");
      break;

      case REPLAY: io = new ClimbIOReal() {};
                   System.out.println("Climb Configured for Replayed simulation");
      break;

      case SIM:
      default: io = null;
               System.out.println("Climb Unconfigured");
      break;
    }

    
  }

  public static SubsystemCatzClimb getInstance() {
    return instance;
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("climb/inputs", inputs);

    if(DriverStation.isDisabled()) {
      io.setClimbMtrPercentOutputLT(0.0);
      io.setClimbMtrPercentOutputRT(0.0);
      climbPercentOutputLT = 0.0;
      climbPercentOutputRT = 0.0;

    } else {
      
        io.setClimbMtrPercentOutputLT(climbPercentOutputLT);
        io.setClimbMtrPercentOutputRT(climbPercentOutputRT);

    }
  }

  public void setClimbModeEnabled(boolean set) {
    isClimbingEnabled = set;
  }

  public boolean isClimbing(){
    return isClimbingEnabled;
  }

  public void setLeftClimbPercentOutput(double output) {
    climbPercentOutputLT = output;
  }

  public void setRightClimbPercentOutput(double output) {
    climbPercentOutputRT = output;
  }

  public Command setClimbOff(){
    return run(()-> setClimbMtrsZero());
  }
  public void setClimbMtrsZero(){
    climbPercentOutputLT = 0.0;
    climbPercentOutputRT = 0.0;
  }
}
