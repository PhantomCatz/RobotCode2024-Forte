// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzMechanismPosition;


public class SubsystemCatzTurret extends SubsystemBase {
  //intake io block
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  //intake instance
  private static SubsystemCatzTurret instance = new SubsystemCatzTurret();

  //turret variables
  //constants
  //private final double ENC_TO_INTAKE_GEAR_RATIO = (46.0 / 18.0)* (32.0 / 10.0);
  //private final double WRIST_CNTS_PER_DEGREE = (2096.0 * ENC_TO_INTAKE_GEAR_RATIO) / 360.0;

  private final double TURRET_POWER     = 0.6;
 

  private static final double TURRET_kP = 0.0;
  private static final double TURRET_kI = 0.0;
  private static final double TURRET_kD = 0.0;

  private static final double TURRET_GEARBOX_PINION      = 9.0/1.0;
  private static final double TURRET_GEARBOX_TURRET_GEAR = 140.0/10.0;
 
  private static final double GEAR_REDUCTION  =  TURRET_GEARBOX_PINION *TURRET_GEARBOX_TURRET_GEAR;
  private static final double TURRET_REV_PER_DEG = GEAR_REDUCTION/360;

  public static double turretEncoderPosition = 0.0;


  //variables
  private double m_turretTargetDegree;
  private double pidTurretPower;

  private boolean left = false;
  private boolean right = false;

  private PIDController pid;

  private double TURRET_POSITIVE_MAX_RANGE = 120.0 -10;
  private double TURRET_NEGATIVE_MAX_RANGE = -120.0 +10.0;
  private double HOME_POSITION             = 0.0;

  public SubsystemCatzTurret() {

    switch (CatzConstants.currentMode) {
      case REAL: io = new TurretIOReal();
      break;

      case SIM : io = null;
      break;

      case REPLAY: io = new TurretIOReal() {};
      break;

      default: io = null;
      break;
    }

    pid = new PIDController(TURRET_kP, 
                            TURRET_kI, 
                            TURRET_kD);

  }

  private static TurretState currentTurretState;
  public static enum TurretState {
    AUTO,
    FULL_MANUAL
  }

  // Get the singleton instance of the Turret Subsystem
  public static SubsystemCatzTurret getInstance() {
      return instance;
  }


  @Override
  public void periodic() {
    //io.turretSetEncoderPos(HOME_POSITION);
    io.updateInputs(inputs);
    Logger.processInputs("intake/inputs", inputs);   
    Logger.recordOutput("Turret Encoder", inputs.turretEncValue);

     turretEncoderPosition = inputs.turretEncValue / TURRET_REV_PER_DEG; //TBD make conversion
    System.out.println(turretEncoderPosition);


  }

  //------------------------------------Turret Methods----------------
  public Command cmdTurretDegree(double turretDeg) {
    return run(()->setTurretTargetDegree(turretDeg));
  }

  public void setTurretTargetDegree(double turretTargetDegree) {
    m_turretTargetDegree = turretTargetDegree;

  }

  public void rotateLeft(){


        if (turretEncoderPosition > TURRET_NEGATIVE_MAX_RANGE ){
              io.turretSetPwr(-TURRET_POWER);
            }
            else {
              io.turretSetPwr(0.0);   
            }        
      }
  

  public void rotateRight(){

        if (turretEncoderPosition < TURRET_POSITIVE_MAX_RANGE ){
               io.turretSetPwr(TURRET_POWER);
            }

        else {
          io.turretSetPwr(0.0);
        }
          
    }
  

  //-------------------------------------Manual methods--------------------------------
  public Command cmdTurretLT() {
    return run(()-> rotateLeft());
  }

  public Command cmdTurretRT() {
    return run(()-> rotateRight());
  }

  public Command cmdTurretOff() {
    currentTurretState = TurretState.FULL_MANUAL;
    return run(()-> io.turretSetPwr(0.0));
  }

  public Command resetTurretPosition(){
    return run(()-> io.turretSetEncoderPos(HOME_POSITION));
  }

}
