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

  private final double TURRET_POWER     = 0.6;
  private final double TURRET_DECEL_PWR = 0.3;
 

  private static final double TURRET_kP = 0.01;
  private static final double TURRET_kI = 0.0;
  private static final double TURRET_kD = 0.0;

  private static final double TURRET_GEARBOX_PINION      = 9.0/1.0;
  private static final double TURRET_GEARBOX_TURRET_GEAR = 140.0/10.0;
 
  private static final double GEAR_REDUCTION  =  TURRET_GEARBOX_PINION *TURRET_GEARBOX_TURRET_GEAR;
  private static final double TURRET_REV_PER_DEG = GEAR_REDUCTION/360;

  public static double currentTurretDegree = 0.0;


  //variables
  private double m_turretTargetDegree;
  private double pidTurretPower;

  private PIDController pid;

  private final double TURRET_POSITIVE_MAX_RANGE = 120.0;
  private final double TURRET_NEGATIVE_MAX_RANGE = -120.0;

  private final double NEGATIVE_DECEL_THRESHOLD  =  -15.0;
  private final double POS_DECEL_THRESHOLD       =   15.0;

  private double HOME_POSITION             = 0.0;
  private double manualTurretPwr;

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

  //  io.turretSetEncoderPos(HOME_POSITION);
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

    currentTurretDegree = inputs.turretEncValue / TURRET_REV_PER_DEG; //TBD make conversion
    System.out.println(currentTurretDegree);
    System.out.println(currentTurretState);
    
    pidTurretPower = pid.calculate(currentTurretDegree, m_turretTargetDegree);
    if (currentTurretState == TurretState.AUTO) {
      io.turretSetPwr(pidTurretPower);
    }
    else {
      io.turretSetPwr(manualTurretPwr);
    }

  }

  //------------------------------------Turret Methods----------------
  public Command cmdTurretDegree(double turretDeg) {
    return run(()->setTurretTargetDegree(turretDeg));
  }

  public void setTurretTargetDegree(double turretTargetDegree) {
    currentTurretState = TurretState.AUTO;
    m_turretTargetDegree = turretTargetDegree;
  }

  public void rotateLeft(){
    currentTurretState = TurretState.FULL_MANUAL;

    if (currentTurretDegree > (TURRET_NEGATIVE_MAX_RANGE - NEGATIVE_DECEL_THRESHOLD)) {
      System.out.println("1");
      manualTurretPwr = -TURRET_POWER;
    }
    else if ((currentTurretDegree < (TURRET_NEGATIVE_MAX_RANGE - NEGATIVE_DECEL_THRESHOLD)) && (currentTurretDegree >= TURRET_NEGATIVE_MAX_RANGE)&& (manualTurretPwr < 0)){
      System.out.println("2");
      io.turretSetPwr(pid.calculate(currentTurretDegree, TURRET_NEGATIVE_MAX_RANGE ));
    }
    else if ((currentTurretDegree < TURRET_NEGATIVE_MAX_RANGE))
    {
      System.out.println("3");
       io.turretSetPwr(pid.calculate(currentTurretDegree, TURRET_NEGATIVE_MAX_RANGE ));
    }  
    else {
      System.out.println("4");
      manualTurretPwr = 0.0;
    }      


 
  }
  

  public void rotateRight(){
    currentTurretState = TurretState.FULL_MANUAL;

    if (currentTurretDegree < (TURRET_POSITIVE_MAX_RANGE - POS_DECEL_THRESHOLD)){
      manualTurretPwr = TURRET_POWER;
    }
    else if ((currentTurretDegree > (TURRET_POSITIVE_MAX_RANGE - POS_DECEL_THRESHOLD)) && (currentTurretDegree < TURRET_POSITIVE_MAX_RANGE) && (manualTurretPwr > 0)){
          io.turretSetPwr(pid.calculate(currentTurretDegree, TURRET_POSITIVE_MAX_RANGE));
    }
    else if ((currentTurretDegree > TURRET_POSITIVE_MAX_RANGE))
    {
       io.turretSetPwr(pid.calculate(currentTurretDegree, TURRET_POSITIVE_MAX_RANGE));
    }  
    else {
      manualTurretPwr = 0.0;
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

  public Command cmdResetTurretPosition(){
    return run(()-> io.turretSetEncoderPos(HOME_POSITION));
  }

}
