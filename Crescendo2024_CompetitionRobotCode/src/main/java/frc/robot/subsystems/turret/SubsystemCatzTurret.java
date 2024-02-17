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
import frc.robot.subsystems.vision.SubsystemCatzVision;


public class SubsystemCatzTurret extends SubsystemBase {
  //intake io block
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  //intake instance
  private static SubsystemCatzTurret instance = new SubsystemCatzTurret();

  //turret variables

  private final double TURRET_POWER     = 0.6;
  private final double TURRET_DECEL_PWR = 0.3;
 

  private static final double TURRET_kP = 0.02;
  private static final double TURRET_kI = 0.0;
  private static final double TURRET_kD = 0.0;

  private static final double TURRET_GEARBOX_PINION      = 9.0/1.0;
  private static final double TURRET_GEARBOX_TURRET_GEAR = 140.0/10.0;
 
  private static final double GEAR_REDUCTION  =  TURRET_GEARBOX_PINION *TURRET_GEARBOX_TURRET_GEAR;
  private static final double TURRET_REV_PER_DEG = GEAR_REDUCTION/360;

  public static double currentTurretDegree = 0.0; //0.0


  //variables
  private double m_turretTargetDegree;
  private double pidTurretPower;

  private PIDController pid;
  private PIDController limelightPID;

  private final double TURRET_POSITIVE_MAX_RANGE = 120.0; //120
  private final double TURRET_NEGATIVE_MAX_RANGE = -120.0; //-120



  private final double NEGATIVE_DECEL_THRESHOLD  =  -15.0;
  private final double POS_DECEL_THRESHOLD       =   15.0;

  private double HOME_POSITION             = 0.0; //0.0
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
    limelightPID = new PIDController(0.013,0.0,0.0001);

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
    Logger.processInputs("turret/inputs", inputs);   
    Logger.recordOutput("Turret Encoder", inputs.turretEncValue);
    Logger.recordOutput("curretnTurretState", currentTurretState);
    Logger.recordOutput("currentTurretDeg", currentTurretDegree);
    Logger.recordOutput("Limelight tX", SubsystemCatzVision.getInstance().getHorizontalAngle());
    Logger.recordOutput("m_TurretTargetDegree", m_turretTargetDegree);
    currentTurretDegree = inputs.turretEncValue / TURRET_REV_PER_DEG; //TBD make conversion

    double offsetX = SubsystemCatzVision.getInstance().getOffsetX();
    
    pidTurretPower = -limelightPID.calculate(offsetX, 0);
    if (currentTurretState == TurretState.AUTO) {
      io.turretSetPwr(pidTurretPower);
    }
    else {
      io.turretSetPwr(manualTurretPwr);
    }

    Logger.recordOutput("turret/offsetXTurret", offsetX);
    Logger.recordOutput("turret/PwrPID", pidTurretPower);

  }

  //------------------------------------Turret Methods----------------
  
  public void setTurretTargetDegree(double turretTargetDegree) {
    currentTurretState = TurretState.AUTO;
    m_turretTargetDegree = turretTargetDegree;
  }
  
  // *********** code for turning turret towards static target (Apriltag tX is static + an offset) ************* //
  public void autoRotate() {
    currentTurretState = TurretState.AUTO;
  }
  
  public void rotateLeft(){
    currentTurretState = TurretState.FULL_MANUAL;
    
    if (currentTurretDegree > (TURRET_NEGATIVE_MAX_RANGE - NEGATIVE_DECEL_THRESHOLD)) {
      manualTurretPwr = -TURRET_POWER;
    }
    else if ((currentTurretDegree < (TURRET_NEGATIVE_MAX_RANGE - NEGATIVE_DECEL_THRESHOLD)) && (currentTurretDegree >= TURRET_NEGATIVE_MAX_RANGE) && (manualTurretPwr < 0)){
      manualTurretPwr = -TURRET_DECEL_PWR;
    }
    else if ((currentTurretDegree < TURRET_NEGATIVE_MAX_RANGE))
    {
       io.turretSetPwr(pid.calculate(currentTurretDegree, TURRET_NEGATIVE_MAX_RANGE ));
    }  
    else {
      manualTurretPwr = 0.0;
    }      
    
    //    currentTurretDegree = SubsystemCatzVision.getInstance().getHorizontalAngle();
  }
  
  
  public void rotateRight(){
    currentTurretState = TurretState.FULL_MANUAL;
    
    if (currentTurretDegree < (TURRET_POSITIVE_MAX_RANGE - POS_DECEL_THRESHOLD)){
      manualTurretPwr = TURRET_POWER;
    }
    else if ((currentTurretDegree > (TURRET_POSITIVE_MAX_RANGE - POS_DECEL_THRESHOLD)) && (currentTurretDegree < TURRET_POSITIVE_MAX_RANGE) && (manualTurretPwr > 0)){
      manualTurretPwr = TURRET_DECEL_PWR;
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
  
  public Command cmdTurretDegree(double turretDeg) {
    return run(()->setTurretTargetDegree(turretDeg));
  }

  public Command cmdAutoRotate() {
    return run(() ->autoRotate());
  }
  
}
