// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import java.sql.Driver;

import javax.swing.TransferHandler.TransferSupport;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzAutonomous;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.Utils.CatzMathUtils;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.vision.SubsystemCatzVision;


public class SubsystemCatzTurret extends SubsystemBase {
  //intake io block
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  //intake instance
  private static SubsystemCatzTurret instance = new SubsystemCatzTurret();

  //------------------------------------------------------------------------
  //      turret constants
  //------------------------------------------------------------------------
  private final double TURRET_POWER     = 0.4;
  private final double TURRET_DECEL_PWR = 0.3;
 
  //pid values
  private static final double TURRET_kP = 0.02;
  private static final double TURRET_kI = 0.0;
  private static final double TURRET_kD = 0.0;
  
  private static final double LIMELIGHT_kP = 0.013;
  private static final double LIMELIGHT_kI = 0.0;
  private static final double LIMELIGHT_kD = 0.0001;

  private final double TURRET_POSITIVE_MAX_RANGE =  120.0; 
  private final double TURRET_NEGATIVE_MAX_RANGE = -120.0;

  private final double NEGATIVE_DECEL_THRESHOLD  =  -15.0;
  private final double POS_DECEL_THRESHOLD       =   15.0;

  private static final double TURRET_GEARBOX_PINION      = 9.0/1.0;
  private static final double TURRET_GEARBOX_TURRET_GEAR = 140.0/10.0;
 
  public static final double GEAR_REDUCTION     =  TURRET_GEARBOX_PINION * TURRET_GEARBOX_TURRET_GEAR;
  public static final double TURRET_REV_PER_DEG = GEAR_REDUCTION / 360;
  
  public static final double HOME_POSITION       = 0.0;

  public static double currentTurretDegree = 0.0; //0.0

  //turret variables
  private double m_turretTargetDegree;
  private double apriltagTrackingPower;
  private double m_closedLoopError;
  private double setPositionPower;
  private double offsetAprilTagX;

  private PIDController m_setPositionPID;
  private PIDController m_trackingApriltagPID;
  private double manualTurretPwr;
  private boolean m_trackTarget = false;

  private boolean m_turretIntPos;

  private XboxController driveRumbleController;

  private final SubsystemCatzDrivetrain drivetrain = SubsystemCatzDrivetrain.getInstance();

  private SubsystemCatzTurret() {

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

    driveRumbleController = new XboxController(CatzConstants.OIConstants.XBOX_DRV_PORT);

    m_setPositionPID = new PIDController(TURRET_kP, 
                                         TURRET_kI, 
                                         TURRET_kD);

    m_trackingApriltagPID = new PIDController(LIMELIGHT_kP,
                                              LIMELIGHT_kI,
                                              LIMELIGHT_kD);
  }
  
  // Get the singleton instance of the Turret Subsystem
  public static SubsystemCatzTurret getInstance() {
      return instance;
  }
  
  private static TurretState currentTurretState;
  public static enum TurretState {
    AUTO,
    TRACKING_APRILTAG,
    FULL_MANUAL
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("turret", inputs);

    currentTurretDegree = inputs.turretEncValue;// / TURRET_REV_PER_DEG; 
    
    //obtain calculation values
    apriltagTrackingPower = -m_trackingApriltagPID.calculate(offsetAprilTagX, 0);
    setPositionPower      =  m_setPositionPID.calculate(currentTurretDegree, m_turretTargetDegree);
    //offsetAprilTagX       = SubsystemCatzVision.getInstance().getOffsetX(1);
    m_closedLoopError = ((currentTurretDegree - m_turretTargetDegree)  * TURRET_REV_PER_DEG);


    if(DriverStation.isDisabled()) {
      io.turretSetPwr(0.0);
      manualTurretPwr = 0.0;
      currentTurretState = TurretState.FULL_MANUAL;
    } else { 
      if (currentTurretState == TurretState.AUTO) {

        if(Math.abs(m_turretTargetDegree) > 120) {
          driveRumbleController.setRumble(RumbleType.kBothRumble, 0.3);
          io.turretSetPwr(0.0);
        } else if(Math.abs(m_turretTargetDegree) > 0.0) {
          if(SubsystemCatzIntake.getInstance().getIntakeInPos()) {
            io.turretSetPwr(setPositionPower);
            driveRumbleController.setRumble(RumbleType.kBothRumble, 0.0);
          }

        } else {
          io.turretSetPwr(setPositionPower);
          driveRumbleController.setRumble(RumbleType.kBothRumble, 0.0);
        } 

        if(Math.abs(currentTurretDegree - m_turretTargetDegree) < 3) {
          m_turretIntPos = true;
        } 

      } else if (currentTurretState == TurretState.TRACKING_APRILTAG) {
        //only track the shooterlimelight to the speaker apriltag
        if(SubsystemCatzVision.getInstance().getAprilTagID(1) == 7) {
          io.turretSetPwr(apriltagTrackingPower);
        }
      } else {
        io.turretSetPwr(manualTurretPwr);
      }
    }


    Logger.recordOutput("turret/offsetXTurret", offsetAprilTagX);
    // whc 01Mar24 need to fix.  Do we need to install a limelight? TBD
   //Logger.recordOutput("turret/PwrPID", apriltagTrackingPower);
   // Logger.recordOutput("turret/currentTurretState", currentTurretState);
    Logger.recordOutput("turret/currentTurretDegee", currentTurretDegree);
    Logger.recordOutput("turret/closedlooperror", m_closedLoopError);
    Logger.recordOutput("turret/m_TurretTargetDegree", m_turretTargetDegree);
  }

  public Command testTurretAngles(){
    return run(()->{
      currentTurretState = TurretState.AUTO;
      if(Math.abs(m_turretTargetDegree) != 40){
        m_turretTargetDegree = 40;
        return;
      }
      m_turretTargetDegree *= -1;
    });
  }

  //-------------------------------------------------------------------------------------------------
  //    Manual Rotate Methods
  //-------------------------------------------------------------------------------------------------
  public void rotateLeft(){
    currentTurretState = TurretState.FULL_MANUAL;


      manualTurretPwr = 0.2;
  
  }
  
  
  public void rotateRight(){
    currentTurretState = TurretState.FULL_MANUAL;
  
      manualTurretPwr = 0.2;

  }

  //-------------------------------------------------------------------------------------------------
  //    Automated Methods
  //-------------------------------------------------------------------------------------------------
  public void aimAtGoal(Translation2d goal, boolean aimAtVision, boolean acctRobotVel) {
    Pose2d robotPose = SubsystemCatzDrivetrain.getInstance().getPose();

    //take difference between speaker and the curret robot translation
    Translation2d robotToGoal = goal.minus(robotPose.getTranslation());

    if(acctRobotVel){
      robotToGoal.div(Math.hypot(robotToGoal.getX(),robotToGoal.getY()));
      robotToGoal.times(SubsystemCatzShooter.getInstance().getApproximateShootingSpeed());
  
      robotToGoal.minus(new Translation2d(drivetrain.getFieldRelativeSpeed().vx,drivetrain.getFieldRelativeSpeed().vy));
    }


    //calculate new turret angle based off current robot position
    double angle = Math.atan2(robotToGoal.getY(), robotToGoal.getX());

    //offset new turret angle based off current robot rotation
    angle = angle - CatzMathUtils.toUnitCircAngle(robotPose.getRotation().getRadians()); 

    angle = Math.toDegrees(angle);

    //if we purely just want to rely on apriltag for aiming
    if (aimAtVision && SubsystemCatzVision.getInstance().getAprilTagID(1) == 7) {
      currentTurretState = TurretState.TRACKING_APRILTAG;
    } else {
      m_turretTargetDegree = angle;
      currentTurretState = TurretState.AUTO;
    }
      m_turretIntPos = false;
  }
  
  public void setTurretTargetDegree(double turretTargetDegree) {
    m_turretIntPos = false;
    currentTurretState = TurretState.AUTO;
    m_turretTargetDegree = turretTargetDegree;
  }

  //-------------------------------------------------------------------------------------------------
  //    Turret getters
  //-------------------------------------------------------------------------------------------------
  public double getTurretAngle() {
    return currentTurretDegree;
  }

  private TurretState getTurretState() {
    return currentTurretState;
  }

  public boolean getTurretInPos() {
    return m_turretIntPos;
  }
  
  //-------------------------------------------------------------------------------------------------
  //    Manual Methods
  //-------------------------------------------------------------------------------------------------
  public Command cmdTurretLT() {
    return run(() -> rotateLeft());
  }
  
  public Command cmdTurretRT() {
    return run(() -> rotateRight());
  }
  
  public Command cmdTurretOff() {
    currentTurretState = TurretState.FULL_MANUAL;
    return run(() -> io.turretSetPwr(0.0));
  }
  
  public Command cmdResetTurretPosition(){
    return run(() -> io.turretSetEncoderPos(HOME_POSITION));
  }
  
  public Command cmdTurretDegree(double turretDeg) {
    return run(() -> setTurretTargetDegree(turretDeg));
  }

  public Command cmdAutoRotate() {
    return run(() -> aimAtGoal(new Translation2d(), true, true));
  }
  
  public void updateTargetPositionTurret(CatzMechanismPosition newPosition) {
    currentTurretState = TurretState.AUTO;
    m_turretTargetDegree = newPosition.getTurretTargetAngle();
  }
}
