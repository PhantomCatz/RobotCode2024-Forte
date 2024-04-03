// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import java.sql.Driver;
import java.util.function.Supplier;

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
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterNoteState;
import frc.robot.subsystems.vision.SubsystemCatzVision;


public class SubsystemCatzTurret extends SubsystemBase {
  //intake io block     //TBD - Delete not very helpful and incorrect
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  //intake instance     //TBD - Delete not very helpful
  private static SubsystemCatzTurret instance = new SubsystemCatzTurret();


  // -----------------------------------------------------------------------------------------------
  //
  // Turret Definitions & Variables
  //
  // -----------------------------------------------------------------------------------------------
  // -----------------------------------------------------------------------------------------------
  //  Turret Gearbox Defs
  // -----------------------------------------------------------------------------------------------
  private static final double TURRET_PLANETARY_GEARBOX_RATIO =   9.0;         //TBD

  private static final double TURRET_GEARBOX_DRIVING_GEAR    =  10.0;        //Pinion Gear
  private static final double TURRET_GEARBOX_DRIVEN_GEAR     = 140.0;        //Turret Gear
  private static final double TURRET_GEARBOX_RATIO           = TURRET_GEARBOX_DRIVEN_GEAR / TURRET_GEARBOX_DRIVING_GEAR; 
 
  public static final double TURRET_GEAR_REDUCTION           = TURRET_PLANETARY_GEARBOX_RATIO * TURRET_GEARBOX_RATIO;    
  public static final double TURRET_MOTOR_SHAFT_REV_PER_DEG  = TURRET_GEAR_REDUCTION / 360.0;     //TBD 
  

  //------------------------------------------------------------------------------------------------
  //  Position Defs & Variables
  //------------------------------------------------------------------------------------------------
  public static final double TURRET_MAX_ANGLE_DEG =  120.0;
  public static final double HOME_POSITION_DEG    =    0.0;  
  public static final double TURRET_MIN_ANGLE_DEG = -120.0;

  public static final double TURRET_MAX_SERVO_LIMIT_DEG =  60.0;
  public static final double TURRET_MIN_SERVO_LIMIT_DEG = -60.0;  

  public static final double SERVO_TURRET_CONSTRAINT = 0.5;

  public static double currentTurretDegree = 0.0; 


  // -----------------------------------------------------------------------------------------------
  //  Turret Closed Loop Processing (PIDF, etc)
  // -----------------------------------------------------------------------------------------------
  private static final double TURRET_kP = 0.02;
  private static final double TURRET_kI = 0.0;
  private static final double TURRET_kD = 0.0;
  
  private static final double LIMELIGHT_kP = 0.01;//0.013;
  private static final double LIMELIGHT_kI = 0.0;
  private static final double LIMELIGHT_kD = 0.0;

  private final double TURRET_ANGLE_THRESHOLD_DEG = 3.0;
  private final double TURRET_APRILTAG_OFFSET_THRESHOLD = 5.0;

  private double m_turretTargetDegree;
  private double m_closedLoopError;
  private double setPositionPower;

  private boolean m_trackTarget = false;

  private double apriltagTrackingPower;

  private double offsetAprilTagX;   //TBD

  private PIDController m_setPositionPID;
  private PIDController m_trackingApriltagPID;


  private boolean m_turretInPos;

  private XboxController driveRumbleController;   //TBD - How does this work with Shooter rumble?

  private final SubsystemCatzDrivetrain drivetrain = SubsystemCatzDrivetrain.getInstance();   //TBD - How does this work?


  // -----------------------------------------------------------------------------------------------
  // Control Mode Defs
  // -----------------------------------------------------------------------------------------------
  public static enum TurretState {
    AUTO,
    TRACKING_APRILTAG,
    FULL_MANUAL
  }
  private static TurretState m_currentTurretState;


  //------------------------------------------------------------------------------------------------
  //  Manual Power Defs & Variables
  //------------------------------------------------------------------------------------------------
  private final double TURRET_POWER_SCALE = 0.5;

  private double  manualTurretPwr;

  //-----------------------------------------------------------------------------------------------
  //
  //  SubsystemCatzTurret()
  //
  //-----------------------------------------------------------------------------------------------
  private SubsystemCatzTurret() {

    switch (CatzConstants.currentMode) {
      case REAL: io = new TurretIOReal();
      break;

      case SIM : io = null;
      break;

      case REPLAY: io = new TurretIOReal() {};
      break;

      default: io = null;   //TBD - Update to match Intake/Elevator
      break;
    }


    m_setPositionPID      = new PIDController(TURRET_kP,    TURRET_kI,    TURRET_kD);
    m_trackingApriltagPID = new PIDController(LIMELIGHT_kP, LIMELIGHT_kI, LIMELIGHT_kD);

  }
  
  // Get the singleton instance of the Turret Subsystem
  public static SubsystemCatzTurret getInstance() {
      return instance;
  }


  // -------------------------------------------------------------------------------------
  //
  //  periodic()
  //
  // -------------------------------------------------------------------------------------
  @Override
  public void periodic() {
  
    io.updateInputs(inputs);
    Logger.processInputs("turret", inputs);

    currentTurretDegree = -inputs.turretEncValue; 
    
    //set turret constraint if shooter is greater than threshold
    if(SubsystemCatzShooter.getInstance().getServoCommandedPosition() > SERVO_TURRET_CONSTRAINT) {

      if(m_turretTargetDegree > TURRET_MAX_SERVO_LIMIT_DEG) {

        m_turretTargetDegree = TURRET_MAX_SERVO_LIMIT_DEG;
      } else if(m_turretTargetDegree < TURRET_MIN_SERVO_LIMIT_DEG) {

        m_turretTargetDegree = TURRET_MIN_SERVO_LIMIT_DEG;
      }
    } 

    //obtain calculation values
    setPositionPower  =  -m_setPositionPID.calculate(currentTurretDegree, m_turretTargetDegree);
    m_closedLoopError = Math.abs(m_turretTargetDegree - currentTurretDegree);



    if(DriverStation.isDisabled()) {
      setTurretDisabled();
    } else {

      if(m_currentTurretState == TurretState.FULL_MANUAL) {
        //------------------------------------------------------------------------------------------
        //  Manual Mode - Use Operator input to set turret motor power % output
        //------------------------------------------------------------------------------------------
        if(SubsystemCatzIntake.getInstance().getWristAngle() < SubsystemCatzIntake.INTAKE_TURRET_CLEARANCE) {   
          //------------------------------------------------------------------------------------------    
          //  Intake angle is wihin valid range.
          //------------------------------------------------------------------------------------------
          io.turretSetPwr(manualTurretPwr);
        }

      } else if(m_currentTurretState == TurretState.AUTO) {
        //------------------------------------------------------------------------------------------
        //  Auto Mode - Use PID to go to specified angle
        //------------------------------------------------------------------------------------------

        if(SubsystemCatzIntake.getInstance().getWristAngle() < SubsystemCatzIntake.INTAKE_TURRET_CLEARANCE) {  
          //------------------------------------------------------------------------------------------    
          //  Intake angle is wihin valid range.
          //------------------------------------------------------------------------------------------
          io.turretSetPwr(setPositionPower);
        }
   
      } else if (m_currentTurretState == TurretState.TRACKING_APRILTAG) {
        //------------------------------------------------------------------------------------------   
        //  TRACKING_APRILTAG Mode - Use shooter limelight to track April Tag 7 or 4 to determine
        //  turret angle go to specified angle
        //  only track the shooterlimelight to the speaker apriltag  
        //------------------------------------------------------------------------------------------
        if(SubsystemCatzVision.getInstance().getAprilTagID(2) == 7 ||
           SubsystemCatzVision.getInstance().getAprilTagID(2) == 4) {
          
          offsetAprilTagX       = SubsystemCatzVision.getInstance().getOffsetX(2);
          apriltagTrackingPower = -m_trackingApriltagPID.calculate(offsetAprilTagX, 0.0);
          io.turretSetPwr(apriltagTrackingPower);
        }
      } 

    }


    //In position check
    if(m_currentTurretState == TurretState.AUTO){
      if(m_closedLoopError < TURRET_ANGLE_THRESHOLD_DEG) {     
        m_turretInPos = true;
      }else{
        m_turretInPos = false;
      }
    }
    else if(m_currentTurretState == TurretState.TRACKING_APRILTAG){
      if(Math.abs(offsetAprilTagX) < TURRET_APRILTAG_OFFSET_THRESHOLD){
        m_turretInPos = true;
      }else{
        m_turretInPos = false;
      }
    }

    Logger.recordOutput("turret/offsetXTurret",        offsetAprilTagX);
    //Logger.recordOutput("turret/PwrPID", apriltagTrackingPower);
    // Logger.recordOutput("turret/currentTurretState", currentTurretState);
    Logger.recordOutput("turret/currentTurretDegee",   currentTurretDegree);
    Logger.recordOutput("turret/closedlooperror",      m_closedLoopError);
    Logger.recordOutput("turret/m_TurretTargetDegree", m_turretTargetDegree);
    Logger.recordOutput("turret/setpositionpwr", setPositionPower);
    Logger.recordOutput("turret/m_TurretinPos", m_turretInPos);
  }   //End of periodic()


  //-------------------------------------------------------------------------------------------------
  //
  //  aimAtGoal()
  //
  //-------------------------------------------------------------------------------------------------
  public void aimAtGoal(Translation2d goal, boolean accountRobotVel) {

      //--------------------------------------------------------------------------------------------
      //  We are aiming using April Tags - Check if we are looking at he April Tag on the Speaker.
      //  If we are then we will TBD.  Otherwise we will TBD
      //--------------------------------------------------------------------------------------------
      // System.out.println(SubsystemCatzVision.getInstance().getAprilTagID(2));
      if(SubsystemCatzVision.getInstance().getAprilTagID(2) == 7 ||
         SubsystemCatzVision.getInstance().getAprilTagID(2) == 4){
          
        m_currentTurretState = TurretState.TRACKING_APRILTAG;
        // System.out.println("apriltag");
      }else{
        //--------------------------------------------------------------------------------------------
        //  We are aiming using Odometry
        //  collect drivetrain pose
        //--------------------------------------------------------------------------------------------
        // System.out.println("pose");
        Pose2d robotPose = SubsystemCatzDrivetrain.getInstance().getPose();
  
        //take difference between speaker and the currnet robot translation
        Translation2d roboDistanceFromSpeaker = goal.minus(robotPose.getTranslation());
        //--------------------------------------------------------------------------------------------
        //  If we are trying to aim while moving, then we need to take into account robot velocity
        //--------------------------------------------------------------------------------------------
        
        
        if(accountRobotVel){
          roboDistanceFromSpeaker.div(Math.hypot(roboDistanceFromSpeaker.getX(), roboDistanceFromSpeaker.getY())); //direction
          double shootingSpeedVelocity = SubsystemCatzShooter.getInstance().getScuffedShootingSpeed();
          roboDistanceFromSpeaker.times(shootingSpeedVelocity);  //magnitude
          roboDistanceFromSpeaker.minus(new Translation2d(drivetrain.getFieldRelativeSpeed().vx, 
                                                          drivetrain.getFieldRelativeSpeed().vy));
        }
  
        //--------------------------------------------------------------------------------------------
        //  Calculate new turret target angle in deg based off:
        //    - Current robot position
        //    - Current robot rotation
        //---------------------------------------------------------------------------------  -----------
        double angle = Math.atan2(roboDistanceFromSpeaker.getY(), roboDistanceFromSpeaker.getX());
  
        Logger.recordOutput("AutoAim/local turret target angle", angle);
  
        angle = CatzMathUtils.toUnitCircAngle(angle - robotPose.getRotation().getRadians() - 3.14); 
        Logger.recordOutput("AutoAim/global turret target angle", angle);
  
        m_turretTargetDegree = Math.toDegrees(angle);    //Convert from radians to deg
        if(m_turretTargetDegree > 180) {
          m_turretTargetDegree = m_turretTargetDegree - 360;
        } else if(m_turretTargetDegree < -180) {
          m_turretTargetDegree = m_turretTargetDegree + 360;
        }
  
  
        Logger.recordOutput("AutoAim/targetTurretDeg", m_turretTargetDegree);
        m_currentTurretState   = TurretState.AUTO;
      }



  }   //End of aimAtGoal()


  //-------------------------------------------------------------------------------------------------
  //    Turret getters
  //-------------------------------------------------------------------------------------------------
  public double getTurretAngle() {
    return currentTurretDegree;
  }

  public boolean getTurretInPos() {
    return m_turretInPos;
  }

  public void setTurretInPose(boolean state){
    m_turretInPos = state;
  }

  public void setTurretState(TurretState state){
    m_currentTurretState = state;
  }

  //-------------------------------------------------------------------------------------------------
  //    Manual Rotate Methods
  //-------------------------------------------------------------------------------------------------

  public Command cmdRotateTurretManualOn(Supplier<Double> power){
    return run(()->{
      m_currentTurretState = TurretState.FULL_MANUAL;
      manualTurretPwr = power.get() * TURRET_POWER_SCALE;
    }).alongWith(Commands.runOnce(
                  ()->SubsystemCatzIntake.getInstance().updateAutoTargetPositionIntake(
                            CatzMechanismConstants.AUTO_AIM_PRESET.getIntakePivotTargetAngle())));
  }

  //-------------------------------------------------------------------------------------------------
  //    Manual Methods
  //-------------------------------------------------------------------------------------------------
  
  public Command cmdTurretOff() {
    return run(() -> setTurretDisabled());
  }


  private void setTurretDisabled() {
      io.turretSetPwr(0.0);
      manualTurretPwr    = 0.0;
      m_currentTurretState = TurretState.FULL_MANUAL;  
  }

  //-------------------------------------------------------------------------------------------------
  //    Cmd Methods
  //-------------------------------------------------------------------------------------------------
  public Command cmdResetTurretPosition(){
    return run(() -> io.turretSetEncoderPos(HOME_POSITION_DEG));   
  }
  
  public Command cmdTurretDegree(double turretDeg) {            
    return run(() -> setTurretTargetDegree(turretDeg));
  }

  public void setTurretTargetDegree(double turretTargetDegree) {    
    m_turretInPos       = false;
    m_currentTurretState   = TurretState.AUTO;
    m_turretTargetDegree = turretTargetDegree;
  }
  
  public void updateTargetPositionTurret(CatzMechanismPosition newPosition) {     //TBD conver to top method and deletee after converting
    m_turretInPos       = false;
    m_currentTurretState = TurretState.AUTO;
    // System.out.println(newPosition.getTurretTargetAngle());
    m_turretTargetDegree = newPosition.getTurretTargetAngle();
  }

  public Command testTurretAngles(){   
    return run(()->{
      m_currentTurretState = TurretState.AUTO;
      if(Math.abs(m_turretTargetDegree) != 40){
        m_turretTargetDegree = 40;
        return;
      }
      m_turretTargetDegree *= -1;
    });
  }


}