package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Utils.CatzMechanismPosition;
/***
 * CatzConstants
 * @version 1.0
 * @author Kynam Lenghiem
 * 
 * This class is where reusable constants are defined
 ***/
public final class CatzConstants {
  public static final boolean tuningMode = true;
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

 public static final class OIConstants {

  public static final int XBOX_DRV_PORT = 0;
  public static final int XBOX_AUX_PORT = 1;

  public static final int kDriverYAxis = 1;
  public static final int kDriverXAxis = 0;
  public static final int kDriverRotAxis = 4;
  public static final int kDriverFieldOrientedButtonIdx = 1;

  public static final double kDeadband = 0.3;
  public static final double kOffPwr = 0.1;
  }

  public static final class CatzMechanismConstants {
    public static final CatzMechanismPosition POS_STOW = new CatzMechanismPosition(0, 0, 0, 0);
    //public static final CatzMechanismPosition NOTE_POS_HANDOFF = new CatzMechanismPosition();
    //public static final CatzMechanismPosition NOTE_POS_SCORING_SPEAKER = new CatzMechanismPosition();
    public static final CatzMechanismPosition NOTE_POS_SCORING_AMP = new CatzMechanismPosition(100000, -2.11, 0, 0);
    public static final CatzMechanismPosition NOTE_POS_INTAKE_GROUND = new CatzMechanismPosition(0, -3.3, 0, 0);
    //public static final CatzMechanismPosition NOTE_POS_INTAKE_SOURCE = new CatzMechanismPosition();
    //public static final CatzMechanismPosition POS_CLIMB_PREP = new CatzMechanismPosition();
    //public static final CatzMechanismPosition POS_CLIMB = new CatzMechanismPosition();
    //public static final CatzMechanismPosition POS_CLIMB_SCORE_TRAP = new CatzMechanismPosition();

  }

  public static final class VisionConstants {
    public static final double LOWEST_DISTANCE = Units.feetToMeters(10.0);
    
    public static final Transform3d LIMELIGHT_OFFSET = new Transform3d(0.0, 0.0, 0.0, new Rotation3d()); //tbd need to understand how these classese work transform3d vs translation3d
  }

  public static final class TrajectoryConstants {
    public static final double ALLOWABLE_POSE_ERROR = 0.05;
    public static final double ALLOWABLE_ROTATION_ERROR = 5;
  }

  public static final class MtrConfigConstants {
    //Falcon configuration constants
    public static final int     FALCON_CURRENT_LIMIT_AMPS            = 55;
    public static final int     FALCON_CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    public static final double  FALCON_CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    public static final boolean FALCON_ENABLE_CURRENT_LIMIT          = true;

    //Neo config constants
    public static final int     NEO_CURRENT_LIMIT_AMPS      = 30;

    //Neo 550 config constants
  }

  //--------------------------------------Drivetrain-------------------------------
  public static final class DriveConstants {

    public static final double LT_FRNT_OFFSET =  0.00406;//atlas 0.5112305378; //this one changed
    public static final double LT_BACK_OFFSET = -0.03950;//0.5446386386;
    public static final double RT_BACK_OFFSET = -0.75084;//0.7591109064;
    public static final double RT_FRNT_OFFSET =  0.55098;//0.5363121009;

    public static final int LT_FRNT_DRIVE_ID = 1;
    public static final int LT_BACK_DRIVE_ID = 3;
    public static final int RT_BACK_DRIVE_ID = 5;
    public static final int RT_FRNT_DRIVE_ID = 7;
    
    public static final int LT_FRNT_STEER_ID = 2;
    public static final int LT_BACK_STEER_ID = 4;
    public static final int RT_BACK_STEER_ID = 6;
    public static final int RT_FRNT_STEER_ID = 8;

    public static final int LT_FRNT_ENC_PORT = 9;
    public static final int LT_BACK_ENC_PORT = 8; //atlas 6
    public static final int RT_BACK_ENC_PORT = 7;
    public static final int RT_FRNT_ENC_PORT = 6; //atlas 8

    //--------------------------------------MTR CONFIGS------------------------------------

    public static final Pose2d initPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    private static final double ROBOT_WIDTH = Units.inchesToMeters(24);
    private static final double ROBOT_LENGTH = Units.inchesToMeters(25);

    public static final double ESTIMATION_COEFFICIENT = 0.025;

    private static final Translation2d SWERVE_LEFT_FRONT_LOCATION  = new Translation2d(ROBOT_LENGTH, ROBOT_WIDTH).div(2.0);
    private static final Translation2d SWERVE_LEFT_BACK_LOCATION   = new Translation2d(-ROBOT_LENGTH, ROBOT_WIDTH).div(2.0);
    private static final Translation2d SWERVE_RIGHT_BACK_LOCATION  = new Translation2d(-ROBOT_LENGTH, -ROBOT_WIDTH).div(2.0);
    private static final Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(ROBOT_LENGTH, -ROBOT_WIDTH).div(2.0);
    
    // calculates the orientation and speed of individual swerve modules when given the motion of the whole robot
    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        SWERVE_LEFT_FRONT_LOCATION,
        SWERVE_LEFT_BACK_LOCATION,
        SWERVE_RIGHT_BACK_LOCATION,
        SWERVE_RIGHT_FRONT_LOCATION
    );
    
    //data has been referenced using recalc calculator https://www.reca.lc/drive
    public static final double MAX_SPEED = Units.feetToMeters(14.34); // meters per second 4.81

    public static final double MAX_ANGSPEED_RAD_PER_SEC = 12.0; // radians per second
    public static final double MAX_SPEED_DESATURATION = MAX_SPEED; 

    public static final double SDS_L1_GEAR_RATIO = 8.14;       //SDS mk4i L1 ratio reduction
    public static final double SDS_L2_GEAR_RATIO = 6.75;       //SDS mk4i L2 ratio reduction
    
    public static final double DRVTRAIN_WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);//atlas 0.095 m
    public static final double DRVTRAIN_WHEEL_CIRCUMFERENCE   = (Math.PI * DRVTRAIN_WHEEL_DIAMETER_METERS);

    public static final double FEEDFOWARD_Kv_VELOCITY_METERS = 2.68;
    public static final double FEEDFOWARD_Kv_VELOCITY_ACCELERATION_METERS = 0.24;

    private static ProfiledPIDController autoTurnPIDController = new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(3,3));

    public static final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
      new PIDController(2, 0, 0),
      new PIDController(2, 0, 0),
      autoTurnPIDController
    );     
  }

  //any type of Elevator Mtr Config Constnats/Logic Constants should go here 
  public static final class ElevatorConstants {
    public static int ELEVATOR_MTR_ID = 50;
  }
  
  //any type of Intake Mtr Config Constnats/Logic Constants should go here 
  public static final class IntakeConstants {
    public static int PIVOT_MTR_ID = 12;
    public static int ROLLER_MTR_ID = 11;
  }

  //any type of Shooter Mtr Config Constnats/Logic Constants should go here 
  public static final class ShooterConstants {
    public static int SHOOTER_MTR_ID = 53;
    public static int TURRET_MTR_ID = 54;
  }
}