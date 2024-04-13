package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.turret.SubsystemCatzTurret;

import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import java.io.IOException;

/***
 * CatzConstants
 * 
 * @version 1.0
 * @author Kynam Lenghiem
 * 
 *         This class is where reusable constants are defined
 ***/
public final class CatzConstants {
  public static final boolean tuningMode = true;
  public static final Mode currentMode = Mode.REAL;
  public static final double LOOP_TIME = 0.02;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum AllianceColor {
    Blue, Red
  }

  public static final class OIConstants {

    public static final int XBOX_DRV_PORT = 0;
    public static final int XBOX_AUX_PORT = 1;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    public static final double kDeadband = 0.1;
    public static final double kOffPwr = 0.1;

  }

  public static final class VisionConstants {
    public static final double SPEAKER_HOOD_HEIGHT = 83.0;
    public static final double LOWEST_DISTANCE = Units.feetToMeters(10.0);
    public static final Transform3d LIMELIGHT_OFFSET = new Transform3d(-Units.inchesToMeters(12),
        -Units.inchesToMeters(9), Units.inchesToMeters(20), new Rotation3d(0.0, 0.0, 180.0));
    public static final Transform3d LIMELIGHT_OFFSET_2 = new Transform3d(0.0, 0.0, 0.0, null);
  }

  public static final class TrajectoryConstants {
    public static final double ALLOWABLE_POSE_ERROR = 0.05;
    public static final double ALLOWABLE_ROTATION_ERROR = 5;
  }

  /**
   * Contains various field dimensions and useful reference points. Dimensions are
   * in meters, and sets
   * of corners start in the lower left moving clockwise. <b>All units in
   * Meters</b> <br>
   * <br>
   *
   * <p>
   * All translations and poses are stored with the origin at the rightmost point
   * on the BLUE
   * ALLIANCE wall.<br>
   * <br>
   * Length refers to the <i>x</i> direction (as described by wpilib) <br>
   * Width refers to the <i>y</i> direction (as described by wpilib)
   */
  public class FieldConstants {

    public static final double SPEAKER_COORD_MTRS_Y = Units.inchesToMeters(219.277);
    public static final double HOARD_LOCATION_Y = Units.inchesToMeters(219.277) + 2.0;
    public static double FIELD_LENGTH_MTRS = Units.inchesToMeters(651.223);
    public static double fieldWidth = Units.inchesToMeters(323.277);
    public static double wingX = Units.inchesToMeters(229.201);
    public static double podiumX = Units.inchesToMeters(126.75);
    public static double startingLineX = Units.inchesToMeters(74.111);

    public static Translation2d ampCenter = new Translation2d(Units.inchesToMeters(72.455),
        Units.inchesToMeters(322.996));

    /** Staging locations for each note */
    public static final class StagingLocations {
      public static double centerlineX = FIELD_LENGTH_MTRS / 2.0;

      // need to update
      public static double centerlineFirstY = Units.inchesToMeters(29.638);
      public static double centerlineSeparationY = Units.inchesToMeters(66);
      public static double spikeX = Units.inchesToMeters(114);
      // need
      public static double spikeFirstY = Units.inchesToMeters(161.638);
      public static double spikeSeparationY = Units.inchesToMeters(57);

      public static Translation2d[] centerlineTranslations = new Translation2d[5];
      public static Translation2d[] spikeTranslations = new Translation2d[3];

      static {
        for (int i = 0; i < centerlineTranslations.length; i++) {
          centerlineTranslations[i] = new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
        }
      }

      static {
        for (int i = 0; i < spikeTranslations.length; i++) {
          spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
        }
      }
    }

    /** Each corner of the speaker * */
    public static final class Speaker {

      // corners (blue alliance origin)
      public static Translation3d topRightSpeaker = new Translation3d(
          Units.inchesToMeters(18.055),
          Units.inchesToMeters(238.815),
          Units.inchesToMeters(13.091));

      public static Translation3d topLeftSpeaker = new Translation3d(
          Units.inchesToMeters(18.055),
          Units.inchesToMeters(197.765),
          Units.inchesToMeters(83.091));

      public static Translation3d bottomRightSpeaker = new Translation3d(0.0, Units.inchesToMeters(238.815),
          Units.inchesToMeters(78.324));
      public static Translation3d bottomLeftSpeaker = new Translation3d(0.0, Units.inchesToMeters(197.765),
          Units.inchesToMeters(78.324));

      /** Center of the speaker opening (blue alliance) */
      public static Translation3d centerSpeakerOpening = new Translation3d(
          topLeftSpeaker.getX() / 2.0,
          fieldWidth - Units.inchesToMeters(104.0),
          (bottomLeftSpeaker.getZ() + bottomRightSpeaker.getZ()) / 2.0);
    }

    public static double aprilTagWidth = Units.inchesToMeters(6.50);
    public static AprilTagFieldLayout aprilTags;

    static {
      try {
        aprilTags = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
      } catch (IOException e) {
        throw new RuntimeException(e);
      }
    }
  }

  public static final class ShooterConstants {
    public static final double WHEEL_DIAMETER = 0.0762; // in meters
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

  }

  // --------------------------------------Drivetrain-------------------------------
  public static final class DriveConstants {
    
  /* ---------------------------------------------------------------------------------
  * SN2 OFFSETS
  -----------------------------------------------------------------------------------*/ 
    // public static final double LT_FRNT_OFFSET = 0.228031255+0.5;// 0.23400625 + 0.5;
    // public static final double LT_BACK_OFFSET = 0.733477518+0.5;//-0.26659156 + 0.5;
    // public static final double RT_BACK_OFFSET = 1.1043222;// 0.10567640; 
    // public static final double RT_FRNT_OFFSET = 0.3417887;// 1.32572595;
  
  /* ---------------------------------------------------------------------------------
  * SN1 OFFSETS
  -----------------------------------------------------------------------------------*/ 
    public static final double LT_FRNT_OFFSET = 1.2307227057;//0.207405905185;//0.20907;//0.21317;
    public static final double LT_BACK_OFFSET = 0.24567763114+0.5;//0.7495236687;//0.250096+0.5;//0.25727+0.5;//0.5446386386;
    public static final double RT_BACK_OFFSET = -0.1892973047;//-0.19950937998;//-0.199788;//-0.1986;//0.7591109064;
    public static final double RT_FRNT_OFFSET = 0.010002000;//0.007320000183;//-0.00320;//0.536312100;

  //---------------------------------------------------------------------------------

  //DRIVE MOTORS ID
    public static final int LT_FRNT_DRIVE_ID = 1;
    public static final int LT_BACK_DRIVE_ID = 3;
    public static final int RT_BACK_DRIVE_ID = 5;
    public static final int RT_FRNT_DRIVE_ID = 7; 

  //STEER MOTORS ID
    public static final int LT_FRNT_STEER_ID = 2;
    public static final int LT_BACK_STEER_ID = 4;
    public static final int RT_BACK_STEER_ID = 6;
    public static final int RT_FRNT_STEER_ID = 8;

  //MAG ENCODERS ID
    public static final int LT_FRNT_ENC_PORT = 9;
    public static final int LT_BACK_ENC_PORT = 8; 
    public static final int RT_BACK_ENC_PORT = 7;
    public static final int RT_FRNT_ENC_PORT = 6; 

    // --------------------------------------MTR CONFIGS------------------------------------

    public static final Rotation2d defaultRot = new Rotation2d(0.0);
    private static final double ROBOT_WIDTH = Units.inchesToMeters(23.5); 
    private static final double ROBOT_LENGTH = Units.inchesToMeters(24); 

    public static final double ESTIMATION_COEFFICIENT = 0.025;

    private static final Translation2d SWERVE_LEFT_FRONT_LOCATION = new Translation2d(ROBOT_LENGTH, ROBOT_WIDTH)
        .div(2.0);
    private static final Translation2d SWERVE_LEFT_BACK_LOCATION = new Translation2d(-ROBOT_LENGTH, ROBOT_WIDTH)
        .div(2.0);
    private static final Translation2d SWERVE_RIGHT_BACK_LOCATION = new Translation2d(-ROBOT_LENGTH, -ROBOT_WIDTH)
        .div(2.0);
    private static final Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(ROBOT_LENGTH, -ROBOT_WIDTH)
        .div(2.0);

    // calculates the orientation and speed of individual swerve modules when given
    // the motion of the whole robot
    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        SWERVE_LEFT_FRONT_LOCATION,
        SWERVE_LEFT_BACK_LOCATION,
        SWERVE_RIGHT_BACK_LOCATION,
        SWERVE_RIGHT_FRONT_LOCATION);

    // data has been referenced using recalc calculator https://www.reca.lc/drive
    public static final double MAX_SPEED = Units.feetToMeters(25.0); // meters per second 4.81

    public static final double MAX_ANGSPEED_RAD_PER_SEC = 12.0; // radians per second
    public static final double MAX_SPEED_DESATURATION = MAX_SPEED;

    public static final double SDS_L2_PLUS_GEAR_RATIO = 6.75 * (14.0 / 16.0); // SDS mk4i L2 ratio reduction plus 16 tooth pinion

    public static final double DRVTRAIN_WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    public static final double DRVTRAIN_WHEEL_CIRCUMFERENCE = (Math.PI * DRVTRAIN_WHEEL_DIAMETER_METERS);

    public static final double FEEDFOWARD_Kv_VELOCITY_METERS = 2.68;
    public static final double FEEDFOWARD_Kv_VELOCITY_ACCELERATION_METERS = 0.24;

    private static ProfiledPIDController autoTurnPIDController = new ProfiledPIDController(5, 0, 0,
        new TrapezoidProfile.Constraints(4.8, 3));// 6

    public static final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        new PIDController(3.0, 0, 0.015),
        new PIDController(3.0, 0, 0.015), //TBD LISA
        autoTurnPIDController);
  }

  public static final class CatzMechanismConstants {


  /* ---------------------------------------------------------------------------------
  * INTAKE MISC. PRESETS
  -----------------------------------------------------------------------------------*/    
    public static final CatzMechanismPosition STOW_PRESET = new CatzMechanismPosition(
        SubsystemCatzElevator.ELEVATOR_STOW,
        SubsystemCatzIntake.INTAKE_STOW_DEG,
        SubsystemCatzShooter.SERVO_IGNORE_POSITION,
        SubsystemCatzTurret.HOME_POSITION_DEG);
    
    public static final CatzMechanismPosition INTAKE_GROUND_PRESET = new CatzMechanismPosition(
        SubsystemCatzElevator.ELEVATOR_STOW,
        SubsystemCatzIntake.INTAKE_GROUND_PICKUP_DEG,
        SubsystemCatzShooter.SERVO_IGNORE_POSITION,
        SubsystemCatzTurret.HOME_POSITION_DEG);

    public static final CatzMechanismPosition INTAKE_SOURCE_PRESET = new CatzMechanismPosition(
        SubsystemCatzElevator.ELEVATOR_SOURCE_PICKUP,
        SubsystemCatzIntake.INTAKE_SOURCE_LOAD_UP_DEG,
        SubsystemCatzShooter.SERVO_IGNORE_POSITION,
        SubsystemCatzTurret.HOME_POSITION_DEG);
 /* ---------------------------------------------------------------------------------
  * SPEAKER PRESETS
  -----------------------------------------------------------------------------------*/ 
    public static final CatzMechanismPosition SUBWOOFER_PRESET = new CatzMechanismPosition(
        SubsystemCatzElevator.ELEVATOR_STOW,
        SubsystemCatzIntake.INTAKE_STOW_DEG,
        SubsystemCatzShooter.SERVO_MAX_POS,
        SubsystemCatzTurret.HOME_POSITION_DEG);
        
    public static final CatzMechanismPosition AUTO_AIM_PRESET = new CatzMechanismPosition(
        SubsystemCatzElevator.ELEVATOR_STOW,
        SubsystemCatzIntake.INTAKE_AMP_SCORE_DN_DEG,
        SubsystemCatzShooter.SERVO_IGNORE_POSITION,
        -999.0);
 /* ---------------------------------------------------------------------------------
  * HOARD PRESETS
  -----------------------------------------------------------------------------------*/ 
    public static final CatzMechanismPosition INTAKE_HOARD_PRESET = new CatzMechanismPosition(
        SubsystemCatzElevator.ELEVATOR_STOW,
        
        SubsystemCatzIntake.INTAKE_HOARD_DEG,
        SubsystemCatzShooter.SERVO_IGNORE_POSITION,
        SubsystemCatzTurret.HOME_POSITION_DEG);

    public static final CatzMechanismPosition SHOOTER_HOARD_PRESET = new CatzMechanismPosition(
        SubsystemCatzElevator.ELEVATOR_STOW,
        SubsystemCatzIntake.INTAKE_STOW_DEG,
        0.5,
        SubsystemCatzTurret.HOME_POSITION_DEG);


 /* ---------------------------------------------------------------------------------
  * AMP PRESETS
  -----------------------------------------------------------------------------------*/ 
    public static final CatzMechanismPosition PREP_FOR_AMP_PRESET = new CatzMechanismPosition(
        SubsystemCatzElevator.ELEVATOR_STOW,
        SubsystemCatzIntake.INTAKE_AMP_SCORE_DN_DEG,
        SubsystemCatzShooter.SERVO_IGNORE_POSITION,
        SubsystemCatzTurret.HOME_POSITION_DEG);

    public static final CatzMechanismPosition SCORING_AMP_PRESET = new CatzMechanismPosition(
        SubsystemCatzElevator.ELEVATOR_AMP_SCORE,
        SubsystemCatzIntake.INTAKE_AMP_SCORE_DEG,
        SubsystemCatzShooter.SERVO_IGNORE_POSITION,
        SubsystemCatzTurret.HOME_POSITION_DEG);

 /* ---------------------------------------------------------------------------------
  * TRAP PRESETS
  -----------------------------------------------------------------------------------*/ 
    public static final CatzMechanismPosition SCORING_TRAP_PRESET = new CatzMechanismPosition(
        SubsystemCatzElevator.ELEVATOR_SCORE_TRAP,
        SubsystemCatzIntake.INTAKE_AMP_SCORE_DN_DEG,
        SubsystemCatzShooter.SERVO_MAX_POS,
        SubsystemCatzTurret.HOME_POSITION_DEG);

 
  }

  public static RobotMode currentRobotMode = RobotMode.SPEAKER;

  public enum RobotMode {
    SPEAKER,
    AMP,
    HOARD,
    CLIMB_MAINTENANCE_MODE,
    CLIMB
  }

  public enum NoteDestination {
    SPEAKER,
    AMP,
    TRAP,
    HOARD
  }

  public enum NoteSource {
    INTAKE_SOURCE,
    INTAKE_GROUND,
    FROM_SHOOTER,
    FROM_INTAKE,
    NULL
  }

  // COLOR CONSTANTS::
  public static final class CatzColorConstants {
    public static final Color PHANTOM_SAPPHIRE = new Color(15, 25, 200);
  }

}