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
    public static final Transform3d LIMELIGHT_OFFSET   = new Transform3d(-Units.inchesToMeters(12), -Units.inchesToMeters(9), Units.inchesToMeters(20), new Rotation3d(0.0,0.0,180.0));
    public static final Transform3d LIMELIGHT_OFFSET_2 = new Transform3d(0.0, 0.0, 0.0, null); 
  }

  public static final class TrajectoryConstants {
    public static final double ALLOWABLE_POSE_ERROR = 0.05;
    public static final double ALLOWABLE_ROTATION_ERROR = 5;
  }

  /**
   * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
   * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
   * <br>
   *
   * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
   * ALLIANCE wall.<br>
   * <br>
   * Length refers to the <i>x</i> direction (as described by wpilib) <br>
   * Width refers to the <i>y</i> direction (as described by wpilib)
   */
  public class FieldConstants {
    public static final double SPEAKER_COORD_MTRS_Y = Units.inchesToMeters(219.277);
    public static double FIELD_LENGTH_MTRS = Units.inchesToMeters(651.223);
    public static double fieldWidth = Units.inchesToMeters(323.277);
    public static double wingX = Units.inchesToMeters(229.201);
    public static double podiumX = Units.inchesToMeters(126.75);
    public static double startingLineX = Units.inchesToMeters(74.111);

    public static Translation2d ampCenter =
        new Translation2d(Units.inchesToMeters(72.455), Units.inchesToMeters(322.996));

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
          centerlineTranslations[i] =
              new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
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
      public static Translation3d topRightSpeaker =
          new Translation3d(
              Units.inchesToMeters(18.055),
              Units.inchesToMeters(238.815),
              Units.inchesToMeters(13.091));

      public static Translation3d topLeftSpeaker =
          new Translation3d(
              Units.inchesToMeters(18.055),
              Units.inchesToMeters(197.765),
              Units.inchesToMeters(83.091));

      public static Translation3d bottomRightSpeaker =
          new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
      public static Translation3d bottomLeftSpeaker =
          new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

      /** Center of the speaker opening (blue alliance) */
      public static Translation3d centerSpeakerOpening =
          new Translation3d(
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

  public static final class ShooterConstants{
    public static final double WHEEL_DIAMETER = 0.0762; //in meters
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

  }

  //--------------------------------------Drivetrain-------------------------------
  public static final class DriveConstants {
    //sn2
    public static final double LT_FRNT_OFFSET = 0.7364;             //mag encoder 0
    public static final double LT_BACK_OFFSET = -0.2732+0.5 ;      //mag encoder 1
    public static final double RT_BACK_OFFSET = 0.138439;          //mag encoder 2
    public static final double RT_FRNT_OFFSET = 0.8088+0.5;        //mag encoder 3

    //sn1
    // public static final double LT_FRNT_OFFSET =  0.21317;
    // public static final double LT_BACK_OFFSET = 0.25727+0.5;//0.5446386386;
    // public static final double RT_BACK_OFFSET = -0.1986;//0.7591109064;
    // public static final double RT_FRNT_OFFSET = -0.00320;//0.536312100;


    public static final int LT_FRNT_DRIVE_ID = 1;
    public static final int LT_BACK_DRIVE_ID = 3;
    public static final int RT_BACK_DRIVE_ID = 5;
    public static final int RT_FRNT_DRIVE_ID = 7; 
    
    public static final int LT_FRNT_STEER_ID = 2;
    public static final int LT_BACK_STEER_ID = 4;
    public static final int RT_BACK_STEER_ID = 6;
    public static final int RT_FRNT_STEER_ID = 8;

    public static final int LT_FRNT_ENC_PORT = 9;
    public static final int LT_BACK_ENC_PORT = 8; 
    public static final int RT_BACK_ENC_PORT = 7;
    public static final int RT_FRNT_ENC_PORT = 6; 

    //--------------------------------------MTR CONFIGS------------------------------------
    public static final Rotation2d defaultRot = new Rotation2d(0.0);
    private static final double ROBOT_WIDTH = Units.inchesToMeters(23.5); //29 atlas
    private static final double ROBOT_LENGTH = Units.inchesToMeters(24); //29 atlas

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
    public static final double MAX_SPEED = 24.0;//Units.feetToMeters(14.34); // meters per second 4.81

    public static final double MAX_ANGSPEED_RAD_PER_SEC = 24.0; // radians per second
    public static final double MAX_SPEED_DESATURATION = MAX_SPEED; 

    public static final double SDS_L1_GEAR_RATIO = 8.14;       //SDS mk4i L1 ratio reduction
    public static final double SDS_L2_GEAR_RATIO = 6.75;       //SDS mk4i L2 ratio reduction
    public static final double SDS_L2_PLUS_GEAR_RATIO = 6.75 * (14/16);       //SDS mk4i L2 ratio reduction plud random numbers from eddy
    public static final double SDS_L2_GEAR_RATIO_PLUS_16T = 5.90;  //SDS mk4i L2+ ratio reduction 16th tooth
    public static final double SDS_L2_GEAR_RATIO_PLUS_15T = 6.30;  //SDS mk4i L2+ ratio reduction 15th tooth
    
                                                                //overtime
    public static final double DRVTRAIN_WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.3);//0.095;// mUnits.inchesToMeters(4);
    public static final double DRVTRAIN_WHEEL_CIRCUMFERENCE   = (Math.PI * DRVTRAIN_WHEEL_DIAMETER_METERS);

    public static final boolean START_FLIPPED = false;

    public static final double FEEDFOWARD_Kv_VELOCITY_METERS = 2.68;
    public static final double FEEDFOWARD_Kv_VELOCITY_ACCELERATION_METERS = 0.24;

    private static ProfiledPIDController autoTurnPIDController = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(3,3));//6

    public static final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
      new PIDController(2, 0, 0),
      new PIDController(2, 0, 0),
      autoTurnPIDController
    );
  }
  

  public static final class CatzMechanismConstants {
    public static final CatzMechanismPosition STOW_PRESET = 
                              new CatzMechanismPosition(SubsystemCatzElevator.ELEVATOR_STOW, 
                                                        SubsystemCatzIntake.INTAKE_STOW_DEG, 
                                                        1.0, 
                                                        SubsystemCatzTurret.HOME_POSITION_DEG);

    public static final CatzMechanismPosition SUBWOOFER_PRESET = 
                              new CatzMechanismPosition(SubsystemCatzElevator.ELEVATOR_STOW, 
                                                        SubsystemCatzIntake.INTAKE_STOW_DEG, 
                                                        1.0, 
                                                        SubsystemCatzTurret.HOME_POSITION_DEG);

    public static final CatzMechanismPosition SUBWOOFER_DEFENSE_PRESET = 
                              new CatzMechanismPosition(SubsystemCatzElevator.ELEVATOR_STOW, 
                                                        SubsystemCatzIntake.INTAKE_STOW_DEG, 
                                                        0.6, 
                                                        SubsystemCatzTurret.HOME_POSITION_DEG);

    public static final CatzMechanismPosition HOARD_PRESET = 
                              new CatzMechanismPosition(SubsystemCatzElevator.ELEVATOR_STOW, 
                                                        SubsystemCatzIntake.INTAKE_STOW_DEG, 
                                                        0.6, 
                                                        -30.0);

    public static final CatzMechanismPosition PREP_FOR_AMP_PRESET = 
                              new CatzMechanismPosition(SubsystemCatzElevator.ELEVATOR_STOW, 
                                                        SubsystemCatzIntake.INTAKE_AMP_SCORE_DN_DEG, 
                                                        SubsystemCatzShooter.SERVO_OPTIMAL_HANDOFF_POS, 
                                                        SubsystemCatzTurret.HOME_POSITION_DEG);
                                                        
    public static final CatzMechanismPosition AMP_TRANSITION_PRESET = 
                              new CatzMechanismPosition(SubsystemCatzElevator.ELEVATOR_AMP_TRANSITION, 
                                                        SubsystemCatzIntake.INTAKE_AMP_TRANSITION_DEG, 
                                                        SubsystemCatzShooter.SERVO_OPTIMAL_HANDOFF_POS, 
                                                        SubsystemCatzTurret.HOME_POSITION_DEG);
                                                      

    public static final CatzMechanismPosition SCORING_SPEAKER_PRESET = 
                              new CatzMechanismPosition(SubsystemCatzElevator.ELEVATOR_STOW,
                                                        SubsystemCatzIntake.INTAKE_AMP_SCORE_DN_DEG,
                                                        SubsystemCatzShooter.SERVO_OPTIMAL_HANDOFF_POS,  
                                                        SubsystemCatzTurret.HOME_POSITION_DEG);

    public static final CatzMechanismPosition SCORING_AMP_PRESET = 
                              new CatzMechanismPosition(SubsystemCatzElevator.ELEVATOR_AMP_SCORE, 
                                                        SubsystemCatzIntake.INTAKE_AMP_SCORE_DEG, 
                                                        SubsystemCatzShooter.SERVO_OPTIMAL_HANDOFF_POS,  
                                                        SubsystemCatzTurret.HOME_POSITION_DEG);

    public static final CatzMechanismPosition INTAKE_GROUND_PRESET = 
                              new CatzMechanismPosition(SubsystemCatzElevator.ELEVATOR_STOW, 
                                                        SubsystemCatzIntake.INTAKE_GROUND_PICKUP_DEG, 
                                                        SubsystemCatzShooter.SERVO_OPTIMAL_HANDOFF_POS, 
                                                        SubsystemCatzTurret.HOME_POSITION_DEG);

    public static final CatzMechanismPosition INTAKE_SOURCE_PRESET = 
                              new CatzMechanismPosition(SubsystemCatzElevator.ELEVATOR_SOURCE_PICKUP,
                                                        SubsystemCatzIntake.INTAKE_SOURCE_LOAD_UP_DEG,
                                                        SubsystemCatzShooter.SERVO_OPTIMAL_HANDOFF_POS, 
                                                        SubsystemCatzTurret.HOME_POSITION_DEG);

    public static final CatzMechanismPosition SCORING_TRAP_PRESET = 
                              new CatzMechanismPosition(SubsystemCatzElevator.ELEVATOR_SCORE_TRAP, 
                                                        119.0,
                                                        SubsystemCatzShooter.SERVO_OPTIMAL_HANDOFF_POS,
                                                        SubsystemCatzTurret.HOME_POSITION_DEG);
    //-10
    
  }

  //COLOR CONSTANTS::
  public static final class CatzColorConstants {
    public static final Color PHANTOM_SAPPHIRE = new Color(15, 25, 200); 
  }


}