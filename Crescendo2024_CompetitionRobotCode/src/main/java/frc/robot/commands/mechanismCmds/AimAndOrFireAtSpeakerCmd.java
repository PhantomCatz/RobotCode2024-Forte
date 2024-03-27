package frc.robot.commands.mechanismCmds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.littletonrobotics.junction.Logger;

import java.awt.geom.Point2D;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.CatzAutonomous;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.CatzConstants.FieldConstants;
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.Utils.FieldRelativeAccel;
import frc.robot.Utils.FieldRelativeSpeed;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.elevator.SubsystemCatzElevator.ElevatorControlState;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.intake.SubsystemCatzIntake.IntakeControlState;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.shooter.SubsystemCatzShooter.ShooterNoteState;
import frc.robot.subsystems.turret.SubsystemCatzTurret;
import frc.robot.subsystems.vision.SubsystemCatzVision;


public class AimAndOrFireAtSpeakerCmd extends Command {

  //subsystem declaration
  private SubsystemCatzElevator   elevator   = SubsystemCatzElevator.getInstance();
  private SubsystemCatzIntake     intake     = SubsystemCatzIntake.getInstance();
  private SubsystemCatzShooter    shooter    = SubsystemCatzShooter.getInstance();
  private SubsystemCatzTurret     turret     = SubsystemCatzTurret.getInstance();
  private SubsystemCatzDrivetrain drivetrain = SubsystemCatzDrivetrain.getInstance();

  //------------------------------------------------------------------------------------------------
  //
  // Interpolation tables
  //
  //------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------
  //  Shooter EL angle look up table key: 
  //    Param 1: Distance in meters from back wall to Center of the robot
  //    Param 2: pivot position % of max elevation units
  // TBD - how did we determine distance interval?
  // TBD - explain why two distance values
  //------------------------------------------------------------------------------------------------
  private static final InterpolatingDoubleTreeMap shooterPivotTable = new InterpolatingDoubleTreeMap();

  static { 
    shooterPivotTable.put(1.37, 0.650);     //53.93701 inches     Shooted from lining up against the subwoofer
    //shooterPivotTable.put(1.37, 0.600);     
    //shooterPivotTable.put(1.37, 0.700);

    shooterPivotTable.put(1.87, 0.575);    
    //shooterPivotTable.put(1.87, 0.650);     //73.62205
    //shooterPivotTable.put(1.87, 0.500);

    shooterPivotTable.put(2.37, 0.325);
    //shooterPivotTable.put(2.37, 0.350);     //93.30709
    //shooterPivotTable.put(2.37, 0.300);

    shooterPivotTable.put(2.87, 0.290); 
    //shooterPivotTable.put(2.87, 0.300);     //112.9921
    //shooterPivotTable.put(2.87, 0.280); 

    shooterPivotTable.put(3.37, 0.225);
    //shooterPivotTable.put(3.37, 0.250);     //132.6772
    //shooterPivotTable.put(3.37, 0.200);

    shooterPivotTable.put(3.87, 0.112);
    //shooterPivotTable.put(3.87, 0.125);     //152.3622  
    //shooterPivotTable.put(3.87, 0.100);

    shooterPivotTable.put(4.87, 0.070);
    //shooterPivotTable.put(4.87, 0.100);     //191.7323
    //shooterPivotTable.put(4.87, 0.050);

    shooterPivotTable.put(5.87, 0.000);     //231.1024

  }

  //------------------------------------------------------------------------------------------------    TBD
  //  time table look up for calculating how long it takes to get note into speaker
  //  angle to time look up table key: ty angle, values: time */
  //  Currently NOT IN USE
  //------------------------------------------------------------------------------------------------
  private static final InterpolatingDoubleTreeMap timeTable = new InterpolatingDoubleTreeMap();
      // (distance, time seconds)
  static { 
        // (ty-angle,time)              TBD - indent
        timeTable.put(1.37, 0.780);
        timeTable.put(2.37, 0.800);
        timeTable.put(2.87, 0.810);
        timeTable.put(3.37, 0.820);
        timeTable.put(4.87, 0.825);
        timeTable.put(5.87, 0.830);
  }


  public static final double k_ACCEL_COMP_FACTOR = 0.100; // in units of seconds    TBD Where is this used?

  private Translation2d m_targetXY;

  //------------------------------------------------------------------------------------------------
  //
  //  
  //
  //------------------------------------------------------------------------------------------------
  private Supplier<Boolean> m_bSupplier;
  

  //for telop
  public AimAndOrFireAtSpeakerCmd(Supplier<Boolean> bSupplier) {
    m_bSupplier = bSupplier;
    addRequirements(turret, shooter, intake, elevator);
  }

  //for autonomous
  public AimAndOrFireAtSpeakerCmd() {                     
    addRequirements(turret, shooter, intake, elevator);
  }

  
  //------------------------------------------------------------------------------------------------
  //
  //  initialize()
  //
  // Called when the command is initially scheduled.
  //------------------------------------------------------------------------------------------------
  @Override
  public void initialize() {
    shooter.startShooterFlywheel();

    intake.updateAutoTargetPositionIntake(CatzMechanismConstants.AUTO_AIM_PRESET.getIntakePivotTargetAngle());
    elevator.updateTargetPositionElevator(CatzMechanismConstants.AUTO_AIM_PRESET.getElevatorTargetRev());

    if(CatzAutonomous.allianceColor == CatzConstants.AllianceColor.Blue) {    //TBD - we should do this once on startup vs every cmd call //TTTchanging to red 
      
      //translation of the blue alliance speaker
      m_targetXY = new Translation2d(0.0, FieldConstants.SPEAKER_COORD_MTRS_Y);
      System.out.println("blue tracking");
    } else {
      
      //translation of the Red alliance speaker
      m_targetXY = new Translation2d(0.0 + CatzConstants.FieldConstants.FIELD_LENGTH_MTRS , FieldConstants.SPEAKER_COORD_MTRS_Y);      //TBD - Magic #'s, what about defining Red & Blue constants, using IF to select and have 1 translation2D() call
      System.out.println("red tracking");
    }


  }

  
  //------------------------------------------------------------------------------------------------
  //
  //  execute()
  //
  //------------------------------------------------------------------------------------------------
  @Override 
  public void execute() {

    double newDist = m_targetXY.getDistance(drivetrain.getPose().getTranslation());
    double servoPos = shooterPivotTable.get(newDist);
    turret.aimAtGoal(m_targetXY, false, false);    
    shooter.updateShooterServo(servoPos);

    //in telop this boolean supplier is being evaluated to see if button was pressed
    if(m_bSupplier != null &&
       m_bSupplier.get() == true) {     
        shooter.cmdShoot();
    }

    Logger.recordOutput("ShooterCalcs/NewDist",           newDist);
    Logger.recordOutput("servoCmdPos",                    servoPos);

  }

  @Override
  public boolean isFinished(){
    return shooter.getShooterNoteState() == ShooterNoteState.NOTE_HAS_BEEN_SHOT;
  }


}
