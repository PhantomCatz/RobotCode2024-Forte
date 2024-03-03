package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.logging.Logger;

import java.awt.geom.Point2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.CatzConstants.ShooterConstants;
import frc.robot.Utils.FieldRelativeAccel;
import frc.robot.Utils.FieldRelativeSpeed;
import frc.robot.Utils.LinearInterpolationTable;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
// import frc.robot.subsystems.turret.SubsystemCatzTurret;
// import frc.robot.subsystems.vision.SubsystemCatzVision;


public class AutoAlignCmd extends InstantCommand {
  private final Timer m_timer = new Timer();

  private static final Point2D[] kHoodPoints = new Point2D.Double[] {
    // (ty-angle,distance)
    new Point2D.Double(35, 0.0),
    new Point2D.Double(55, 0.0),
    new Point2D.Double(80, 7.5), //
    new Point2D.Double(105, 16.5), //
    new Point2D.Double(130, 22.0), //
    new Point2D.Double(155, 25.5), //
    new Point2D.Double(165, 25.5), //
    new Point2D.Double(180, 27.5), //
    new Point2D.Double(205, 29.0), //
    new Point2D.Double(230, 33.0), //
    new Point2D.Double(255, 33.0), //
    new Point2D.Double(270, 33.5), //
    new Point2D.Double(280, 36.1)
    };

    public static final LinearInterpolationTable kHoodTable = new LinearInterpolationTable(kHoodPoints);

    private static final Point2D[] kRPMPoints = new Point2D.Double[] {
        // (ty-angle,distance)
        new Point2D.Double(35, 1500+10),
        new Point2D.Double(55, 1860+10),
        new Point2D.Double(80, 2000+10), //
        new Point2D.Double(105, 2100+10), //
        new Point2D.Double(130, 2170+20), //
        new Point2D.Double(155, 2245+30), //
        new Point2D.Double(165, 2380), //
        new Point2D.Double(180, 2465+30), //
        new Point2D.Double(205, 2670+30), //
        new Point2D.Double(230, 2840+35), //
        new Point2D.Double(255, 2980+40), //
        new Point2D.Double(270, 3300), //
        new Point2D.Double(280, 3350+60)

    };

    public static final LinearInterpolationTable kRPMTable = new LinearInterpolationTable(kRPMPoints);

    private static final Point2D[] kShotTimes = new Point2D.Double[] {
        // (ty-angle,time)
        new Point2D.Double(80, 0.78),
        new Point2D.Double(130, 0.80),
        new Point2D.Double(190, 0.81),
        new Point2D.Double(240, 0.82),
        new Point2D.Double(280, 0.83)
    };
    private static Point2D[] m_shotTimes = 
        new Point2D.Double[]{
            //(dist,time)
            new Point2D.Double(105,0.82), 
            new Point2D.Double(135,0.82), 
            new Point2D.Double(165,0.85),//
            new Point2D.Double(195,0.85),
            new Point2D.Double(250,1.05),
            //
        };

    public static final LinearInterpolationTable kTimeTable = new LinearInterpolationTable(kShotTimes);
    private static LinearInterpolationTable m_timeTable = new LinearInterpolationTable(m_shotTimes);

  private static LinearInterpolationTable m_hoodTable = kHoodTable;
  private static LinearInterpolationTable m_rpmTable = kRPMTable;
  private final SubsystemCatzDrivetrain drivetrain = SubsystemCatzDrivetrain.getInstance();
  // private final SubsystemCatzTurret     turret     = SubsystemCatzTurret.getInstance();
  private DriverStation.Alliance alliance = null;

  private static final double SPEAKER_COORD_MTRS_Y = Units.inchesToMeters(219.277);
  private static final double FIELD_LENGTH_MTRS = Units.inchesToMeters(651.223);
  public static final double kAccelCompFactor = 0.100; // in units of seconds
  private double hozDeg;
  private double driveTrainOffset;


  public AutoAlignCmd() {
    // addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  @Override 
  public void execute() {
    double currentTime = m_timer.get();        

    SmartDashboard.putNumber("Current Time", currentTime);

    SmartDashboard.putBoolean("Shooter Running", true);

    FieldRelativeSpeed robotVel = drivetrain.getFieldRelativeSpeed();
    FieldRelativeAccel robotAccel = drivetrain.getFieldRelativeAccel();

    Translation2d target = new Translation2d(5,5);

    Translation2d robotToGoal = target.minus(drivetrain.getPose().getTranslation());
    double dist = robotToGoal.getDistance(new Translation2d())*39.37;

    SmartDashboard.putNumber("Calculated (in)", dist);

    double fixedShotTime = m_timeTable.getOutput(dist);

    double virtualGoalX = target.getX()-fixedShotTime*(robotVel.vx+robotAccel.ax*kAccelCompFactor);
    double virtualGoalY = target.getY()-fixedShotTime*(robotVel.vy+robotAccel.ay*kAccelCompFactor);

    SmartDashboard.putNumber("Goal X", virtualGoalX);
    SmartDashboard.putNumber("Goal Y", virtualGoalY);

    Translation2d movingGoalLocation = new Translation2d(virtualGoalX,virtualGoalY);

    Translation2d toMovingGoal = movingGoalLocation.minus(drivetrain.getPose().getTranslation());

    double newDist = toMovingGoal.getDistance(new Translation2d())*39.37;

    // turret.aimAtGoal( movingGoalLocation, false);
    
        if(SmartDashboard.getBoolean("Adjust Shot?", false)){
            //m_shooter.run(kRPMTable.getOutput(newDist)+SmartDashboard.getNumber("SetShotAdjust", 0));
           // m_hood.run(m_hoodTable.getOutput(newDist)+SmartDashboard.getNumber("SetHoodAdjust", 0));
        }
        else{
           // m_shooter.run(m_rpmTable.getOutput(newDist));
            //.run(m_hoodTable.getOutput(newDist));
            
        }
  }

}
