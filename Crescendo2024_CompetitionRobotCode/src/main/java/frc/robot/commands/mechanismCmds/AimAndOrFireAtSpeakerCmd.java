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
import frc.robot.Utils.CatzMechanismPosition;
import frc.robot.Utils.FieldRelativeAccel;
import frc.robot.Utils.FieldRelativeSpeed;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.elevator.SubsystemCatzElevator.ElevatorState;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.intake.SubsystemCatzIntake.IntakeControlState;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.turret.SubsystemCatzTurret;
import frc.robot.subsystems.vision.SubsystemCatzVision;


public class AimAndOrFireAtSpeakerCmd extends Command {
  //subsystem declaration
  private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
  private SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  private SubsystemCatzShooter shooter = SubsystemCatzShooter.getInstance();
  private SubsystemCatzTurret turret = SubsystemCatzTurret.getInstance();
  private SubsystemCatzDrivetrain drivetrain = SubsystemCatzDrivetrain.getInstance();

  //--------------------------------------------------------------
  // Interpolation tables
  //--------------------------------------------------------------
  /** Shooter angle look up table key: meters, values: pivot position */
  private static final InterpolatingDoubleTreeMap shooterPivotTable = new InterpolatingDoubleTreeMap();

  static { //TBD add values in through testing
    shooterPivotTable.put(1.0, 2.0);
    shooterPivotTable.put(1.5, 3.5);
    shooterPivotTable.put(2.0, 5.0);
    shooterPivotTable.put(2.5, 7.5);
    shooterPivotTable.put(3.0, 10.0);
    shooterPivotTable.put(3.5, 12.5);
    shooterPivotTable.put(4.0, 15.0);
    shooterPivotTable.put(4.5, 17.5);
    shooterPivotTable.put(5.0, 20.0);
    shooterPivotTable.put(5.5, 22.5);
    shooterPivotTable.put(6.0, 25.0);
    shooterPivotTable.put(6.5, 28.0);
    shooterPivotTable.put(7.0, 31.0);
    shooterPivotTable.put(7.5, 34.0);
    shooterPivotTable.put(8.0, 37.0);
    shooterPivotTable.put(9.0, 41.0);
    shooterPivotTable.put(10.0, 45.0);
  }

  //time table look up for calculating how long it takes to get note into speaker
  /** angle to time look up table key: ty angle, values: time */
  private static final InterpolatingDoubleTreeMap timeTable = new InterpolatingDoubleTreeMap();
      // (ty-angle,time)
  static { //TBD add values in through testing
    timeTable.put(80.0, 2.0);

  }

  public static final double k_ACCEL_COMP_FACTOR = 0.100; // in units of seconds

  //aiming variables
  private FieldRelativeSpeed m_robotVel;
  private FieldRelativeAccel m_robotAccel;

  private Translation2d m_targetXY;
  private Translation2d robotToGoalXY;
  private Translation2d movingGoalLocation;
  private Translation2d testGoalLocation;
  private Translation2d toTestGoal;

  //number variables
  private double distanceToSpeakerInches;
  private double shotTime;
  private double newShotTime;

  private double m_virtualGoalX;
  private double m_virtualGoalY;

  private Supplier<Boolean> m_bSupplier;

  public AimAndOrFireAtSpeakerCmd(Supplier<Boolean> bSupplier) {
    m_bSupplier = bSupplier;
    addRequirements(turret, shooter, intake, elevator);
  }

  public AimAndOrFireAtSpeakerCmd() {
    addRequirements(turret, shooter, intake, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //start the flywheel
    shooter.startShooterFlywheel();
    intake.updateTargetPositionIntake(CatzMechanismConstants.POS_AMP_TRANSITION);
    elevator.updateTargetPositionElevator(CatzMechanismConstants.POS_STOW);

    if(CatzAutonomous.chosenAllianceColor.get() == CatzConstants.AllianceColor.Red) {
        //translation of the blue alliance speaker
      m_targetXY = new Translation2d(0.0, 5.55);
    } else {
      //translation of the blue alliance speaker
      m_targetXY = new Translation2d(16, 5.55);
    }

  }

  @Override 
  public void execute() {
    if(m_bSupplier != null &&
       m_bSupplier.get() == true) {
        shooter.cmdShoot();
    }


    m_robotVel = drivetrain.getFieldRelativeSpeed();
    m_robotAccel = drivetrain.getFieldRelativeAccel();

    //take the distance to the speaker
    robotToGoalXY = m_targetXY.minus(drivetrain.getPose().getTranslation());

    //convert the distance to inches
    distanceToSpeakerInches = robotToGoalXY.getDistance(new Translation2d()) * 39.37;

    //get the time it takes for note to reach the speaker in seconds? TBD
    shotTime = timeTable.get(distanceToSpeakerInches);

    movingGoalLocation = new Translation2d();

    //run 5 iterations to test the goal location accuraccy
    for(int i=0;i<5;i++){
      //collect new shooting targets modifed by robot acceleration and velocity
        m_virtualGoalX = m_targetXY.getX()
                - shotTime * (m_robotVel.vx + m_robotAccel.ax * k_ACCEL_COMP_FACTOR);
        m_virtualGoalY = m_targetXY.getY()
                - shotTime * (m_robotVel.vy + m_robotAccel.ay * k_ACCEL_COMP_FACTOR);

        testGoalLocation = new Translation2d(m_virtualGoalX, m_virtualGoalY);

        toTestGoal = testGoalLocation.minus(drivetrain.getPose().getTranslation());

        //take the new time to reach target
        newShotTime = timeTable.get(toTestGoal.getDistance(new Translation2d()) * 39.37);

        //if the difference between the two is low, skip the iterations 
        if(Math.abs(newShotTime-shotTime) <= 0.010){
            i=4;
        }
      
        //create the new target for the shooter and shooter pivot
        if(i == 4){
            movingGoalLocation = testGoalLocation;
        }
        else{ //continue attempting to close the gap
            shotTime = newShotTime;
        }
    }

    double newDist = movingGoalLocation.minus(drivetrain.getPose().getTranslation()).getDistance(new Translation2d()) * 39.37;

    if(intake.getIntakeState() == IntakeControlState.IN_POSITION &&
       elevator.getElevatorState() == ElevatorState.IN_POSITION) {
      //send the new target to the turret
      turret.aimAtGoal(movingGoalLocation, false);
    }

    //send new target to the shooter
    shooter.updateShooterServo(shooterPivotTable.get(newDist));

    Logger.recordOutput("ShooterCalcs/Fixed Time", shotTime);
    Logger.recordOutput("ShooterCalcs/NewDist", newDist);
    Logger.recordOutput("ShooterCalcs/Calculated (in)", distanceToSpeakerInches);
    Logger.recordOutput("ShooterCalcs/Goal X", m_virtualGoalX);
    Logger.recordOutput("ShooterCalcs/Goal Y", m_virtualGoalY);
    Logger.recordOutput("ShooterCalcs/NewShotTime", newShotTime);

  }

}
