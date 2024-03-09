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
import frc.robot.subsystems.intake.SubsystemCatzIntake.IntakeState;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.turret.SubsystemCatzTurret;
import frc.robot.subsystems.vision.SubsystemCatzVision;


public class ScoreSpeakerCmd extends InstantCommand {
  //subsystem declaration
  private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
  private SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  private SubsystemCatzShooter shooter = SubsystemCatzShooter.getInstance();
  private SubsystemCatzTurret turret = SubsystemCatzTurret.getInstance();
  private static SubsystemCatzDrivetrain drivetrain = SubsystemCatzDrivetrain.getInstance();

  //--------------------------------------------------------------
  // Interpolation tables
  //--------------------------------------------------------------
  /** Shooter angle look up table key: meters, values: pivot position */
  private static final InterpolatingDoubleTreeMap shooterPivotTable = new InterpolatingDoubleTreeMap();

  static { //TBD add values in through testing
    shooterPivotTable.put(1.0, 2.0);
  }

  /** angle to time look up table key: ty angle, values: time */
  private static final InterpolatingDoubleTreeMap timeTable = new InterpolatingDoubleTreeMap();
      // (ty-angle,time)
  static { //TBD add values in through testing
    timeTable.put(80.0, 2.0);
  }

  public static final double kAccelCompFactor = 0.100; // in units of seconds

  public ScoreSpeakerCmd() {
    addRequirements(turret, shooter, intake, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //start the flywheel
    shooter.startShooterFlywheel();
    intake.updateTargetPositionIntake(CatzMechanismConstants.POS_AMP_TRANSITION);
    elevator.updateTargetPositionElevator(CatzMechanismConstants.POS_STOW);
  }

  @Override 
  public void execute() {
    FieldRelativeSpeed robotVel = drivetrain.getFieldRelativeSpeed();
    FieldRelativeAccel robotAccel = drivetrain.getFieldRelativeAccel();

    Translation2d target;
    if(CatzAutonomous.chosenAllianceColor.get() == CatzConstants.AllianceColor.Red) {
        //translation of the blue alliance speaker
      target = new Translation2d(0.0, 5.55);
    } else {
      //translation of the blue alliance speaker
      target = new Translation2d(0.0, 10 + 5.55);
    }

    //take the distance to the speaker
    Translation2d robotToGoal = target.minus(drivetrain.getPose().getTranslation());

    //convert the distance to inches
    double dist = robotToGoal.getDistance(new Translation2d()) * 39.37;

    //get the time it takes for note to reach the speaker in seconds? TBD
    double shotTime = timeTable.get(dist);

    Translation2d movingGoalLocation = new Translation2d();

    //run 5 iterations to test the goal location accuraccy
    for(int i=0;i<5;i++){
      //collect new shooting targets modifed by robot acceleration and velocity
        double virtualGoalX = target.getX()
                - shotTime * (robotVel.vx + robotAccel.ax * kAccelCompFactor);
        double virtualGoalY = target.getY()
                - shotTime * (robotVel.vy + robotAccel.ay * kAccelCompFactor);

        //Logger.recordOutput("ShooterCalcs/Goal X", virtualGoalX);
        //Logger.recordOutput("ShooterCalcs/Goal Y", virtualGoalY);

        Translation2d testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

        Translation2d toTestGoal = testGoalLocation.minus(drivetrain.getPose().getTranslation());

        //take the new time to reach target
        double newShotTime = timeTable.get(toTestGoal.getDistance(new Translation2d()) * 39.37);

        //if the difference between the two is low, skip the iterations 
        if(Math.abs(newShotTime-shotTime) <= 0.010){
            i=4;
        }
      
        //create the new target for the shooter and shooter pivot
        if(i == 4){
            movingGoalLocation = testGoalLocation;
            //Logger.recordOutput("ShooterCalcs/NewShotTime", newShotTime);
        }
        else{ //continue attempting to close the gap
            shotTime = newShotTime;
        }
    }

    double newDist = movingGoalLocation.minus(drivetrain.getPose().getTranslation()).getDistance(new Translation2d()) * 39.37;

    if(intake.getIntakeState() == IntakeState.IN_POSITION &&
       elevator.getElevatorState() == ElevatorState.IN_POSITION) {
      //send the new target to the turret
      turret.aimAtGoal(movingGoalLocation, false);
    }

    //send new target to the shooter
    //shooter.updateShooterServo(shooterPivotTable.get(newDist + SmartDashboard.getNumber("SetHoodAdjust", 0)));
    shooter.updateShooterServo(shooterPivotTable.get(newDist));

    Logger.recordOutput("ShooterCalcs/Fixed Time", shotTime);
    Logger.recordOutput("ShooterCalcs/NewDist", newDist);
    Logger.recordOutput("ShooterCalcs/Calculated (in)", dist);


  }

}
