package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCmds.PPTrajectoryFollowingCmd;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;

public class CatzAutonomous {
    private SubsystemCatzDrivetrain m_driveTrain = SubsystemCatzDrivetrain.getInstance();

    private static LoggedDashboardChooser<DriverStation.Alliance> chosenAllianceColor = new LoggedDashboardChooser<>("alliance selector");
    private static LoggedDashboardChooser<Command> internalPathChooser = new LoggedDashboardChooser<>("Chosen Autonomous Path");

    public CatzAutonomous() {
        chosenAllianceColor.addDefaultOption("Blue Alliance", DriverStation.Alliance.Blue);
        chosenAllianceColor.addOption       ("Red Alliance",  DriverStation.Alliance.Red);

        internalPathChooser.addOption("Bulldozer Auto", bulldozerAuto());
        internalPathChooser.addOption("DriveTranslate Auto", driveTranslateAuto());
        internalPathChooser.addOption("ScoringC13", scoringC13());
        //internalPathChooser.addOption("Drive Straight", driveStraight());
    }

    //configured dashboard
    public Command getCommand() {
        m_driveTrain.resetForAutonomous();

        return internalPathChooser.get(); //for internal path choosing TBD should we use pathplanners or a coded version?
    }

    //-------------------------------------------Auton Paths--------------------------------------------
    private Command bulldozerAuto() {
        return new SequentialCommandGroup(
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Bulldozer"))
        );
    }

    private Command driveTranslateAuto() {
        return new SequentialCommandGroup(
            Commands.runOnce(()->m_driveTrain.resetPosition(new Pose2d(2,2,Rotation2d.fromDegrees(0)))),//TBD let's make reset position not hard coded
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("DriveStraightFullTurn")),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Right"))
        );
    }

    private Command scoringC13() {
        return new SequentialCommandGroup(
            Commands.runOnce(()->m_driveTrain.resetPosition(new Pose2d(1.27, 7.38, Rotation2d.fromDegrees(0)))),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_1")),
            Commands.waitSeconds(4),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_2")),
            Commands.waitSeconds(4),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_3")),
            Commands.waitSeconds(4),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_4")),
            Commands.waitSeconds(4),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_5")),
            Commands.waitSeconds(4),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_6"))
        );
    }

    //---------------------------------------------------------Trajectories/Swervepathing---------------------------------------------------------

    //Automatic pathfinding command
    public Command autoFindPathSource() {
        //Create a new pathplanner path on the fly
                // Create the constraints to use while pathfinding
            PathConstraints constraints = new PathConstraints(
            3.0, 4.0, 
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
                    );

        //send path info to trajectory following command
        return new PPTrajectoryFollowingCmd(bezierPoints, 
                                            constraints, 
                                            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)));
    }
}