package frc.robot;

import java.nio.file.Path;
import java.sql.Driver;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;

public class CatzAutonomous {
    private SubsystemCatzDrivetrain m_driveTrain = SubsystemCatzDrivetrain.getInstance();

    private static LoggedDashboardChooser<DriverStation.Alliance> chosenAllianceColor = new LoggedDashboardChooser<>("alliance selector");
    private SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    private static LoggedDashboardChooser<Command> internalPathChooser = new LoggedDashboardChooser<>("Chosen Autonomous Path");

    public CatzAutonomous() {
        chosenAllianceColor.addDefaultOption("Blue Alliance", DriverStation.Alliance.Blue);
        chosenAllianceColor.addOption       ("Red Alliance",  DriverStation.Alliance.Red);

        internalPathChooser.addOption("Bulldozer Auto", bulldozerAuto());
        internalPathChooser.addOption("DriveTranslate Auto", driveTranslateAuto());
        internalPathChooser.addOption("ScoringC13", scoringC13());


        SmartDashboard.putData("Auto Chooser", autoChooser);;
    }

    //configured dashboard
    public Command getCommand() {
        m_driveTrain.resetForAutonomous();

        //return autoChooser.getSelected(); // nanny library on top
        return internalPathChooser.get(); //for internal path choosing TBD should we use pathplanners or a coded version?
    }



    //-------------------------------------------Auton Paths--------------------------------------------
    private Command bulldozerAuto() {
        return new SequentialCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Bulldozer"))
            );
    }

    private Command driveTranslateAuto() {
        return new SequentialCommandGroup(
            Commands.runOnce(()->m_driveTrain.resetPosition(new Pose2d(2,2,Rotation2d.fromDegrees(0)))),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveStraightFullTurn")),
            Commands.waitSeconds(2),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Right")));
    }

    private Command scoringC13() {
        return new SequentialCommandGroup(
            Commands.runOnce(()->m_driveTrain.resetPosition(new Pose2d(1.27, 7.38, Rotation2d.fromDegrees(0)))),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(("Scoring_C1-3_1"))),
            Commands.waitSeconds(4),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(("Scoring_C1-3_2"))),
            Commands.waitSeconds(4),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(("Scoring_C1-3_3"))),
            Commands.waitSeconds(4),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(("Scoring_C1-3_4"))),
            Commands.waitSeconds(4),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(("Scoring_C1-3_5"))),
            Commands.waitSeconds(4),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(("Scoring_C1-3_6"))));
        
    }
    //---------------------------------------------------------Trajectories/Swervepathing---------------------------------------------------------

    //Automatic pathfinding command
    public Command autoFindPathOne() {
                // Create the constraints to use while pathfinding
            PathConstraints constraints = new PathConstraints(
            3.0, 4.0, 
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        // See the "Follow a single path" example for more info on what gets passed here
        return AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("driveStraightFullTurn"),
                constraints,
                3.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
    }
}