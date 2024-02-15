package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCmds.PPTrajectoryFollowingCmd;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;

public class CatzAutonomous {
    
    private SubsystemCatzDrivetrain m_driveTrain = SubsystemCatzDrivetrain.getInstance();

    public static LoggedDashboardChooser<CatzConstants.AllianceColor> chosenAllianceColor = new LoggedDashboardChooser<>("alliance selector");
    private static LoggedDashboardChooser<Command> internalPathChooser = new LoggedDashboardChooser<>("Chosen Autonomous Path");

    public CatzAutonomous() {
        chosenAllianceColor.addDefaultOption("Blue Alliance", CatzConstants.AllianceColor.Blue);
        chosenAllianceColor.addOption       ("Red Alliance",  CatzConstants.AllianceColor.Red);

        internalPathChooser.addOption("Bulldozer Auto", bulldozerAuto());
        internalPathChooser.addOption("Speaker 4 Piece Wing", speaker4PieceWing());


        internalPathChooser.addOption("DriveTranslate Auto", driveTranslateAuto());
        internalPathChooser.addOption("ScoringC13", scoringC13());
        internalPathChooser.addOption("Curve", curveAuto());

        /*PathPlannerPath path = PathPlannerPath.fromPathFile("Bulldozer");
        for(int i=0; i<path.getAllPathPoints().size(); i++){
            System.out.println(path.getAllPathPoints().get(i).position);
        }
        System.out.println("\n");
        //path = path.flipPath();
        path.flipPath();
        for(int i=0; i<path.getAllPathPoints().size(); i++){
            System.out.println(path.getAllPathPoints().get(i).position);
        }*/

        //internalPathChooser.addOption("Drive Straight", driveStraight());
    }

    //configured dashboard
    public Command getCommand() {
        return internalPathChooser.get(); 
    }

    //-------------------------------------------Auton Paths--------------------------------------------
    private Command bulldozerAuto() {
        return new SequentialCommandGroup(
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Bulldozer"))
        );
    }

    private Command speaker4PieceWing(){
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("S4PW1"),true),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PW1")),
            Commands.waitSeconds(0.5),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PW2")),
            Commands.waitSeconds(0.5),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PW3"))
        );
    }
    
    //below are test paths

    private Command driveTranslateAuto() {
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("DriveStraightFullTurn"),false),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("DriveStraightFullTurn")),
            Commands.waitSeconds(1),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Right"))
        );
    }

    private Command curveAuto(){
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("Curve"),false),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Curve")),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Right Straight"))
        );
    }

    private Command scoringC13() {
        return new SequentialCommandGroup(
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
    
    private Command setAutonStartPose(PathPlannerPath startPath, boolean isFlipped){
        return Commands.runOnce(()->{
            PathPlannerPath path = startPath;
            if(CatzAutonomous.chosenAllianceColor.get() == CatzConstants.AllianceColor.Red) {
                path = startPath.flipPath();
            }

            m_driveTrain.resetPosition(path.getPreviewStartingHolonomicPose(), isFlipped);
        });
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