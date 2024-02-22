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
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.commands.DriveCmds.PPTrajectoryFollowingCmd;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;

public class CatzAutonomous {

    private static CatzAutonomous Instance;
    
    private SubsystemCatzDrivetrain m_driveTrain = SubsystemCatzDrivetrain.getInstance();

    public static LoggedDashboardChooser<CatzConstants.AllianceColor> chosenAllianceColor = new LoggedDashboardChooser<>("alliance selector");
    private static LoggedDashboardChooser<Command> pathChooser = new LoggedDashboardChooser<>("Chosen Autonomous Path");

    private AllianceColor allianceColor;

    public CatzAutonomous() {
        chosenAllianceColor.addDefaultOption("Blue Alliance", CatzConstants.AllianceColor.Blue);
        chosenAllianceColor.addOption       ("Red Alliance",  CatzConstants.AllianceColor.Red);

        pathChooser.addOption("Bulldozer Auto", bulldozerAuto());
        pathChooser.addOption("Speaker 4 Piece Wing", speaker4PieceWing());


        pathChooser.addOption("DriveTranslate Auto", driveTranslateAuto());
        pathChooser.addOption("ScoringC13", scoringC13());
        pathChooser.addOption("Curve", curveAuto());

    }

    //configured dashboard
    public Command getCommand() {
        return pathChooser.get(); 
    }

    //-------------------------------------------Auton Paths--------------------------------------------
    private Command bulldozerAuto() {
        return new SequentialCommandGroup(
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Bulldozer"))
        );
    }

    private Command speaker4PieceWing(){
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("S4PW1")),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PW1")),
            Commands.waitSeconds(0.5),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PW2")),
            Commands.waitSeconds(0.5),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PW3"))
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
    
    //below are test paths
    private Command driveTranslateAuto() {
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("DriveStraightFullTurn")),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("DriveStraightFullTurn")),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Right"))
        );
    }

    private Command curveAuto(){
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("Curve")),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Curve")),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Right Straight"))
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
                new Pose2d(2.7, 2.8, Rotation2d.fromDegrees(0)),
                new Pose2d(2.5, 2.3, Rotation2d.fromDegrees(0))
                    );

        //send path info to trajectory following command
        return new PPTrajectoryFollowingCmd(bezierPoints, 
                                            constraints, 
                                            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)));
    }

    //-------------------------------------------------MISC-------------------------------------------------------

    private Command setAutonStartPose(PathPlannerPath startPath){
    return Commands.runOnce(()->{
        PathPlannerPath path = startPath;
        if(CatzAutonomous.chosenAllianceColor.get() == CatzConstants.AllianceColor.Red) {
            path = startPath.flipPath();
        }

        m_driveTrain.resetPosition(path.getPreviewStartingHolonomicPose());
        allianceColor = chosenAllianceColor.get();

        if(allianceColor == CatzConstants.AllianceColor.Red) {
            m_driveTrain.flipGyro();
        }
    });
    }

    public AllianceColor getAllianceColor(){
        return allianceColor;
    }

    public static CatzAutonomous getInstance(){
        if(Instance == null){
            Instance = new CatzAutonomous();
        }
        return Instance;
    }
}