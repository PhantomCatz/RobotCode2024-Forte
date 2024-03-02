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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.commands.DriveCmds.PPTrajectoryFollowingCmd;
import frc.robot.commands.mechanismCmds.IntakeCmd;
import frc.robot.commands.mechanismCmds.ShootCmd;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;

public class CatzAutonomous {
    private static CatzAutonomous Instance;
    
    private SubsystemCatzDrivetrain m_driveTrain = SubsystemCatzDrivetrain.getInstance();

    public static LoggedDashboardChooser<CatzConstants.AllianceColor> chosenAllianceColor = new LoggedDashboardChooser<>("alliance selector");
    private static LoggedDashboardChooser<Command> pathChooser = new LoggedDashboardChooser<>("Chosen Autonomous Path");

    PathConstraints autoPathfindingConstraints = new PathConstraints(
        3.0, 4.0, 
        Units.degreesToRadians(540), Units.degreesToRadians(720));
    private AllianceColor allianceColor;

    public CatzAutonomous() {
        chosenAllianceColor.addDefaultOption("Blue Alliance", CatzConstants.AllianceColor.Blue);
        chosenAllianceColor.addOption       ("Red Alliance",  CatzConstants.AllianceColor.Red);

        pathChooser.addOption("Speaker 4 Piece Wing", speaker4PieceWing());
        pathChooser.addOption("Speaker 4 Piece CS Wing", speaker4PieceCSWing());

        pathChooser.addOption("1 Wing Bulldoze Under", WingBulldozeUnder());
        pathChooser.addOption("1 Wing Bulldoze Above", WingBulldozeAbove());

        pathChooser.addOption("ScoringC13", scoringC13());
        pathChooser.addOption("ScoringC53", scoringC53());
        pathChooser.addOption("Run and gun W1 C1-3", RNGC1W13());

        pathChooser.addOption("Hoard Lower Mid", HoardLowerMid());
        pathChooser.addOption("Bottom Mid Clear", BottomMidClear());

        pathChooser.addOption("Center Rush MId", CenterRushMid());

        pathChooser.addOption("DriveTranslate Auto", driveTranslateAuto());
        pathChooser.addOption("Curve", curveAuto());
    }

    //configured dashboard
    public Command getCommand() {
        return pathChooser.get(); 

    }

    //-------------------------------------------Auton Paths--------------------------------------------
<<<<<<< Updated upstream
    private Command driveStraightPickup(){
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("DriveStraightFullTurn")),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("DriveStraightFullTurn")),
            new MoveToNewPositionCmd(CatzConstants.CatzMechanismConstants.NOTE_POS_INTAKE_GROUND),
            new HandoffCmd()
        )
    }

    private Command bulldozerAuto() {
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("1WingBulldozeAbove-1")),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("1WingBulldozeAbove-1")),
            Commands.waitSeconds(0.5),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("1WingBulldozeAbove-2")),
            Commands.waitSeconds(0.5),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("1WingBulldozeAbove-3"))
        );

    }

=======
    //https://docs.google.com/presentation/d/19F_5L03n90t7GhtzQhD4mYNEMkdFsUGoDSb4tT9HqNI/edit#slide=id.g268da342b19_1_0
>>>>>>> Stashed changes
    private Command speaker4PieceWing(){
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("S4PW1")),
            new ShootCmd(true, true),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PW1")),
            new IntakeCmd(), new ShootCmd(true, true),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PW2")),
            new IntakeCmd(), new ShootCmd(true, true),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PW3")),
            new IntakeCmd(), new ShootCmd(true, true)
        );
    }

    private Command speaker4PieceCSWing(){
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("S4PCSW1")),
            new ShootCmd(true, true),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PCSW1")),
            new IntakeCmd(), new ShootCmd(true, true),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PCSW2")),
            new IntakeCmd(), new ShootCmd(true, true),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PCSW3")),
            new IntakeCmd(), new ShootCmd(true, true),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PCSW4")),
            new IntakeCmd()
        );
    }

    private Command scoringC13() {
    return new SequentialCommandGroup(
        setAutonStartPose(PathPlannerPath.fromPathFile("Scoring_C1-3_1")),
        new ShootCmd(true, true),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_1")),
        new IntakeCmd(),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_2")),
        new ShootCmd(true, true),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_3")),
        new IntakeCmd(),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_4")),
        new ShootCmd(true, true),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_5")),
        new IntakeCmd(),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_6")),
        new ShootCmd(true, true)
        );
    }

    private Command scoringC53() {
    return new SequentialCommandGroup(
        setAutonStartPose(PathPlannerPath.fromPathFile("Scoring_C5-3_1")),
        new ShootCmd(true, true),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C5-3_1")),
        new IntakeCmd(),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C5-3_2")),
        new ShootCmd(true, true),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C5-3_3")),
        new IntakeCmd(),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C5-3_4")),
        new ShootCmd(true, true),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C5-3_5")),
        new IntakeCmd(),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C5-3_6")),
        new ShootCmd(true, true)
        );
    }

    private Command RNGC1W13() {
    return new SequentialCommandGroup(
        setAutonStartPose(PathPlannerPath.fromPathFile("Scoring_C1-3_1")),
        new ShootCmd(true, true),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(1), // dummy
                new IntakeCmd(), new ShootCmd(true, true)
            ),
            new SequentialCommandGroup(
                new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_1")),
                new IntakeCmd(),
                new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_2")),
                new ShootCmd(true, true),
                new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_3")),
                new IntakeCmd(),
                new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_4")),
                new ShootCmd(true, true),
                new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_5")),
                new IntakeCmd(),
                new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_6")),
                new ShootCmd(true, true)
            )
        )
        );
    }

    private Command CenterRushMid() {
    return new SequentialCommandGroup(
        setAutonStartPose(PathPlannerPath.fromPathFile("CenterRushMid-1")),
        new ShootCmd(true, true),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("CenterRushMid-2")),
        new IntakeCmd(),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("CenterRushMid-3")),
        new ShootCmd(true, true),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C5-3_3")),
        new IntakeCmd(), new ShootCmd(true, true)
        );
    }

    private Command BottomMidClear() {
    return new SequentialCommandGroup(
        setAutonStartPose(PathPlannerPath.fromPathFile("BottomMidClear-1")),
        new ShootCmd(true, true),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("BottomMidClear-1")),
        new IntakeCmd(), new ShootCmd(true, true),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("BottomMidClear-2")),
        new IntakeCmd(),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("BottomMidClear-3")),
        new ShootCmd(true, false),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("BottomMidClear-4")),
        new IntakeCmd(),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("BottomMidClear-5")),
        new ShootCmd(true, false),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("BottomMidClear-6")),
        new IntakeCmd(),
        );
    }

    private Command MidClearHoardAmp() {
    return new SequentialCommandGroup(
        setAutonStartPose(PathPlannerPath.fromPathFile("MCHA1")),
        new ShootCmd(true, true),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("MCHA1")),
        new IntakeCmd(),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("MCHA2")),
        new ShootCmd(false, false),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("MCHA3")),
        new IntakeCmd(),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("MCHA4")),
        new ShootCmd(true, false),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("MCHA5")),
        new IntakeCmd(),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("MCHA6")),
        new ShootCmd(true, false)
        );
    }

    private Command HoardLowerMid(){
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("HoardLowMid-1")),
            new ShootCmd(false, true),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("HoardLowMid-1")),
            new IntakeCmd(),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("HoardLowMid-2")),
            new ShootCmd(false, true),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("HoardLowMid-3")),
            new IntakeCmd(),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("HoardLowMid-4")),
            new ShootCmd(false, true),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("HoardLowMid-5")),
            new IntakeCmd(),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("HoardLowMid-6")),
            new ShootCmd(false, true)
        );
    }

    private Command WingBulldozeUnder() {
    return new SequentialCommandGroup(
        setAutonStartPose(PathPlannerPath.fromPathFile("1WingBulldozeUnder-1")),
        new ShootCmd(true, true),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("1WingBulldozeUnder-1")),
        new IntakeCmd(), new ShootCmd(true, true),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("1WingBulldozeUnder-2")),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("1WingBulldozeUnder-3"))
        );
    }

    private Command WingBulldozeAbove() {
    return new SequentialCommandGroup(
        setAutonStartPose(PathPlannerPath.fromPathFile("1WingBulldozeAbove-1")),
        new ShootCmd(true, true),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("1WingBulldozeAbove-1")),
        new IntakeCmd(), new ShootCmd(true, true),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("1WingBulldozeAbove-2")),
        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("1WingBulldozeAbove-3"))
        );
    }
    
    //below are test paths
    private Command driveTranslateAuto() {
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("DriveStraightFullTurn")),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("DriveStraightFullTurn"))
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
    public Command autoFindPathSource() {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(2.7, 7.5, Rotation2d.fromDegrees(0)),
                new Pose2d(2.5, 7.8, Rotation2d.fromDegrees(0))
                    );

        //send path info to trajectory following command
        return new PPTrajectoryFollowingCmd(bezierPoints, 
                                            autoPathfindingConstraints, 
                                            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)));
    }

    public Command autoFindPathAmp() {

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(1.85, 7.5, Rotation2d.fromDegrees(90)),
                new Pose2d(1.85, 7.8, Rotation2d.fromDegrees(90))
                    );

        //send path info to trajectory following command
        return new PPTrajectoryFollowingCmd(bezierPoints, 
                                            autoPathfindingConstraints, 
                                            new GoalEndState(0.0, Rotation2d.fromDegrees(90)));
    }

    public Command autoFindPathSpeakerAW() {

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(2.7, 2.8, Rotation2d.fromDegrees(0)),
                new Pose2d(2.5, 2.3, Rotation2d.fromDegrees(0))
                    );

        //send path info to trajectory following command
        return new PPTrajectoryFollowingCmd(bezierPoints, 
                                            autoPathfindingConstraints, 
                                            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)));
    }

    public Command autoFindPathSpeakerUT() {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(2.7, 2.8, Rotation2d.fromDegrees(0)),
                new Pose2d(2.5, 2.3, Rotation2d.fromDegrees(0))
                    );

        //send path info to trajectory following command
        return new PPTrajectoryFollowingCmd(bezierPoints, 
                                            autoPathfindingConstraints, 
                                            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)));
    }

    //Automatic pathfinding command
    public Command autoFindPathSpeakerCS() {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(2.7, 2.8, Rotation2d.fromDegrees(0)),
                new Pose2d(2.5, 2.3, Rotation2d.fromDegrees(0))
                    );

        return new PPTrajectoryFollowingCmd(bezierPoints, 
                                            autoPathfindingConstraints, 
                                            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)));
    }

    //Automatic pathfinding command
    public Command autoFindPathSpeakerLOT() {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(180)),
                new Pose2d(1.50, 0.69, Rotation2d.fromDegrees(235))
                    );

        //send path info to trajectory following command
        return new PPTrajectoryFollowingCmd(bezierPoints, 
                                            autoPathfindingConstraints, 
                                            new GoalEndState(0.0, Rotation2d.fromDegrees(235)));
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