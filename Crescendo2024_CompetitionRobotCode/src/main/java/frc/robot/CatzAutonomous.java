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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.CatzMechanismConstants;
import frc.robot.commands.DriveCmds.PPTrajectoryFollowingCmd;
import frc.robot.commands.mechanismCmds.HoardShotCmd;
import frc.robot.commands.mechanismCmds.HomeToSpeakerCmd;
import frc.robot.commands.mechanismCmds.MoveToPresetHandoffCmd;
import frc.robot.commands.utilCmds.WaitUntilSecondsLeft;
import frc.robot.commands.mechanismCmds.MoveToPreset;
import frc.robot.CatzConstants.NoteDestination;
import frc.robot.CatzConstants.NoteSource;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.subsystems.elevator.SubsystemCatzElevator;
import frc.robot.subsystems.intake.SubsystemCatzIntake;
import frc.robot.subsystems.shooter.SubsystemCatzShooter;
import frc.robot.subsystems.turret.SubsystemCatzTurret;

public class CatzAutonomous {
    private static CatzAutonomous instance;
    
    //subsystem declaration
    private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
    private SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
    private SubsystemCatzShooter shooter = SubsystemCatzShooter.getInstance();
    private SubsystemCatzTurret turret = SubsystemCatzTurret.getInstance();
    private SubsystemCatzDrivetrain drivetrain = SubsystemCatzDrivetrain.getInstance();

    private static AllianceColor allianceColor = AllianceColor.Blue;
    private static LoggedDashboardChooser<AllianceColor> chosenAllianceColor = new LoggedDashboardChooser<>("alliance selector");
    private static LoggedDashboardChooser<Command> autoPathChooser = new LoggedDashboardChooser<>("Chosen Autonomous Path");

    PathConstraints autoPathfindingConstraints = new PathConstraints(
        4.8, 4.0, 
        Units.degreesToRadians(540), Units.degreesToRadians(720));
    
    private CatzAutonomous() {

    //Alliance Color Selection in ShuffleBoard
        chosenAllianceColor.addDefaultOption("Blue Alliance", AllianceColor.Blue);
        chosenAllianceColor.addOption       ("Red Alliance",  AllianceColor.Red);

    /*-------------------------------------------------------------------------------------------------------------------
    *
    *   AUTON Priority LIST (It's in order - So Don't Mess it Up)
    *
    *-------------------------------------------------------------------------------------------------------------------*/

        autoPathChooser.addOption("Score and Wait US_W1", US_W1());
        autoPathChooser.addOption("Score and Wait LS_W3", LS_W3()); 
        autoPathChooser.addOption("Score and Wait CS_W2", CS_W2());
        
        autoPathChooser.addOption("ScoringC13", US_C13());
        autoPathChooser.addOption("ScoringC35", US_C35());
        autoPathChooser.addOption("Hoard C1-2", US_C12_Hoard());
        autoPathChooser.addOption("Hoard C4-5", LS_C45_Hoard());
        
        //4 Note Autons
        autoPathChooser.addOption("Scoring US W1-3", US_W13());
        autoPathChooser.addOption("Scoring LS W1-3", LS_W13());
        autoPathChooser.addOption("Speaker 4 Piece Wing", speaker4PieceWing());

        //Subwoofer 4 Note Autons
        autoPathChooser.addOption("Subwoofer US W1-3", sub_US_W13());
        autoPathChooser.addOption("Subwoofer LS W1-3", sub_LS_W13());
        autoPathChooser.addOption("Subwoofer CS W1-3", sub_speaker4PieceWing());

        autoPathChooser.addOption("", getCommand());

        //Semi Illegal Paths 
        autoPathChooser.addOption("1 Wing Bulldoze Under", WingBulldozeUnder());
        autoPathChooser.addOption("1 Wing Bulldoze Above", WingBulldozeAbove());

    //-----------------------------------------------------------------------------------------------------------------------


        // pathChooser.addOption("mid", mid());
        // pathChooser.addOption("bot", bot());
        // pathChooser.addOption("top", top());
        // pathChooser.addOption("Speaker 4 Piece CS Wing", speaker4PieceCSWing());
        // pathChooser.addOption("TestDrive US W1-3", US_W13_TEST_DRIVE());
        // pathChooser.addOption("Run and gun W1 C1-3", RNGC1W13());
        // pathChooser.addOption("Hoard Lower Mid", HoardLowerMid());
        // pathChooser.addOption("Bottom Mid Clear", BottomMidClear());
        // pathChooser.addOption("DriveStraightRight", driveTranslateAutoRight());
        // pathChooser.addOption("DriveStraightMid", driveTranslateAutoMid());
        // pathChooser.addOption("DriveStraightLeft", driveTranslateAutoLeft());
        // pathChooser.addOption("Curve", curveAuto());
        autoPathChooser.addOption("Test", test());
    }

    //configured dashboard
    public Command getCommand() {
        return CS_W1111();
    }

    public static CatzAutonomous getInstance(){
        if(instance == null){
            instance = new CatzAutonomous();
        }
        return instance;
    }

    //--------------------------------------------------------------------------------------------
    //  
    //      4 NOTE Autonomous Paths
    //  
    //--------------------------------------------------------------------------------------------

    private final double FOUR_PIECE_AUTON_INTAKE_TIMEOUT = 4.0;
    /*
     * Robot Starting Position: Upper Speaker
     * Seems to be redundant with 4 piece auton, only advantage is having code for different starting positions depending on
     * alliance's capabilities
     */

    private PathPlannerPath US_W1_3_1 = PathPlannerPath.fromPathFile("US_W1-3_1");
    private PathPlannerPath US_W1_3_2 = PathPlannerPath.fromPathFile("ver2 US_W1-3_2");
    private PathPlannerPath US_W1_3_3 = PathPlannerPath.fromPathFile("ver2 US_W1-3_3");

    private Command US_W13() {
        return new SequentialCommandGroup(
            setAutonStartPose(US_W1_3_1),
            shooter.cmdShooterRamp(),
            // shooter.cmdSetKeepShooterOn(true),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(US_W1_3_1),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(US_W1_3_2),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(US_W1_3_3),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            // shooter.cmdSetKeepShooterOn(false),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd()
        );
    }

    /*
     * Modified version of US_W1_3 to return to subwoofer to shoot with fixed shooter pivot angle
     */

    private PathPlannerPath sub_US_W1_3_2 = PathPlannerPath.fromPathFile("sub US_W1-3_2");
    private PathPlannerPath sub_US_W1_3_3 = PathPlannerPath.fromPathFile("sub US_W1-3_3");
    private PathPlannerPath sub_US_W1_3_4 = PathPlannerPath.fromPathFile("sub US_W1-3_4");
    private PathPlannerPath sub_US_W1_3_5 = PathPlannerPath.fromPathFile("sub US_W1-3_5");
    private PathPlannerPath sub_US_W1_3_6 = PathPlannerPath.fromPathFile("sub US_W1-3_6");

    private Command sub_US_W13() {
        return new SequentialCommandGroup(
            setAutonStartPose(US_W1_3_1),
            shooter.cmdShooterRamp(),
            // shooter.cmdSetKeepShooterOn(true),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(US_W1_3_1),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            new PPTrajectoryFollowingCmd(sub_US_W1_3_2),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(sub_US_W1_3_3),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            new PPTrajectoryFollowingCmd(sub_US_W1_3_4),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(sub_US_W1_3_5),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            new PPTrajectoryFollowingCmd(sub_US_W1_3_6),
            // shooter.cmdSetKeepShooterOn(false),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd()
        );
    }

    private Command US_W13_TEST_DRIVE() {
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("US_W1-3_1")),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("US_W1-3_1"))),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("US_W1-3_2"))),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("US_W1-3_3")))
        );
    }

    /*
     * Robot Starting Position: Upper Speaker
     * Seems to be redundant with 4 piece auton, only advantage is having code for different starting positions depending on
     * alliance's capabilities
     */
    private PathPlannerPath LS_W1_3_1 = PathPlannerPath.fromPathFile("LS_W1-3_1");
    private PathPlannerPath LS_W1_3_2 = PathPlannerPath.fromPathFile("LS_W1-3_2");
    private PathPlannerPath LS_W1_3_3 = PathPlannerPath.fromPathFile("LS_W1-3_3");

    private Command LS_W13() {
        return new SequentialCommandGroup(
            setAutonStartPose(LS_W1_3_1),
            shooter.cmdShooterRamp(),
            // shooter.cmdSetKeepShooterOn(true),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(LS_W1_3_1),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(LS_W1_3_2),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(LS_W1_3_3),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            // shooter.cmdSetKeepShooterOn(false),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd()
        );
    }

    /*
     * Modified version of LS_W13 to accomodate fixed shooter pivot angle
     */
    private PathPlannerPath sub_LS_W1_3_2 = PathPlannerPath.fromPathFile("sub LS_W1-3_2");
    private PathPlannerPath sub_LS_W1_3_3 = PathPlannerPath.fromPathFile("sub LS_W1-3_3");
    private PathPlannerPath sub_LS_W1_3_4 = PathPlannerPath.fromPathFile("sub LS_W1-3_4");
    private PathPlannerPath sub_LS_W1_3_5 = PathPlannerPath.fromPathFile("sub LS_W1-3_5");
    private PathPlannerPath sub_LS_W1_3_6 = PathPlannerPath.fromPathFile("sub LS_W1-3_6");

    private Command sub_LS_W13() {
        return new SequentialCommandGroup(
            setAutonStartPose(LS_W1_3_1),
            shooter.cmdShooterRamp(),
            // shooter.cmdSetKeepShooterOn(true),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(LS_W1_3_1),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            new PPTrajectoryFollowingCmd(sub_LS_W1_3_2),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(sub_LS_W1_3_3),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            new PPTrajectoryFollowingCmd(sub_LS_W1_3_4),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(sub_LS_W1_3_5),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            new PPTrajectoryFollowingCmd(sub_LS_W1_3_6),
            // shooter.cmdSetKeepShooterOn(false),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd()
        );
    }

    //https://docs.google.com/presentation/d/19F_5L03n90t7GhtzQhD4mYNEMkdFsUGoDSb4tT9HqNI/edit#slide=id.g268da342b19_1_0

    private PathPlannerPath S4PW1 = PathPlannerPath.fromPathFile("S4PW1");
    private PathPlannerPath S4PW2 = PathPlannerPath.fromPathFile("S4PW2");
    private PathPlannerPath S4PW3 = PathPlannerPath.fromPathFile("S4PW3");

    private Command speaker4PieceWing(){
        return new SequentialCommandGroup(
            setAutonStartPose(S4PW1),
            shooter.cmdShooterRamp(),
            // shooter.cmdSetKeepShooterOn(true),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(S4PW1),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(S4PW2),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(S4PW3),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            // shooter.cmdSetKeepShooterOn(false),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd()
        );
    }

    private PathPlannerPath sub_S4PW1 = PathPlannerPath.fromPathFile("sub S4PW1");
    private PathPlannerPath sub_S4PW2 = PathPlannerPath.fromPathFile("sub S4PW2");
    private PathPlannerPath sub_S4PW3 = PathPlannerPath.fromPathFile("sub S4PW3");
    private PathPlannerPath sub_S4PW4 = PathPlannerPath.fromPathFile("sub S4PW4");
    private PathPlannerPath sub_S4PW5 = PathPlannerPath.fromPathFile("sub S4PW5");
    private PathPlannerPath sub_S4PW6 = PathPlannerPath.fromPathFile("sub S4PW6");

    private Command sub_speaker4PieceWing(){
        return new SequentialCommandGroup(
            setAutonStartPose(sub_S4PW1),
            shooter.cmdShooterRamp(),
            // shooter.cmdSetKeepShooterOn(true),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(sub_S4PW1),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            new PPTrajectoryFollowingCmd(sub_S4PW2),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(sub_S4PW3),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            new PPTrajectoryFollowingCmd(sub_S4PW4),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),

            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(sub_S4PW5),
                                     new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(FOUR_PIECE_AUTON_INTAKE_TIMEOUT)),
            new PPTrajectoryFollowingCmd(sub_S4PW6),
            // shooter.cmdSetKeepShooterOn(false),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd()
        );
    }

    //--------------------------------------------------------------------------------------------
    //  
    //      Priority Autonomous Paths
    //  
    //--------------------------------------------------------------------------------------------

    /*
     * Robot Starting Position: Lower Speaker
     * Sequence: Shoots preload into speaker
     *           Picks up note in front it (W3)
     *           Waits until last 5 seconds and then shoots into speaker
     */
    private PathPlannerPath LS_W3_1 = PathPlannerPath.fromPathFile("LS_W3-1");

    private Command LS_W3() {
        return new SequentialCommandGroup(
            setAutonStartPose(LS_W3_1),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),
            new WaitUntilSecondsLeft(5.0),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(LS_W3_1),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND)),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd()
        );
    }

    /*
     * Robot Starting Position: Upper Speaker
     * Sequence: Shoots preload into speaker
     *           Picks up note in front it (W1)
     *           Waits until last 5 seconds and then shoots into speaker
     */
    private PathPlannerPath US_W1_1 = PathPlannerPath.fromPathFile("US_W1-1");

    private Command US_W1() {
        return new SequentialCommandGroup(
            setAutonStartPose(US_W1_1),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),
            new WaitUntilSecondsLeft(5.0),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(US_W1_1),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND)
                                    ),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd()
        );
    }
    
    /*
     * Robot Starting Position: Center Speaker
     * Sequence: Shoots preload into speaker
     *           Picks up note in front of it (W2)
     *           Waits until last 5 seconds and then shoots into speaker
     */

    private PathPlannerPath CS_W2_1 = PathPlannerPath.fromPathFile("CS_W2-1");

    private Command CS_W2() {
        return new SequentialCommandGroup(
            setAutonStartPose(CS_W2_1),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),
            new WaitUntilSecondsLeft(5.0),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(CS_W2_1),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND)
                                    ),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd()
        );
    }

    /*
     * Robot Starting Position: Lower Speaker
     * Sequence: Shoots preload into speaker
     *           Drives to center line to pick up note at the very bottom in Pathplanner (C5)
     *           Goes near stage to shoot into speaker
     *           Drives to center line to pick up note above previous note (C4)
     *           Goes under stage to shoot into speaker
     *           Drives to center line to pick up note above previous note (C3)
     *           Goes to same spot under stage to shoot into speaker
     */

    private PathPlannerPath C35_1 = PathPlannerPath.fromPathFile("Scoring_C3-5_1");
    private PathPlannerPath C35_2 = PathPlannerPath.fromPathFile("Scoring_C3-5_2");
    private PathPlannerPath C35_3 = PathPlannerPath.fromPathFile("Scoring_C3-5_3");
    private PathPlannerPath C35_4 = PathPlannerPath.fromPathFile("Scoring_C3-5_4");
    private PathPlannerPath C35_5 = PathPlannerPath.fromPathFile("Scoring_C3-5_5");
    private PathPlannerPath C35_6 = PathPlannerPath.fromPathFile("Scoring_C3-5_6");

    private Command US_C35() {
        return new SequentialCommandGroup(
            setAutonStartPose(C35_1),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(C35_1),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND)).withTimeout(5.0),
            shooter.cmdShooterRamp(),
            new PPTrajectoryFollowingCmd(C35_2),
            new HomeToSpeakerCmd(),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(C35_3),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(4.0)),
            shooter.cmdShooterRamp(),
            new PPTrajectoryFollowingCmd(C35_4),
            new HomeToSpeakerCmd(),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(C35_5),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(4.0)),
            shooter.cmdShooterRamp(),
            new PPTrajectoryFollowingCmd(C35_6),
            new HomeToSpeakerCmd()
        );
    }

    /*
     * Robot Starting Position: Upper Speaker
     * Sequence: Shoots preload into speaker
     *           Drives to center line to pick up note at the very top in PathPlanner (C1)
     *           Goes near stage to shoot into speaker
     *           Drives to center line to pick up note below previous note (C2)
     *           Goes near stage to shoot into speaker
     *           Drives to center line to pick up note below previous note (C3)
     *           Goes under stage to shoot into speaker
     */

    private PathPlannerPath Scoring_C1_3_1 = PathPlannerPath.fromPathFile("Scoring_C1-3_1");
    private PathPlannerPath Scoring_C1_3_2 = PathPlannerPath.fromPathFile("Scoring_C1-3_2");
    private PathPlannerPath Scoring_C1_3_3 = PathPlannerPath.fromPathFile("Scoring_C1-3_3");
    private PathPlannerPath Scoring_C1_3_4 = PathPlannerPath.fromPathFile("Scoring_C1-3_4");
    private PathPlannerPath Scoring_C1_3_5 = PathPlannerPath.fromPathFile("Scoring_C1-3_5");
    private PathPlannerPath Scoring_C1_3_6 = PathPlannerPath.fromPathFile("Scoring_C1-3_6");
    
    private Command US_C13() {
        return new SequentialCommandGroup(
            setAutonStartPose(Scoring_C1_3_1),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(Scoring_C1_3_1),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(5.0)),
            new PPTrajectoryFollowingCmd(Scoring_C1_3_2),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(Scoring_C1_3_3),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(4.0)),
            new PPTrajectoryFollowingCmd(Scoring_C1_3_4),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(Scoring_C1_3_5),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(4.0)),
            new PPTrajectoryFollowingCmd(Scoring_C1_3_6),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd()
        );
    }

    /*
     * Robot Starting Position: Upper Speaker
     * Sequence: Shoots preload into speaker
     *           Drives to center line to pick up note at the very top in PathPlanner (C1)
     *           Goes near stage to hoard by shooting near speaker
     *           Drives to center line to pick up note below previous note (C2)
     *           Goes near stage to hoard by shooting near speaker
     */
    private final PathPlannerPath Hoard_C1_2_1 = PathPlannerPath.fromPathFile("Hoard_C1-2_1");
    private final PathPlannerPath Hoard_C1_2_2 = PathPlannerPath.fromPathFile("Hoard_C1-2_2");
    private final PathPlannerPath Hoard_C1_2_3 = PathPlannerPath.fromPathFile("Hoard_C1-2_3");
    private final PathPlannerPath Hoard_C1_2_4 = PathPlannerPath.fromPathFile("Hoard_C1-2_4");
    
    private Command US_C12_Hoard() {
        return new SequentialCommandGroup( 
            setAutonStartPose(Hoard_C1_2_1),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(Hoard_C1_2_1),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(5.0)),
            new PPTrajectoryFollowingCmd(Hoard_C1_2_2),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(Hoard_C1_2_3),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(4.0)),
            new PPTrajectoryFollowingCmd(Hoard_C1_2_4),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd()
        );
    }


    private Command CS_W1111(){
        return new SequentialCommandGroup(
            setAutonStartPose(CS_W2_1),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd()
        );
    }
    /*
     * Robot Starting Position: Lower Speaker
     * Sequence: Shoots preload into speaker
     *           Drives to center line to pick up note at the very bottom in Pathplanner (C5)
     *           Goes near stage to hoard by shooting near speaker
     *           Drives to center line to pick up note above previous note (C4)
     *           Goes under stage to hoard by shooting near speaker
     */
    private final PathPlannerPath Hoard_C4_5_1 = PathPlannerPath.fromPathFile("Hoard_C4-5_1");
    private final PathPlannerPath Hoard_C4_5_2 = PathPlannerPath.fromPathFile("Hoard_C4-5_2");
    private final PathPlannerPath Hoard_C4_5_3 = PathPlannerPath.fromPathFile("Hoard_C4-5_3");
    private final PathPlannerPath Hoard_C4_5_4 = PathPlannerPath.fromPathFile("Hoard_C4-5_4");

    private Command LS_C45_Hoard() {
        return new SequentialCommandGroup(
            setAutonStartPose(Hoard_C4_5_1),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(Hoard_C4_5_1),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(5.0)),
            new PPTrajectoryFollowingCmd(Hoard_C4_5_2),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(Hoard_C4_5_3),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND).withTimeout(4.0)),
            new PPTrajectoryFollowingCmd(Hoard_C4_5_4),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd()
        );
    }
        
    /*
        * PathPlanner does not show past the center line, so this bulldozing path cannot be done effectively
        * The robot will follow the center line in a straight line instead of a squiggly line
    */
    private Command WingBulldozeUnder() {
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("1WingBulldozeUnder-1")),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("1WingBulldozeUnder-1")),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND)),
            shooter.cmdShooterRamp(),
            new HomeToSpeakerCmd(),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("1WingBulldozeUnder-2")),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("1WingBulldozeUnder-3"))
        );
    }
    
    /*
        * PathPlanner does not show past the center line, so this bulldozing path cannot be done effectively
        * The robot will follow the center line in a straight line instead of a squiggly line
    */
    private Command WingBulldozeAbove() {
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("1WingBulldozeAbove-1")),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("1WingBulldozeAbove-1")),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND)),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("1WingBulldozeAbove-2")),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("1WingBulldozeAbove-3"))
        );
    }


    //--------------------------------------------------------------------------------------------
    //  
    //      Autonomous Paths
    //  
    //--------------------------------------------------------------------------------------------
    
    private Command mid(){
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("DriveStraightMid")),
            shooter.cmdShooterRamp().withTimeout(1.5),
            Commands.waitSeconds(1.0),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("DriveStraightMid"))
            );
            // new AimAndOrFireAtSpeakerCmd(),
            // shooter.cmdShooterRamp());
    }

    private Command bot(){
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("DriveStraightBot")),
            shooter.cmdShooterRamp().withTimeout(1.5),
            Commands.waitSeconds(1.0),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("DriveStraightBot"))
        );
    }

    private Command top(){
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("DriveStraightTop")),
            shooter.cmdShooterRamp().withTimeout(1.5),
            Commands.waitSeconds(1.0),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("DriveStraightTop"))
        );
    }

    private Command speaker4PieceCSWing(){
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("S4PCSW1")),
            // new AimAndOrFireAtSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                     new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PCSW1"))),
            new HomeToSpeakerCmd().withTimeout(3.0),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PCSW2"))),
            // new AimAndOrFireAtSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PCSW3"))),
            // new AimAndOrFireAtSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("S4PCSW4"))),
            // new AimAndOrFireAtSpeakerCmd(),
            shooter.cmdShooterRamp()
        );
    }

    private Command RNGC1W13() {
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("Scoring_C1-3_1")),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_1"))),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_2")),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_3"))),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_4")),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_5"))),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C1-3_6")),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp()
        );
    }

    // private Command CenterRushMid() {
    // return new SequentialCommandGroup(
    //     setAutonStartPose(PathPlannerPath.fromPathFile("CenterRushMid-1")),
    //     new AimAndOrFireAtSpeakerCmd(),
    //     shooter.cmdShooterRamp(),
    //     new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
    //                                  new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("CenterRushMid-2"))),
    //     new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("CenterRushMid-3")),
    //     new AimAndOrFireAtSpeakerCmd(),
    //     shooter.cmdShooterRamp(),
    //     new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
    //                                  new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Scoring_C3-5_3"))),
    //     new AimAndOrFireAtSpeakerCmd(),
    //     shooter.cmdShooterRamp()
    //     );
    // }

    private Command BottomMidClear() {
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("BottomMidClear-1")),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("BottomMidClear-1"))),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("BottomMidClear-2"))),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("BottomMidClear-3")),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("BottomMidClear-4"))),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("BottomMidClear-5")),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("BottomMidClear-6")))
        );
    }

    private Command MidClearHoardAmp() {
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("MCHA1")),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("MCHA1"))),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("MCHA2")),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("MCHA3"))),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("MCHA4")),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("MCHA5"))),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("MCHA6")),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp()
        );
    }

    private Command HoardLowerMid(){
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("HoardLowMid-1")),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("HoardLowMid-1"))),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("HoardLowMid-2")),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("HoardLowMid-3"))),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("HoardLowMid-4")),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp(),
            new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND),
                                        new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("HoardLowMid-5"))),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("HoardLowMid-6")),
            new HomeToSpeakerCmd(),
            shooter.cmdShooterRamp()
        );
    }
    
    //below are test paths
    private Command driveTranslateAutoRight() {
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("DriveStraightRight")),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("DriveStraightRight"))
        );
    }

    private Command driveTranslateAutoMid(){
        return new SequentialCommandGroup(
          setAutonStartPose(PathPlannerPath.fromPathFile("DriveStraightMid")),
          new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("DriveStraightMid"))

        );
    }

    private Command driveTranslateAutoLeft(){
        return new SequentialCommandGroup(
          setAutonStartPose(PathPlannerPath.fromPathFile("DriveStraightLeft")),
          new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("DriveStraightLeft"))

        );
    }

    private Command curveAuto(){
        return new SequentialCommandGroup(
            setAutonStartPose(PathPlannerPath.fromPathFile("Curve")),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Curve")),
            new PPTrajectoryFollowingCmd(PathPlannerPath.fromPathFile("Right Straight"))
        );
    }

       // private Command test(){
    //     return new SequentialCommandGroup(
    //         // shooter.cmdShooterRamp(),
    //         new ParallelCommandGroup(new PPTrajectoryFollowingCmd(test),
    //                                  new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND)),
    //         new HomeToSpeakerCmd()
    //     );
    // }
    private PathPlannerPath test = PathPlannerPath.fromPathFile("Test");

    private Command test(){
        return new SequentialCommandGroup(
            setAutonStartPose(test),
            new ParallelCommandGroup(new PPTrajectoryFollowingCmd(test),
                                        new MoveToPresetHandoffCmd(NoteDestination.SPEAKER, NoteSource.INTAKE_GROUND))
        );
    }
    //--------------------------------------------------------------------------------------------
    //  
    //      Auto Trjaectories for Telop
    //  
    //--------------------------------------------------------------------------------------------
    public Command autoFindClimbFar() {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(5.90, 4.15, Rotation2d.fromDegrees(180.0)),
        new Pose2d(5.81, 4.15, Rotation2d.fromDegrees(180.0))
            );

        //send path info to trajectory following command in a chained autocoring path
        return new PPTrajectoryFollowingCmd(bezierPoints,
                                            autoPathfindingConstraints,
                                            new GoalEndState(0.0, Rotation2d.fromDegrees(180.0))); 
    }

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

    public Command autoScoreAmp() {
        
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(1.85, 7.5, Rotation2d.fromDegrees(90)),
                new Pose2d(1.85, 7.8, Rotation2d.fromDegrees(90))
                    );

        //send path info to trajectory following command in a chained autocoring path
        return new SequentialCommandGroup(new ParallelCommandGroup(new MoveToPresetHandoffCmd(NoteDestination.AMP, NoteSource.FROM_SHOOTER)
                                                                                    .onlyWhile(()->intake.getIntakeLoadBeamBreakBroken() == false)), //transfer note to intake if applicable
                                                                    new PPTrajectoryFollowingCmd(bezierPoints,                                //start auto trajectory
                                                                                                        autoPathfindingConstraints, 
                                                                                                            new GoalEndState(0.0, Rotation2d.fromDegrees(90))),
                                          new MoveToPreset(CatzMechanismConstants.SCORING_AMP_PRESET));                    //move to amp scoring position
    }

    public Command autoHoardFromSource() {

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(1.85, 7.5, Rotation2d.fromDegrees(90)),
                new Pose2d(1.85, 7.8, Rotation2d.fromDegrees(90))
                    );

        //send path info to trajectory following command in a chained autocoring path
        return new SequentialCommandGroup(new ParallelCommandGroup(new PPTrajectoryFollowingCmd(bezierPoints,                                //start auto trajectory
                                                                                                autoPathfindingConstraints, 
                                                                                                new GoalEndState(0.0, Rotation2d.fromDegrees(90))),
                                                                    new MoveToPresetHandoffCmd(NoteDestination.AMP, NoteSource.FROM_SHOOTER)
                                                                                                            .onlyWhile(()->intake.getIntakeLoadBeamBreakBroken() == false)), //transfer note to intake if applicable
                                          new MoveToPreset(CatzMechanismConstants.INTAKE_HOARD_PRESET),                                                                 //move to hoard preset
                                          new HomeToSpeakerCmd(),
                                          shooter.cmdShooterRamp());                    
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


    //--------------------------------------------------------------------------------------------
    //  
    //      MISC()
    //  
    //--------------------------------------------------------------------------------------------

    private Command setAutonStartPose(PathPlannerPath startPath){
        return Commands.runOnce(()->{
            PathPlannerPath path = startPath;
            if(getAllianceColor() == CatzConstants.AllianceColor.Red) {
                path = startPath.flipPath();
            }

            drivetrain.resetPosition(path.getPreviewStartingHolonomicPose());
        });
    }

    public AllianceColor getAllianceColor(){
        return allianceColor;
    }

    public void chooseAllianceColor() {
        allianceColor = AllianceColor.Blue;
    }

}