package frc.robot.commands.DriveCmds.Trajectory.Paths;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.CatzConstants.DriveConstants;

public class Trajectories {
    //all the trajectories will be created here
    public static final Trajectory testTrajectoryStraight = generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),  //starting point
        List.of(
            new Translation2d(1.5, 0)                   //any point inbetween start and end so a curve can be created.
        ),
        new Pose2d(3, 0, Rotation2d.fromDegrees(0)),//ending point
        DriveConstants.MAX_SPEED, DriveConstants.MAX_SPEED
    );

    public static final Trajectory testTrajectoryCurve = generateTrajectory(
        new Pose2d(0,0,Rotation2d.fromDegrees(90)),

        List.of(
            new Translation2d(0.5,1.5)
        ),

        new Pose2d(2,2,Rotation2d.fromDegrees(0)),
        DriveConstants.MAX_SPEED, DriveConstants.MAX_SPEED
    );

    public static final Trajectory testTrajectoryCurveGoBack = generateTrajectory(
        new Pose2d(2,2,Rotation2d.fromDegrees(180)),

        List.of(
            new Translation2d(0.5,1.5)
        ),

        new Pose2d(0,0,Rotation2d.fromDegrees(-90)),
        DriveConstants.MAX_SPEED, DriveConstants.MAX_SPEED
    );

    public static final Trajectory testTrajectoryConnect1 = generateTrajectory(
        new Pose2d(0,0,Rotation2d.fromDegrees(0)),

        List.of(
            new Translation2d(1.5,0)
        ),

        new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        DriveConstants.MAX_SPEED, DriveConstants.MAX_SPEED
    );

    public static final Trajectory testTrajectoryConnect2 = generateTrajectory(
        new Pose2d(3, 0, Rotation2d.fromDegrees(0)),

        List.of(
            new Translation2d(3,1.5)
        ),

        new Pose2d(3,3,Rotation2d.fromDegrees(0)),
        DriveConstants.MAX_SPEED, DriveConstants.MAX_SPEED
    );

    public static final Trajectory testTrajectoryBellsCurve = generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),

        List.of(
            new Translation2d(1.5, 1.5)
        ),

        new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        DriveConstants.MAX_SPEED, DriveConstants.MAX_SPEED
    );

    // generates trajectory
    private static Trajectory generateTrajectory(Pose2d startPos, List<Translation2d> waypoints, Pose2d endPos, double maxVel, double maxAccel)
    {
        return TrajectoryGenerator.generateTrajectory(startPos, waypoints, endPos, new TrajectoryConfig(maxVel, maxAccel));
    }
}
