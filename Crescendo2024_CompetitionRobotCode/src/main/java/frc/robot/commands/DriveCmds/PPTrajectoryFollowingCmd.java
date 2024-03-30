package frc.robot.commands.DriveCmds;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.CatzAutonomous;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.CatzConstants.TrajectoryConstants;

import java.util.List;

import org.littletonrobotics.junction.Logger;

public class PPTrajectoryFollowingCmd extends Command {

    private final HolonomicDriveController hocontroller;
    private SubsystemCatzDrivetrain m_driveTrain = SubsystemCatzDrivetrain.getInstance();
    private PathPlannerTrajectory trajectory;
    
    private final Timer timer = new Timer();
    private final double TIMEOUT_RATIO = 5;
    private PathPlannerPath path;

    /**
     * @param drivetrain The coordinator between the gyro and the swerve modules.
     * @param trajectory          The trajectory to follow.
     */
    public PPTrajectoryFollowingCmd(PathPlannerPath newPath) {
        path = newPath;

        hocontroller = DriveConstants.holonomicDriveController;
        addRequirements(m_driveTrain);
    }

    //Auto Pathplanning trajectoreies
    public PPTrajectoryFollowingCmd(List<Translation2d> bezierPoints, PathConstraints constraints, GoalEndState endRobotState) {
        PathPlannerPath newPath = new PathPlannerPath(bezierPoints, constraints, endRobotState);
        path = newPath;

        hocontroller = DriveConstants.holonomicDriveController;

        addRequirements(m_driveTrain);
    }

    @Override
    public void initialize() {
        // Reset and begin timer
        timer.reset();
        timer.start();

        //flip auton path to mirrored red side if we choose red alliance
        if(CatzAutonomous.getInstance().getAllianceColor() == CatzConstants.AllianceColor.Red) {
            path = path.flipPath();
            // System.out.println("flip");
        }

        //path debug
        // for(int i=0; i<path.getAllPathPoints().size(); i++){
        //     System.out.println(path.getAllPathPoints().get(i).position);
        // }
       // Logger.recordOutput("Inital pose", path.getPreviewStartingHolonomicPose());
        
        //create pathplanner trajectory

        this.trajectory = new PathPlannerTrajectory(
                                path, 
                                DriveConstants.
                                    swerveDriveKinematics.
                                        toChassisSpeeds(m_driveTrain.getModuleStates()),
                                m_driveTrain.getRotation2d());
    }

    //private double prevSpeed = previousState.velocityMps;

    @Override
    public void execute() {
        double currentTime = this.timer.get();

        //getters from pathplanner and current robot pose
        PathPlannerTrajectory.State goal = trajectory.sample(currentTime);
        Rotation2d targetOrientation     = goal.targetHolonomicRotation;
        Pose2d currentPose               = m_driveTrain.getPose();

        trajectory.eve

        //Logger.recordOutput("PathPlanner Goal MPS", goal.velocityMps);
        
        /* 
        * Convert PP trajectory into a wpilib trajectory type 
        * Only takes in the current robot position 
        * Does not take acceleration to be used with the internal WPILIB trajectory library
        */
        Trajectory.State state = new Trajectory.State(currentTime, 
                                                      0,  //made the holonomic drive controller only rely on its current position, not its velocity because the target velocity is used as a ff
                                                      0, 
                                                      new Pose2d(goal.positionMeters, new Rotation2d()), 
                                                      0);

        //debug
        //System.out.println(goal.getTargetHolonomicPose());
        //Logger.recordOutput("Trajectory Goal MPS", state.velocityMetersPerSecond);
        //construct chassisspeeds
        ChassisSpeeds adjustedSpeeds = hocontroller.calculate(currentPose, state, targetOrientation);
        //Logger.recordOutput("Adjusted Speeds X", adjustedSpeeds.vxMetersPerSecond);
        //Logger.recordOutput("Adjusted Speeds Y", adjustedSpeeds.vyMetersPerSecond);
        //send to drivetrain
        m_driveTrain.driveRobotWithDescritizeDynamics(adjustedSpeeds);

        Logger.recordOutput("Desired Auto Pose", new Pose2d(state.poseMeters.getTranslation(), goal.targetHolonomicRotation));
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop(); // Stop timer
        m_driveTrain.stopDriving();
    }

    @Override
    public boolean isFinished() {
        // Finish command if the total time the path takes is over
        double currentPosX =        m_driveTrain.getPose().getX();
        double currentPosY =        m_driveTrain.getPose().getY();
        double currentRotation =    m_driveTrain.getPose().getRotation().getDegrees();

        double desiredPosX =        trajectory.getEndState().positionMeters.getX();
        double desiredPosY =        trajectory.getEndState().positionMeters.getY();
        double desiredRotation =    trajectory.getEndState().targetHolonomicRotation.getDegrees();

        double xError =        Math.abs(desiredPosX - currentPosX);
        double yError =        Math.abs(desiredPosY - currentPosY);
        double rotationError = Math.abs(desiredRotation - currentRotation);

        //System.out.println("X error " + xError);
        //System.out.println("Y error " + yError);
        //System.out.println("Angle error " + rotationError);
        boolean inPose = (xError < TrajectoryConstants.ALLOWABLE_POSE_ERROR && 
                yError < TrajectoryConstants.ALLOWABLE_POSE_ERROR && 
                rotationError < TrajectoryConstants.ALLOWABLE_ROTATION_ERROR) || 
                timer.hasElapsed(trajectory.getTotalTimeSeconds() * TIMEOUT_RATIO);
        return inPose;
    }

}