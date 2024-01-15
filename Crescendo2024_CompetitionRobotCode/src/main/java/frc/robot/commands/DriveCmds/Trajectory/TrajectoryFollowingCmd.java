package frc.robot.commands.DriveCmds.Trajectory;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;
import frc.robot.CatzConstants.DriveConstants;

public class TrajectoryFollowingCmd extends Command{
    // private final double TIMEOUT_RATIO = 3;
    // private final double END_POS_ERROR = 0.05;
    // private final double END_ROT_ERROR = 2.5; //degrees

    // private final Timer timer = new Timer();
    // private final HolonomicDriveController controller;
    // private SubsystemCatzDrivetrain m_driveTrain = SubsystemCatzDrivetrain.getInstance();

    // private final Trajectory trajectory;
    // private final Rotation2d endOrientation;
    // private Rotation2d initOrientation;

    // /**
    //  * @param trajectory The trajectory to follow
    //  * @param endOrientation The goal orientation for the robot to be in while in the middle of the trajectory. Takes a Pose2d parameter so that the heading may change based on external factors. 
    //  */
    // public TrajectoryFollowingCmd(Trajectory trajectory, Rotation2d endOrientation)
    // {
    //     this.trajectory = trajectory;
    //     this.endOrientation = endOrientation; 

    //     controller = DriveConstants.holonomicDriveController; 
    //     addRequirements(m_driveTrain);
    // }

    // // reset and start timer
    // @Override
    // public void initialize() {
    //     timer.reset();
    //     timer.start();
    //     initOrientation = m_driveTrain.getRotation2d();
    // }

    // // calculates if trajectory is finished
    // @Override
    // public boolean isFinished() {
    //     double maxTime = trajectory.getTotalTimeSeconds();
    //     Pose2d currentPosition = m_driveTrain.getPose();
    //     Pose2d dist = trajectory.sample(maxTime).poseMeters.relativeTo(currentPosition);

    //     double angleError = Math.abs(endOrientation.getDegrees() - currentPosition.getRotation().getDegrees());
    //     double posError = Math.hypot(dist.getX(), dist.getY());

    //     return 
    //         timer.get() > maxTime * TIMEOUT_RATIO || 
    //         (
    //             angleError <= END_ROT_ERROR &&
    //             posError <= END_POS_ERROR
    //         );
    // }

    // // sets swerve modules to their target states so that the robot will follow the trajectory
    // @Override
    // public void execute() {
    //     double currentTime = timer.get();
    //     Trajectory.State goal = trajectory.sample(currentTime);
    //     Rotation2d targetOrientation = initOrientation.interpolate(endOrientation, currentTime / trajectory.getTotalTimeSeconds());
    //     Pose2d currentPosition = m_driveTrain.getPose();
        
    //     ChassisSpeeds adjustedSpeed = controller.calculate(currentPosition, goal, targetOrientation);
    //     SwerveModuleState[] targetModuleStates = DriveConstants.swerveDriveKinematics.toSwerveModuleStates(adjustedSpeed);
    //     m_driveTrain.setModuleStates(targetModuleStates);

    //     //data logging
    //     Logger.recordOutput("Current Position", currentPosition);
    //     Logger.recordOutput("Target Position", goal.poseMeters);
    //     Logger.recordOutput("Adjusted VelX", adjustedSpeed.vxMetersPerSecond);
    //     Logger.recordOutput("Adjusted VelX", adjustedSpeed.vyMetersPerSecond);
    //     Logger.recordOutput("Adjusted VelW", adjustedSpeed.omegaRadiansPerSecond);

    //     // for debugging
    // }

    // // stop all robot motion
    // @Override
    // public void end(boolean interrupted) {
    //     timer.stop();
    //     m_driveTrain.stopDriving();
    //     System.out.println("trajectory done");
    // }
}