package frc.robot.subsystems.drivetrain;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzAutonomous;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.Utils.FieldRelativeAccel;
import frc.robot.Utils.FieldRelativeSpeed;
import frc.robot.Utils.LocalADStarAK;
// import frc.robot.subsystems.vision.SubsystemCatzVision;;
import frc.robot.subsystems.vision.SubsystemCatzVision;

// Drive train subsystem for swerve drive implementation
public class SubsystemCatzDrivetrain extends SubsystemBase {

    // Singleton instance of the CatzDriveTrainSubsystem
    private static SubsystemCatzDrivetrain instance = new SubsystemCatzDrivetrain();

    // Gyro input/output interface
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();


    // Array of swerve modules representing each wheel in the drive train
    private CatzSwerveModule[] m_swerveModules = new CatzSwerveModule[4];

    // Swerve drive pose estimator for tracking robot pose
    private static SwerveDrivePoseEstimator m_poseEstimator;

    private final SubsystemCatzVision vision = SubsystemCatzVision.getInstance();

    // Swerve modules representing each corner of the robot
    public final CatzSwerveModule LT_FRNT_MODULE;
    public final CatzSwerveModule LT_BACK_MODULE;
    public final CatzSwerveModule RT_FRNT_MODULE;
    public final CatzSwerveModule RT_BACK_MODULE;

    // boolean for determining whether to use vision estimates in pose estimation
    private boolean isVisionEnabled = true;

    private FieldRelativeSpeed m_fieldRelVel = new FieldRelativeSpeed();
    private FieldRelativeSpeed m_lastFieldRelVel = new FieldRelativeSpeed();
    private FieldRelativeAccel m_fieldRelAccel = new FieldRelativeAccel();

    private final double AUTON_SPEED_SLOWDOWN_FACTOR = 0.1;
    private boolean isAutonSlowedDown = false;

    private double xyStdDev;

    // Private constructor for the singleton instance
    private SubsystemCatzDrivetrain() {
        
        // Determine gyro input/output based on the robot mode
        switch (CatzConstants.currentMode) {
            case REAL:
                gyroIO = new GyroIONavX();
                break;
            case REPLAY:
                gyroIO = new GyroIONavX() {};
                break;
            default:
                gyroIO = null;
                break;
        }

        // Create swerve modules for each corner of the robot
        LT_FRNT_MODULE = new CatzSwerveModule(DriveConstants.LT_FRNT_DRIVE_ID, DriveConstants.LT_FRNT_STEER_ID,
                DriveConstants.LT_FRNT_ENC_PORT, DriveConstants.LT_FRNT_OFFSET, 0,0);

        LT_BACK_MODULE = new CatzSwerveModule(DriveConstants.LT_BACK_DRIVE_ID, DriveConstants.LT_BACK_STEER_ID,
                DriveConstants.LT_BACK_ENC_PORT, DriveConstants.LT_BACK_OFFSET, 1, 0);

        RT_BACK_MODULE = new CatzSwerveModule(DriveConstants.RT_BACK_DRIVE_ID, DriveConstants.RT_BACK_STEER_ID,
                DriveConstants.RT_BACK_ENC_PORT, DriveConstants.RT_BACK_OFFSET, 2, 0);

        RT_FRNT_MODULE = new CatzSwerveModule(DriveConstants.RT_FRNT_DRIVE_ID, DriveConstants.RT_FRNT_STEER_ID,
                DriveConstants.RT_FRNT_ENC_PORT, DriveConstants.RT_FRNT_OFFSET, 3, 0);

        // Assign swerve modules to the array for easier access
        m_swerveModules[0] = LT_FRNT_MODULE;
        m_swerveModules[1] = LT_BACK_MODULE;
        m_swerveModules[2] = RT_BACK_MODULE;
        m_swerveModules[3] = RT_FRNT_MODULE;

        // Initialize the swerve drive pose estimator
        m_poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.swerveDriveKinematics,
            Rotation2d.fromDegrees(getGyroAngle()), 
            getModulePositions(), 
            new Pose2d(1.5, 5.55, Rotation2d.fromDegrees(0.0)), 
            VecBuilder.fill(1, 1, 0.7),  //odometry standard devs
            VecBuilder.fill(5, 5, 99999.0) //vision pose estimators standard dev are increase x, y, rotatinal radians values to trust vision less           
        ); 
        
        //Configure logging trajectories to advantage kit
        Pathfinding.setPathfinder(new LocalADStarAK());
        
        //DEBUG
        PathPlannerLogging.setLogActivePathCallback(
            (activepath)->{
                Logger.recordOutput("Obometry/Trajectory", activepath.toArray(new Pose2d[activepath.size()]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose)-> {
                Logger.recordOutput("Obometry/TrajectorySetpoint", targetPose);
            });

        gyroIO.resetNavXIO(0);  //TBD if red alliance how does the gryo get reset
        
    }

    // Get the singleton instance of the CatzDriveTrainSubsystem
    public static SubsystemCatzDrivetrain getInstance() {
        return instance;
    }

    @Override
    public void periodic() {
        // Update inputs (sensors/encoders) for code logic and advantage kit
        for (CatzSwerveModule module : m_swerveModules) {
            module.periodic();
        }
        // Update gyro inputs and log them
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/gyroinputs ", gyroInputs); 

        m_fieldRelVel = new FieldRelativeSpeed(DriveConstants.swerveDriveKinematics.toChassisSpeeds(getModuleStates()), 
                                               Rotation2d.fromDegrees(getGyroAngle()));

        
        //------------------------------------------------------------------------------------------------
        // Odometry pose updating
        //------------------------------------------------------------------------------------------------
        //
        m_poseEstimator.update(getRotation2d(), getModulePositions());    
        
        //------------------------------------------------------------------------------------------------
        // Vison pose updating
        //------------------------------------------------------------------------------------------------
        var visionOdometry = vision.getVisionOdometry();   
        for (int i = 0; i < visionOdometry.size(); i++) {
            if(visionOdometry.get(i).getName().equals("limelight-ramen")){
                continue;
            } 
            //pose estimators standard dev are increase x, y, rotatinal radians values to trust vision less   
            xyStdDev = 0;

            if(visionOdometry.get(i).getNumOfTagsVisible() >= 2){

                //vision is trusted more with more tags visible
                xyStdDev = 3; 
            }else if(visionOdometry.get(i).getAvgArea() >= 0.15){

                //vision is trusted more with tags that are closer to the target which inherhently take more of the frame
                xyStdDev = 5; 
            }else if(visionOdometry.get(i).getAvgArea() >= 0.12){ 
                //if vision takes up less than 12% of the frame
                xyStdDev = 10; 
            }else if(visionOdometry.get(i).getAvgArea() <= 0.05){

                //Do not trust vision inputs
                xyStdDev = 100; 
            }

            if(DriverStation.isAutonomous()){
                xyStdDev *= 2.5;
            }

            m_poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStdDev, xyStdDev,99999999.0)
            ); //gyro can be purely trusted for pose calculations so always trust it more than vision
            
            if(visionOdometry.get(i).hasTarget() && DriverStation.isTeleop()){ //-999.0 indicates that limelight had bad data or no target
                m_poseEstimator.addVisionMeasurement(
                    new Pose2d(visionOdometry.get(i).getPose().getTranslation(),getRotation2d()), //only use vison for x,y pose, because gyro is already accurate enough
                    visionOdometry.get(i).getTimestamp()
                );
            }

            //DEBUG
            Logger.recordOutput("Obm/ViPose "+ visionOdometry.get(i).getName(), visionOdometry.get(i).getPose());
        }


        //------------------------------------------------------------------------------------------------
        // Logging
        //------------------------------------------------------------------------------------------------
        //Long Term
        Logger.recordOutput("Obm/EstPose", getPose());

        //DEBUG
        // Logger.recordOutput("Obometry/LimelightPose Soba" , vision.getVisionOdometry().get(1).getPose()); 
        // Logger.recordOutput("Obometry/LimelightPose Udon" , vision.getVisionOdometry().get(2).getPose()); 
        SmartDashboard.putNumber("gyroAngle", getGyroAngle());

    }   //end of drivetrain periodic

    public void driveRobotWithDiscretizeKinematics(ChassisSpeeds chassisSpeeds) {

        //correct dynamics with wpilib internal "2nd order kinematics"
        ChassisSpeeds descreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        // Convert chassis speeds to individual module states and set module states
        SwerveModuleState[] moduleStates = DriveConstants.swerveDriveKinematics.toSwerveModuleStates(descreteSpeeds);
        setModuleStates(moduleStates);
    }

    // Set individual module states to each of the swerve modules
    private void setModuleStates(SwerveModuleState[] desiredStates) {

        // Scale down wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_DESATURATION);

        //optimize wheel angles
        SwerveModuleState[] optimizedDesiredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {  

            // The module returns the optimized state, useful for logging
            optimizedDesiredStates[i] = m_swerveModules[i].optimizeWheelAngles(desiredStates[i]);
        }

        // Set module states to each of the swerve modules
        LT_FRNT_MODULE.setDesiredState(optimizedDesiredStates[0]);
        LT_BACK_MODULE.setDesiredState(optimizedDesiredStates[1]);
        RT_BACK_MODULE.setDesiredState(optimizedDesiredStates[2]);
        RT_FRNT_MODULE.setDesiredState(optimizedDesiredStates[3]);

       
    }

    //--------------------------------------------------DEBUG PURPOSES LOGS-------------------------------------------------
    public void debugLogsDriveSubSys(){
        // Logger.recordOutput("Drive/unoptimized module states", desiredStates);
        // Logger.recordOutput("Drive/optimized module states", optimizedDesiredStates);
    }

    //--------------------------------------------------DriveTrain MISC methods-------------------------------------------------
    public void printAverageWheelMagEncValues(){
        System.out.println("LF: " + m_swerveModules[0].getAverageRawMagEnc());
        System.out.println("LB: " + m_swerveModules[1].getAverageRawMagEnc());
        System.out.println("RB: " + m_swerveModules[2].getAverageRawMagEnc());
        System.out.println("RF: " + m_swerveModules[3].getAverageRawMagEnc());

    }
    
    // Set brake mode for all swerve modules
    public void setBrakeMode() {
        for (CatzSwerveModule module : m_swerveModules) {
            module.setBrakeMode();
        }
    }

    // Set coast mode for all swerve modules
    public void setCoastMode() {
        for (CatzSwerveModule module : m_swerveModules) {
            module.setCoastMode();
        }
    }

    // Create a command to stop driving
    public void stopDriving() {
        for (CatzSwerveModule module : m_swerveModules) {
            module.stopDriving();
            module.setSteerPower(0.0);
        }
    }

    //command to cancel running auto trajectories
    public Command cancelTrajectory() {
        return new InstantCommand();
    }

    public Command cmdSetAutonSlowdown(boolean state){
        return runOnce(()->isAutonSlowedDown = state);
    }

    public FieldRelativeSpeed getFieldRelativeSpeed() {
        return m_fieldRelVel;
      }
    
      public FieldRelativeAccel getFieldRelativeAccel() {
        return m_fieldRelAccel;
      }
    //----------------------------------------------Gyro methods----------------------------------------------


    //TBD do not use unless autoaim does not work anymore
    public void flipGyro() {
        gyroIO.setAngleAdjustmentIO(180);
    }

    public Command resetGyro() {
        return runOnce(() -> {
            if(CatzAutonomous.getInstance().getAllianceColor() == AllianceColor.Red){
                gyroIO.setAngleAdjustmentIO(-gyroInputs.gyroYaw + 180);
            }else{
                gyroIO.setAngleAdjustmentIO(-gyroInputs.gyroYaw);
            }
        });
    }

    // Get the gyro angle (negative due to the weird coordinate system)
    public double getGyroAngle() {
        return -gyroInputs.gyroAngle; //- for atlas
    }

    // Get the roll angle of the gyro
    public double getRollAngle() {
        return gyroInputs.gyroRoll;
    }

    // Get the heading of the robot in a integer quantity
    public double getHeading() {
        return Math.IEEEremainder(getGyroAngle(), 360);
    }

    // Get the heading of the robot in radians
    public double getHeadingRadians() {
        return (getHeading() * Math.PI / 180);
    }

    // Get the Rotation2d object based on the gyro angle
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getGyroAngle());
    }

    // Reset the position of the robot with a given pose
    public void resetPosition(Pose2d pose) {
        double angle = pose.getRotation().getDegrees();

        gyroIO.setAngleAdjustmentIO(angle);
        pose = new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(angle));
        m_poseEstimator.resetPosition(Rotation2d.fromDegrees(angle),getModulePositions(),pose);
    }
 
    // Get the current pose of the robot
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public Command zeroPoseEstimatorCmd() {
        return runOnce(()->resetPosition(new Pose2d(0.0,0.0, Rotation2d.fromDegrees(0))));
    }

    //----------------------------------------------Enc resets-------------------------------------------------------

    // Reset drive encoders for all swerve modules
    public void resetDriveEncs() {
        for (CatzSwerveModule module : m_swerveModules) {
            module.resetDriveEncs();
        }
    }

    // Get an array of swerve module states
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < m_swerveModules.length; i++) {
            moduleStates[i] = m_swerveModules[i].getModuleState();
        }
        return moduleStates;
    }

    // Get an array of swerve module positions
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < m_swerveModules.length; i++) {
            modulePositions[i] = m_swerveModules[i].getModulePosition();
        }
        return modulePositions;
    }

    //----------------------------------------vision-----------------------------------------
    public Command toggleVisionEnableCommand() {
        if(isVisionEnabled == true) {
            return run(()-> setVisionEnable(false));
        }
        else {
            return run(()-> setVisionEnable(true));
        }
    }

    //access method for determining whether to use vision in pose estimation
    private void setVisionEnable(boolean enable) {
        isVisionEnabled = enable;
    }

}